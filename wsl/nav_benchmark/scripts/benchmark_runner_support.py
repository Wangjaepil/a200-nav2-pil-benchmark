#!/usr/bin/env python3
"""PC/Pi process control, readiness probes, and artifact helpers.

The orchestration policy lives in benchmark_runner.py. This module owns the
mechanics used by that policy so process and transport changes stay isolated.
"""
from __future__ import annotations

import math
import os
import re
import shlex
import shutil
import signal
import socket
import subprocess
import sys
import threading
import time
from datetime import datetime
from pathlib import Path
from queue import Empty, Queue

try:
    import yaml
except ImportError:
    sys.exit("PyYAML is required: sudo apt install python3-yaml")

from benchmark_common import Pose2D, SUITE_VERSION, pose_error


HOME = Path.home()
ROOT = HOME / "nav_benchmark"
SCRIPTS = ROOT / "scripts"
CASES = ROOT / "cases"
RESULTS = ROOT / "results"
RUNNER_LOG_ROOT = ROOT / "runner_logs"

RUN_CASE = SCRIPTS / "run_case.py"
LOGGER = SCRIPTS / "benchmark_logger.py"
SEND_GOAL = SCRIPTS / "send_case_goal.py"

PI_USER = os.environ.get("NAV_BENCH_PI_USER", "tb-pil")
PI_HOST = os.environ.get("NAV_BENCH_PI_HOST", "").strip()
PI_TARGET = f"{PI_USER}@{PI_HOST}"
PI_MANAGER = "~/nav_benchmark/bin/pi_stack_manager.sh"

ZENOH_ENDPOINT = os.environ.get(
    "NAV_BENCH_ZENOH_ENDPOINT",
    "",
).strip() or (f"tcp/{PI_HOST}:7447" if PI_HOST else "")

# Keep all PC-local ROS traffic on the PC.  Only the local router-to-router
# uplink crosses Wi-Fi to the Pi.  Port 7448 deliberately avoids a manually
# started/default router on 7447.
PC_ZENOH_PORT = int(os.environ.get("NAV_BENCH_PC_ZENOH_PORT", "7448"))
PC_ZENOH_ENDPOINT = f"tcp/127.0.0.1:{PC_ZENOH_PORT}"

SSH_BASE = [
    "ssh",
    "-o", "BatchMode=yes",
    "-o", "ConnectTimeout=6",
    "-o", "ServerAliveInterval=5",
    "-o", "ServerAliveCountMax=3",
    # Reuse one authenticated TCP connection for the many small manager and
    # artifact commands in a case. This removes thousands of Wi-Fi handshakes
    # from a full 120-case batch.
    "-o", "ControlMaster=auto",
    "-o", "ControlPersist=300",
    "-o", "ControlPath=/tmp/nav_benchmark_ssh_%C",
    PI_TARGET,
]

SSH_TRANSPORT_ATTEMPTS = 3
SSH_RETRY_BACKOFF_SEC = (1.0, 2.0)
SSH_TRANSPORT_ERROR_MARKERS = (
    "connection timed out during banner exchange",
    "ssh: connect to host ",
    "connection reset by peer",
    "connection closed by ",
    "connection refused",
    "no route to host",
    "network is unreachable",
    "kex_exchange_identification",
)

PC_ENV_PREFIX = (
    "source /opt/ros/jazzy/setup.bash; "
    "export RMW_IMPLEMENTATION=rmw_zenoh_cpp; "
    "export ROS_DOMAIN_ID=10; "
    f"export ZENOH_CONFIG_OVERRIDE={shlex.quote('mode=\"client\";connect/endpoints=[\"' + PC_ZENOH_ENDPOINT + '\"]')}; "
)

PC_ROUTER_ENV_PREFIX = (
    "source /opt/ros/jazzy/setup.bash; "
    "export ROS_DOMAIN_ID=10; "
    "export RUST_LOG=zenoh=info; "
    f"export ZENOH_CONFIG_OVERRIDE={shlex.quote('listen/endpoints=[\"' + PC_ZENOH_ENDPOINT + '\"];connect/endpoints=[\"' + ZENOH_ENDPOINT + '\"]')}; "
)

TF_ARGS = [
    "-r", "/tf:=/a200_0000/tf",
    "-r", "/tf_static:=/a200_0000/tf_static",
]

BRIDGE_ARGS = [
    "ros2", "run", "ros_gz_bridge", "parameter_bridge",
    "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
    "/gps/front@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat",
    "/gps/rear@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat",
]

SENSOR_TOPICS = [
    "/clock",
    "/scan",
    "/gps/front",
    "/gps/rear",
    "/a200_0000/platform/odom",
]

COSTMAP_TOPICS = [
    "/local_costmap/costmap",
    "/global_costmap/costmap",
]

LIFECYCLE_NODES = [
    "controller_server",
    "planner_server",
    "behavior_server",
    "collision_monitor",
    "bt_navigator",
]


PRE_GOAL_CMD_TOPICS = (
    "/cmd_vel_raw",
    "/a200_0000/platform/cmd_vel",
)
PRE_GOAL_CMD_EPS = 1e-4
PLATFORM_ODOM_TOPIC = "/a200_0000/platform/odom"
PRE_GOAL_HOLD_RATE_HZ = 50
PC_BENCHMARK_ORPHAN_SIGNATURES = (
    str(RUN_CASE),
    str(LOGGER),
    str(SEND_GOAL),
    "ros2 run ros_gz_bridge parameter_bridge",
    "ros2 launch clearpath_gz gz_sim.launch.py",
    "ros2 launch clearpath_gz robot_spawn.launch.py",
    "ros2 topic pub --rate 50 /a200_0000/platform/cmd_vel",
)

PI_LOG_NAMES = [
    "dual_gps_heading",
    "global_ekf",
    "navsat_transform",
    "controller_server",
    "planner_server",
    "behavior_server",
    "collision_monitor",
    "bt_navigator",
    "far_goal",
]


class InfraError(RuntimeError):
    def __init__(self, stage: str, message: str):
        super().__init__(message)
        self.stage = stage
        self.message = message


def log_tail(path: "Path | None", limit: int = 5000) -> str:
    """Return the tail of a process log for an error message."""
    if path is None:
        return "(no diagnostic log was provided)"
    path = Path(path)
    if not path.exists():
        return f"(diagnostic log not found: {path})"
    try:
        body = path.read_text(encoding="utf-8", errors="replace")
    except OSError as exc:
        return f"(could not read {path}: {exc})"
    if not body.strip():
        return f"(diagnostic log is empty: {path})"
    return f"--- tail of {path} ---\n{body[-limit:]}"


def ts() -> str:
    return datetime.now().strftime("%Y-%m-%d %H:%M:%S")


def slug_ts() -> str:
    return datetime.now().strftime("%Y%m%d_%H%M%S")


def print_stage(number: int, text: str) -> None:
    print()
    print("=" * 72)
    print(f"[{number:02d}] {text}")
    print("=" * 72, flush=True)


def shell_join(args) -> str:
    return shlex.join([str(x) for x in args])


def pc_shell(command: str) -> list[str]:
    return ["bash", "-lc", PC_ENV_PREFIX + command]


def run_pc(
    args,
    *,
    timeout: float | None = None,
    check: bool = True,
) -> subprocess.CompletedProcess:
    cmd = pc_shell(shell_join(args))
    cp = subprocess.run(
        cmd,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        timeout=timeout,
    )
    if check and cp.returncode != 0:
        raise RuntimeError(
            f"PC command failed ({cp.returncode}): {shell_join(args)}\n"
            f"{cp.stdout}"
        )
    return cp


def start_pc_process(
    name: str,
    args,
    log_path: Path,
) -> subprocess.Popen:
    log_path.parent.mkdir(parents=True, exist_ok=True)
    log_fp = log_path.open("w", encoding="utf-8")
    full = pc_shell("exec " + shell_join(args))
    p = subprocess.Popen(
        full,
        stdout=log_fp,
        stderr=subprocess.STDOUT,
        text=True,
        start_new_session=True,
    )
    # Parent does not need the descriptor; child inherited it.
    log_fp.close()
    print(f"[STARTED][PC] {name}: PID={p.pid}")
    print(f"              log={log_path}")
    return p


def start_pc_zenoh_router(log_path: Path) -> subprocess.Popen:
    """Start the benchmark's PC-local router and link it to the Pi router."""
    log_path.parent.mkdir(parents=True, exist_ok=True)
    log_fp = log_path.open("w", encoding="utf-8")
    command = [
        "bash",
        "-lc",
        PC_ROUTER_ENV_PREFIX
        + "exec ros2 run rmw_zenoh_cpp rmw_zenohd",
    ]
    proc = subprocess.Popen(
        command,
        stdout=log_fp,
        stderr=subprocess.STDOUT,
        text=True,
        start_new_session=True,
    )
    log_fp.close()
    print(f"[STARTED][PC] local_zenoh_router: PID={proc.pid}")
    print(f"              local={PC_ZENOH_ENDPOINT}")
    print(f"              uplink={ZENOH_ENDPOINT}")
    print(f"              log={log_path}")
    return proc


def wait_pc_zenoh_router(
    proc: subprocess.Popen,
    log_path: Path,
    *,
    timeout_sec: float = 12.0,
) -> None:
    """Wait until the PC-local router accepts TCP client connections."""
    deadline = time.monotonic() + timeout_sec

    while time.monotonic() < deadline:
        rc = proc.poll()
        if rc is not None:
            tail = ""
            if log_path.exists():
                tail = log_path.read_text(
                    encoding="utf-8", errors="replace"
                )[-4000:]
            raise RuntimeError(
                f"PC local Zenoh router exited early (rc={rc}).\n{tail}"
            )

        try:
            with socket.create_connection(
                ("127.0.0.1", PC_ZENOH_PORT),
                timeout=0.4,
            ):
                print("[READY][PC] local Zenoh router")
                return
        except OSError:
            time.sleep(0.2)

    raise RuntimeError(
        "PC local Zenoh router did not open "
        f"127.0.0.1:{PC_ZENOH_PORT} within {timeout_sec:.0f}s."
    )


def assert_local_router_port_free() -> None:
    """Fail clearly instead of accidentally using a stale local router."""
    probe = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        probe.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        probe.bind(("127.0.0.1", PC_ZENOH_PORT))
    except OSError as exc:
        raise SystemExit(
            f"PC Zenoh port 127.0.0.1:{PC_ZENOH_PORT} is already in use.\n"
            "Stop the stale local rmw_zenohd process, then retry.\n"
            f"Details: {exc}"
        ) from exc
    finally:
        probe.close()


def assert_pi_zenoh_port(timeout_sec: float = 2.0) -> None:
    """Verify the external Pi hil_router before launching the local router."""
    try:
        with socket.create_connection((PI_HOST, 7447), timeout=timeout_sec):
            pass
    except OSError as exc:
        raise SystemExit(
            f"Pi hil_router is not reachable at {PI_HOST}:7447.\n"
            "Start hil_router on the Pi and verify the network before running.\n"
            f"Details: {exc}"
        ) from exc
    print(f"[OK] Pi hil_router TCP endpoint -> {PI_HOST}:7447")


def stop_pc_process(
    name: str,
    proc: subprocess.Popen | None,
    *,
    grace: float = 10.0,
) -> None:
    if proc is None or proc.poll() is not None:
        return

    print(f"[STOP][PC] {name}: SIGINT")
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
    except ProcessLookupError:
        return

    deadline = time.monotonic() + grace
    while time.monotonic() < deadline:
        if proc.poll() is not None:
            print(f"[STOPPED][PC] {name}")
            return
        time.sleep(0.2)

    print(f"[STOP][PC] {name}: SIGTERM")
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
    except ProcessLookupError:
        return

    deadline = time.monotonic() + 4.0
    while time.monotonic() < deadline:
        if proc.poll() is not None:
            print(f"[STOPPED][PC] {name}")
            return
        time.sleep(0.2)

    print(f"[WARN][PC] {name}: SIGKILL")
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
    except ProcessLookupError:
        pass


def _is_ssh_transport_failure(cp: subprocess.CompletedProcess) -> bool:
    """Return True only for connection-level SSH failures safe to retry."""
    if cp.returncode != 255:
        return False

    text = (cp.stdout or "").lower()
    return any(marker in text for marker in SSH_TRANSPORT_ERROR_MARKERS)


def remote(
    command: str,
    *,
    timeout: float = 60.0,
    check=True,
    show_output=True,
    transport_attempts: int = SSH_TRANSPORT_ATTEMPTS,
):
    # Execute through a login shell so ~ expands on the Pi.
    # Retry only SSH transport establishment failures (rc=255 + known
    # connection error).  Do NOT retry a real remote-command failure and do
    # NOT retry subprocess.TimeoutExpired, because the remote command may
    # already have started and repeating it could duplicate side effects.
    remote_cmd = f"bash -lc {shlex.quote(command)}"
    attempts = max(1, int(transport_attempts))
    last_cp = None

    for attempt in range(1, attempts + 1):
        cp = subprocess.run(
            SSH_BASE + [remote_cmd],
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            timeout=timeout,
        )
        last_cp = cp

        if cp.returncode == 0 or not _is_ssh_transport_failure(cp):
            break

        if attempt >= attempts:
            break

        delay_index = min(attempt - 1, len(SSH_RETRY_BACKOFF_SEC) - 1)
        delay = SSH_RETRY_BACKOFF_SEC[delay_index]
        print(
            f"[WARN] SSH transport failure to {PI_TARGET}; "
            f"retry {attempt + 1}/{attempts} in {delay:.1f}s"
        )
        if cp.stdout:
            print(cp.stdout.rstrip())
        time.sleep(delay)

    assert last_cp is not None
    cp = last_cp

    if show_output and cp.stdout:
        print(cp.stdout.rstrip())
    if check and cp.returncode != 0:
        raise RuntimeError(
            f"Pi command failed ({cp.returncode}): {command}\n{cp.stdout}"
        )
    return cp


def manager(action: str, *, timeout: float = 90.0):
    return remote(f"{PI_MANAGER} {shlex.quote(action)}", timeout=timeout)


def wait_log_marker(
    proc: subprocess.Popen,
    log_path: Path,
    marker: str,
    *,
    timeout: float,
    stage: str,
) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if log_path.exists():
            text = log_path.read_text(
                encoding="utf-8", errors="replace"
            )
            if marker in text:
                return

        rc = proc.poll()
        if rc is not None:
            tail = ""
            if log_path.exists():
                tail = log_path.read_text(
                    encoding="utf-8", errors="replace"
                )[-4000:]
            raise InfraError(
                stage,
                f"Process exited before marker '{marker}' "
                f"(rc={rc}).\n{tail}",
            )
        time.sleep(0.25)

    tail = ""
    if log_path.exists():
        tail = log_path.read_text(
            encoding="utf-8", errors="replace"
        )[-4000:]
    raise InfraError(
        stage,
        f"Timeout waiting for '{marker}'.\n{tail}",
    )


def _topic_message_seen(topic: str, output: str) -> bool:
    """Return True only when output contains fields from a real message."""
    if not output or not output.strip():
        return False

    markers = {
        "/clock": ("clock:",),
        "/scan": ("header:", "ranges:"),
        "/gps/front": ("latitude:", "longitude:"),
        "/gps/rear": ("latitude:", "longitude:"),
        "/a200_0000/platform/odom": ("pose:", "twist:"),
        "/local_costmap/costmap": ("info:", "data:"),
        "/global_costmap/costmap": ("info:", "data:"),
    }
    required = markers.get(topic)
    if required is None:
        return "---" in output or "header:" in output

    return all(marker in output for marker in required)


def _probe_topic_once(topic: str, probe_sec: float = 8.0) -> str:
    """
    Run one topic probe in its own process group.

    rmw_zenoh discovery and ros2 CLI teardown can both take a few seconds.
    Killing only the parent shell can leave orphaned `ros2 topic echo`
    processes behind, so this probe owns a process group and always cleans
    the whole group up.
    """
    args = [
        "ros2", "topic", "echo", topic,
        "--once",
        "--qos-reliability", "best_effort",
    ]
    p = subprocess.Popen(
        pc_shell("exec " + shell_join(args)),
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        start_new_session=True,
    )

    try:
        out, _ = p.communicate(timeout=probe_sec)
        return out or ""

    except subprocess.TimeoutExpired:
        try:
            os.killpg(os.getpgid(p.pid), signal.SIGINT)
        except ProcessLookupError:
            pass

        try:
            out, _ = p.communicate(timeout=2.0)
            return out or ""
        except subprocess.TimeoutExpired:
            try:
                os.killpg(os.getpgid(p.pid), signal.SIGKILL)
            except ProcessLookupError:
                pass
            out, _ = p.communicate()
            return out or ""


def wait_topic_once(
    topic: str,
    timeout_sec: float = 45.0,
    *,
    stage: str = "SENSOR_READINESS",
) -> None:
    deadline = time.monotonic() + timeout_sec
    last_output = ""

    while time.monotonic() < deadline:
        remaining = deadline - time.monotonic()
        probe_sec = min(8.0, max(1.0, remaining))
        last_output = _probe_topic_once(topic, probe_sec=probe_sec)

        if _topic_message_seen(topic, last_output):
            print(f"[READY][PC] topic {topic}")
            return

        time.sleep(0.5)

    raise InfraError(
        stage,
        f"No fresh message received on {topic} within "
        f"{timeout_sec:.0f}s. Last output:\n{last_output[-2000:]}",
    )


def zero_hold_command():
    """Continuously hold the physical platform command at zero during setup."""
    msg = (
        "{header: {frame_id: ''}, twist: {linear: {x: 0.0, y: 0.0, z: 0.0}, "
        "angular: {x: 0.0, y: 0.0, z: 0.0}}}"
    )
    return [
        "ros2", "topic", "pub",
        "--rate", str(PRE_GOAL_HOLD_RATE_HZ),
        "/a200_0000/platform/cmd_vel",
        "geometry_msgs/msg/TwistStamped",
        msg,
    ]


def start_pre_goal_zero_hold(log_path: Path) -> subprocess.Popen:
    """Safety interlock: no physical motion is allowed before Stage 11."""
    proc = start_pc_process(
        "pre_goal_zero_hold",
        zero_hold_command(),
        log_path,
    )
    # Give ROS discovery time and ensure the publisher did not die immediately.
    time.sleep(1.0)
    if proc.poll() is not None:
        tail = ""
        if log_path.exists():
            tail = log_path.read_text(encoding="utf-8", errors="replace")[-3000:]
        raise InfraError(
            "PRE_GOAL_ZERO_HOLD",
            "Failed to start the pre-goal zero-velocity safety interlock.\n" + tail,
        )
    print("[SAFETY] Pre-goal platform zero hold ACTIVE")
    return proc


def stop_pre_goal_zero_hold(proc: subprocess.Popen | None) -> None:
    if proc is None:
        return
    stop_pc_process("pre_goal_zero_hold", proc, grace=2.0)
    print("[SAFETY] Pre-goal platform zero hold RELEASED")


def force_platform_zero_burst(label: str, duration_sec: float = 0.8) -> None:
    """Best-effort zero burst used on cleanup and before setup transitions."""
    log_path = RUNNER_LOG_ROOT / ".zero_burst.log"
    proc = start_pc_process("zero_burst", zero_hold_command(), log_path)
    try:
        time.sleep(max(0.2, duration_sec))
    finally:
        stop_pc_process("zero_burst", proc, grace=1.0)
    print(f"[SAFETY] Zero velocity burst: {label}")


def cleanup_pc_benchmark_orphans() -> None:
    """Remove stale PC-side benchmark/Gazebo process groups from crashed runs."""
    try:
        cp = subprocess.run(
            ["ps", "-eo", "pid=,pgid=,args="],
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            timeout=5.0,
            check=False,
        )
    except Exception as exc:
        raise InfraError("PC_ORPHAN_CLEANUP", f"Failed to inspect PC processes: {exc}")

    own_pid = os.getpid()
    own_pgid = os.getpgid(own_pid)
    matched_pgids = set()
    matched_lines = []
    for raw in (cp.stdout or "").splitlines():
        parts = raw.strip().split(None, 2)
        if len(parts) < 3:
            continue
        try:
            pid, pgid = int(parts[0]), int(parts[1])
        except ValueError:
            continue
        args = parts[2]
        if pid == own_pid or pgid == own_pgid:
            continue
        if any(signature in args for signature in PC_BENCHMARK_ORPHAN_SIGNATURES):
            matched_pgids.add(pgid)
            matched_lines.append(f"pid={pid} pgid={pgid} {args}")

    if not matched_pgids:
        print("[READY] No stale PC benchmark/Gazebo processes")
        return

    print("[CLEANUP] Stale PC benchmark/Gazebo processes detected:")
    for line in matched_lines:
        print("          " + line)

    for sig, wait_sec in ((signal.SIGINT, 1.5), (signal.SIGTERM, 1.0), (signal.SIGKILL, 0.2)):
        for pgid in list(matched_pgids):
            try:
                os.killpg(pgid, sig)
            except ProcessLookupError:
                matched_pgids.discard(pgid)
            except PermissionError as exc:
                raise InfraError("PC_ORPHAN_CLEANUP", f"Cannot signal PGID {pgid}: {exc}")
        time.sleep(wait_sec)
        # A terminated child may remain briefly as a zombie until its parent
        # reaps it.  Zombies cannot execute or publish ROS traffic, so do not
        # treat a Z-only process group as a surviving benchmark process.
        try:
            ps_state = subprocess.run(
                ["ps", "-eo", "pgid=,stat="],
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL,
                timeout=3.0,
                check=False,
            ).stdout or ""
        except Exception:
            ps_state = ""

        non_zombie_pgids = set()
        for raw_state in ps_state.splitlines():
            fields = raw_state.strip().split(None, 1)
            if len(fields) != 2:
                continue
            try:
                state_pgid = int(fields[0])
            except ValueError:
                continue
            stat = fields[1].strip()
            if state_pgid in matched_pgids and not stat.startswith("Z"):
                non_zombie_pgids.add(state_pgid)

        matched_pgids = non_zombie_pgids
        if not matched_pgids:
            break

    if matched_pgids:
        raise InfraError(
            "PC_ORPHAN_CLEANUP",
            "Stale PC benchmark process groups survived cleanup: "
            + ", ".join(map(str, sorted(matched_pgids))),
        )
    print("[READY] Stale PC benchmark/Gazebo processes removed")


def assert_platform_odom_stationary(label: str, sample_sec: float = 5.0) -> None:
    """Check the robot's actual reported velocity, not only command topics."""
    out = _probe_topic_once(PLATFORM_ODOM_TOPIC, probe_sec=sample_sec)
    twist = _parse_odometry_twist(out)
    if twist is None:
        raise InfraError(
            "PRE_GOAL_ODOM",
            f"Could not read platform odometry velocity at checkpoint: {label}\n"
            f"Last odom probe output:\n{out[-2500:]}",
        )
    vx, wz = twist
    # Simulation odometry should be effectively zero before a goal.  Keep a
    # small tolerance for numerical noise, not for actual setup motion.
    if abs(vx) > 0.02 or abs(wz) > 0.03:
        raise InfraError(
            "PRE_GOAL_ODOM",
            f"Robot is physically moving before goal at {label}: "
            f"odom vx={vx:.4f} m/s, wz={wz:.4f} rad/s\n"
            + _topic_info_verbose("/a200_0000/platform/cmd_vel")[-5000:],
        )
    print(
        f"[READY] Platform odom stationary ({label}): "
        f"vx={vx:.4f}, wz={wz:.4f}"
    )


def wait_file_marker(
    path: Path,
    marker: str,
    *,
    proc: subprocess.Popen | None,
    timeout_sec: float,
    stage: str,
    diagnostic_log: Path | None = None,
) -> None:
    """Wait for a file marker written by a long-lived local process."""
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        if path.exists():
            try:
                body = path.read_text(encoding="utf-8", errors="replace")
            except OSError:
                body = ""
            if marker in body:
                return

        if proc is not None:
            rc = proc.poll()
            if rc is not None:
                tail = ""
                if diagnostic_log is not None and diagnostic_log.exists():
                    tail = diagnostic_log.read_text(
                        encoding="utf-8", errors="replace"
                    )[-5000:]
                raise InfraError(
                    stage,
                    f"Process exited before marker '{marker}' (rc={rc}).\n{tail}",
                )
        time.sleep(0.1)

    tail = ""
    if diagnostic_log is not None and diagnostic_log.exists():
        tail = diagnostic_log.read_text(
            encoding="utf-8", errors="replace"
        )[-5000:]
    raise InfraError(
        stage,
        f"Timeout waiting for marker '{marker}' in {path}.\n{tail}",
    )


def _parse_odometry_twist(output: str) -> tuple[float, float] | None:
    """Extract twist.twist.linear.x / angular.z from nav_msgs/Odometry echo."""
    if not output or not output.strip():
        return None

    # ros2 topic echo renders nav_msgs/Odometry as YAML.  Parse the message
    # structure directly instead of reusing the TwistStamped parser: Odometry
    # nests velocity under twist.twist.
    try:
        body = output
        # Ignore a leading ros2/rmw diagnostic line if one was merged from
        # stderr.  A real Odometry message starts at header:.
        header_pos = body.find("header:")
        if header_pos >= 0:
            body = body[header_pos:]
        data = yaml.safe_load(body)
        twist = data["twist"]["twist"]
        return (
            float(twist["linear"]["x"]),
            float(twist["angular"]["z"]),
        )
    except (TypeError, ValueError, KeyError, yaml.YAMLError):
        pass

    # Fallback for harmless formatting changes in ros2 CLI output.  Require
    # the nested `twist -> twist` section so this cannot accidentally parse
    # pose/covariance data as velocity.
    nested = re.search(
        r"(?ms)^\s*twist:\s*\n\s*twist:\s*\n(?P<body>.*?)(?=^\s*covariance:|\Z)",
        output,
    )
    if nested is None:
        return None
    body = nested.group("body")
    linear = re.search(
        r"(?ms)^\s*linear:\s*\n\s*x:\s*([-+0-9.eE]+)",
        body,
    )
    angular = re.search(
        r"(?ms)^\s*angular:\s*\n(?:\s*x:\s*[-+0-9.eE]+\s*\n)?"
        r"(?:\s*y:\s*[-+0-9.eE]+\s*\n)?\s*z:\s*([-+0-9.eE]+)",
        body,
    )
    if linear is None or angular is None:
        return None
    try:
        return float(linear.group(1)), float(angular.group(1))
    except ValueError:
        return None


def _parse_twist_stamped(output: str) -> tuple[float, float] | None:
    """Extract linear.x / angular.z from `ros2 topic echo` TwistStamped text."""
    if not output or not output.strip():
        return None

    linear = re.search(
        r"(?ms)^\s*linear:\s*\n\s*x:\s*([-+0-9.eE]+)",
        output,
    )
    angular = re.search(
        r"(?ms)^\s*angular:\s*\n(?:\s*x:\s*[-+0-9.eE]+\s*\n)?"
        r"(?:\s*y:\s*[-+0-9.eE]+\s*\n)?\s*z:\s*([-+0-9.eE]+)",
        output,
    )
    if linear is None or angular is None:
        return None

    try:
        return float(linear.group(1)), float(angular.group(1))
    except ValueError:
        return None


def _topic_info_verbose(topic: str) -> str:
    try:
        cp = run_pc(
            ["ros2", "topic", "info", topic, "-v"],
            timeout=5.0,
            check=False,
        )
        return cp.stdout or ""
    except Exception as exc:
        return f"topic info failed: {exc!r}"


_TF_TRANSLATION_RE = re.compile(
    r"Translation:\s*\[\s*([-+0-9.eE]+)\s*,\s*"
    r"([-+0-9.eE]+)\s*,\s*([-+0-9.eE]+)\s*\]"
)
_TF_RPY_RE = re.compile(
    r"RPY\s*\(radian\)\s*\[\s*([-+0-9.eE]+)\s*,\s*"
    r"([-+0-9.eE]+)\s*,\s*([-+0-9.eE]+)\s*\]"
)


def _parse_tf2_echo_pose(output: str) -> Pose2D | None:
    translation = _TF_TRANSLATION_RE.search(output or "")
    rpy = _TF_RPY_RE.search(output or "")
    if translation is None or rpy is None:
        return None
    try:
        return Pose2D(
            float(translation.group(1)),
            float(translation.group(2)),
            float(rpy.group(3)),
        )
    except ValueError:
        return None


def _terminate_probe(process: subprocess.Popen, grace: float = 1.0) -> None:
    """Tear down a probe's whole process group."""
    if process.poll() is not None:
        return
    for sig in (signal.SIGINT, signal.SIGKILL):
        try:
            os.killpg(os.getpgid(process.pid), sig)
        except ProcessLookupError:
            return
        try:
            process.wait(timeout=grace)
            return
        except subprocess.TimeoutExpired:
            continue


def _probe_map_pose(probe_sec: float = 8.0) -> tuple[Pose2D | None, str]:
    """Run one tf2_echo, returning as soon as a full transform block parses.

    tf2_echo never exits on its own, so this used to block for the entire
    window and only then parse.  A fresh `ros2 run` node spends well over a
    second starting up and joining the Zenoh session before it can receive
    any /tf, so a short window left under a second of real listening time:
    the 2.0s stability probe reported

        Invalid frame ID "map" ... frame does not exist

    even while TF was perfectly healthy, and stage 5 failed with
    LOCALIZATION_STABILITY.  Read incrementally instead - return the moment
    a pose is available, and spend the full window only when nothing comes.
    """
    args = [
        "ros2", "run", "tf2_ros", "tf2_echo",
        "map", "base_link",
        "--ros-args",
        *TF_ARGS,
    ]
    process = subprocess.Popen(
        pc_shell("exec " + shell_join(args)),
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        start_new_session=True,
    )

    lines = []
    line_queue = Queue()

    def _reader():
        try:
            if process.stdout is not None:
                for line in process.stdout:
                    line_queue.put(line)
        except (OSError, ValueError):
            pass
        line_queue.put(None)

    threading.Thread(target=_reader, daemon=True).start()

    pose = None
    deadline = time.monotonic() + max(1.0, float(probe_sec))
    while time.monotonic() < deadline:
        try:
            line = line_queue.get(timeout=0.2)
        except Empty:
            continue
        if line is None:
            break
        lines.append(line)
        pose = _parse_tf2_echo_pose("".join(lines))
        if pose is not None:
            break

    _terminate_probe(process)

    # Keep whatever is still queued so the caller reports full diagnostics.
    while True:
        try:
            line = line_queue.get_nowait()
        except Empty:
            break
        if line is None:
            break
        lines.append(line)

    output = "".join(lines)
    return pose, output


def assert_map_pose_unchanged(
    reference: Pose2D,
    label: str,
    *,
    max_xy_m: float = 0.05,
    max_yaw_deg: float = 3.0,
) -> Pose2D:
    current, output = _probe_map_pose()
    if current is None:
        raise InfraError(
            "PRE_GOAL_POSE",
            f"Could not read map->base_link at checkpoint: {label}\n"
            + output[-3000:],
        )
    xy_error, yaw_error = pose_error(current, reference)
    if xy_error > max_xy_m or math.degrees(yaw_error) > max_yaw_deg:
        raise InfraError(
            "PRE_GOAL_POSE",
            f"Robot pose changed before goal at {label}: "
            f"xy={xy_error:.3f}m yaw={math.degrees(yaw_error):.2f}deg; "
            f"limits={max_xy_m:.3f}m/{max_yaw_deg:.1f}deg",
        )
    print(
        f"[READY] Pre-goal pose unchanged ({label}): "
        f"xy={xy_error:.3f}m yaw={math.degrees(yaw_error):.2f}deg"
    )
    return current


def assert_pre_goal_stationary(label: str, sample_sec: float = 5.0) -> None:
    """
    Before /far_goal_pose is sent, no navigation command may move the robot.

    A setup-stage non-zero command means a stale/duplicate publisher survived
    cleanup.  Fail immediately and print publisher diagnostics instead of
    allowing the benchmark to start in a contaminated state.
    """
    bad = []
    observations = []

    for topic in PRE_GOAL_CMD_TOPICS:
        out = _probe_topic_once(topic, probe_sec=sample_sec)
        twist = _parse_twist_stamped(out)
        if twist is None:
            observations.append(f"{topic}: no fresh command observed")
            continue

        vx, wz = twist
        observations.append(f"{topic}: vx={vx:.6f}, wz={wz:.6f}")
        if abs(vx) > PRE_GOAL_CMD_EPS or abs(wz) > PRE_GOAL_CMD_EPS:
            bad.append((topic, vx, wz))

    if not bad:
        print(f"[READY] Pre-goal motion guard ({label}): stationary")
        for line in observations:
            print(f"        {line}")
        return

    diagnostics = []
    for topic, vx, wz in bad:
        diagnostics.append(
            f"NON-ZERO before goal: {topic} vx={vx:.6f}, wz={wz:.6f}\n"
            + _topic_info_verbose(topic)[-5000:]
        )

    raise InfraError(
        "PRE_GOAL_MOTION",
        "Robot command became non-zero before /far_goal_pose was sent.\n"
        f"Checkpoint: {label}\n\n"
        + "\n\n".join(diagnostics),
    )


def wait_map_to_base(timeout_sec: float = 45.0) -> Pose2D:
    deadline = time.monotonic() + timeout_sec
    last_output = ""

    while time.monotonic() < deadline:
        remaining = deadline - time.monotonic()
        pose, out = _probe_map_pose(probe_sec=min(8.0, max(2.0, remaining)))

        last_output = out
        if pose is not None:
            print(
                "[READY] map -> base_link TF: "
                f"x={pose.x:.3f}, y={pose.y:.3f}, yaw={pose.yaw:.4f}"
            )
            return pose

        time.sleep(0.4)

    raise InfraError(
        "LOCALIZATION_READINESS",
        "map -> base_link did not become available.\n"
        + last_output[-3000:],
    )


def wait_stable_map_pose(
    initial: Pose2D,
    *,
    timeout_sec: float = 40.0,
    max_xy_change_m: float = 0.05,
    max_yaw_change_deg: float = 3.0,
) -> Pose2D:
    """Wait for two consecutive localization poses to agree.

    The probe window used to be 2.0s.  Node startup alone eats more than a
    second of that, so every probe here failed to read TF at all while the
    3.0s probe in wait_map_to_base() right above succeeded - the stage failed
    with "frame does not exist" on a healthy stack.  _probe_map_pose() now
    returns as soon as a pose parses, so a generous window costs nothing when
    localization is actually publishing.
    """
    deadline = time.monotonic() + timeout_sec
    previous = initial
    last_detail = ""
    reads = 0
    misses = 0

    while time.monotonic() < deadline:
        time.sleep(0.5)
        remaining = deadline - time.monotonic()
        if remaining <= 0.0:
            break

        current, output = _probe_map_pose(
            probe_sec=min(8.0, max(2.0, remaining))
        )
        if current is None:
            misses += 1
            last_detail = output[-2000:]
            continue

        reads += 1
        xy_change, yaw_change = pose_error(current, previous)
        last_detail = (
            f"xy_change={xy_change:.3f}m, "
            f"yaw_change={math.degrees(yaw_change):.2f}deg"
        )
        if (
            xy_change <= max_xy_change_m
            and math.degrees(yaw_change) <= max_yaw_change_deg
        ):
            print(f"[READY] Localization pose stable: {last_detail}")
            return current
        previous = current

    raise InfraError(
        "LOCALIZATION_STABILITY",
        "map->base_link did not settle before Nav2 startup "
        f"(pose reads={reads}, probes that read nothing={misses}). "
        "A high miss count means TF was not readable, not that the robot "
        "was moving.\n" + last_detail,
    )


def verify_nav2_active() -> None:
    # Ask the Pi manager for a machine-verifiable readiness decision instead
    # of re-parsing its human-readable `status` table on the PC.
    cp = remote(
        f"{PI_MANAGER} verify",
        timeout=30,
        check=False,
    )
    output = cp.stdout or ""

    if cp.returncode != 0 or "NAV2_READY" not in output.splitlines():
        raise InfraError(
            "NAV2_ACTIVATION",
            "Pi manager verification failed.\n" + output[-3000:],
        )

    print("[READY] Nav2 lifecycle ACTIVE + far_goal RUNNING")


def current_run_dirs(case_id: str) -> set[Path]:
    scenario = case_id.split("_", 1)[0]
    base = RESULTS / scenario / case_id
    if not base.exists():
        return set()
    return {
        p.resolve()
        for p in base.glob("run_*")
        if p.is_dir()
    }


def wait_new_run_dir(
    case_id: str,
    before: set[Path],
    logger_proc: subprocess.Popen,
    timeout_sec: float = 12.0,
    *,
    diagnostic_log: Path | None = None,
) -> Path:
    """Wait for benchmark_logger to create its run directory.

    rc=1 from benchmark_logger is an unhandled Python exception whose
    traceback is in its log.  This used to report only the return code, so
    the actual cause never reached the runner console or runner_status.yaml.
    """
    scenario = case_id.split("_", 1)[0]
    base = RESULTS / scenario / case_id
    deadline = time.monotonic() + timeout_sec

    while time.monotonic() < deadline:
        now = current_run_dirs(case_id)
        new = sorted(now - before)
        if new:
            return new[-1]

        if logger_proc.poll() is not None:
            raise InfraError(
                "LOGGER_START",
                f"benchmark_logger exited early "
                f"(rc={logger_proc.returncode}).\n"
                + log_tail(diagnostic_log),
            )
        time.sleep(0.2)

    raise InfraError(
        "LOGGER_START",
        f"No new result run directory appeared under {base}\n"
        + log_tail(diagnostic_log),
    )


def snapshot_configs(case_id: str, run_dir: Path) -> None:
    config_dir = run_dir / "config_snapshot"
    config_dir.mkdir(parents=True, exist_ok=True)

    scenario = case_id.split("_", 1)[0]
    local_case = CASES / scenario / f"{case_id}.yaml"
    if local_case.exists():
        shutil.copy2(local_case, config_dir / "case.yaml")

    remote_files = [
        "pil_controller.yaml",
        "pil_navigation.yaml",
        "collision_monitor.yaml",
        "gps_global.yaml",
    ]

    for filename in remote_files:
        try:
            cp = remote(
                f"cat \"$HOME/{filename}\"",
                timeout=15,
                check=True,
                show_output=False,
            )
            (config_dir / filename).write_text(
                cp.stdout,
                encoding="utf-8",
            )
        except Exception as e:
            (config_dir / f"{filename}.snapshot_error.txt").write_text(
                str(e),
                encoding="utf-8",
            )

    # Code/version metadata is useful but non-fatal.
    metadata = []
    for workspace in [
        "adaptive_escape_ws",
        "far_goal_ws",
        "dual_gps_ws",
    ]:
        try:
            cp = remote(
                f"git -C \"$HOME/{workspace}\" rev-parse HEAD 2>/dev/null "
                f"|| echo NOT_A_GIT_REPO",
                timeout=10,
                check=False,
                show_output=False,
            )
            metadata.append(
                f"{workspace}: {(cp.stdout or '').strip()}"
            )
        except Exception as e:
            metadata.append(f"{workspace}: ERROR {e}")

    (config_dir / "workspace_versions.txt").write_text(
        "\n".join(metadata) + "\n",
        encoding="utf-8",
    )


def _extract_latest_manager_session(text: str) -> str:
    """
    pi_stack_manager.sh appends multiple executions to the same log file.
    Keep only the newest managed-session block so each benchmark run gets
    a clean, case-specific Pi log snapshot.
    """
    lines = text.splitlines(keepends=True)
    start_indexes = []

    for i, line in enumerate(lines):
        if line.startswith("START "):
            # Manager format:
            # ============================================================
            # START YYYY-MM-DD HH:MM:SS
            # PROCESS: ...
            # ============================================================
            start_indexes.append(max(0, i - 1))

    if not start_indexes:
        return text

    return "".join(lines[start_indexes[-1]:])


def copy_pi_logs(run_dir: Path) -> None:
    """
    Snapshot Pi-side logs BEFORE pi_stack_manager.sh stop.

    This is intentionally done before cleanup so shutdown/SIGINT noise does
    not hide the navigation failure that ended the benchmark.
    """
    dst = run_dir / "process_logs" / "pi"
    dst.mkdir(parents=True, exist_ok=True)

    errors = []

    for name in PI_LOG_NAMES:
        try:
            cp = remote(
                f'cat "$HOME/nav_benchmark/logs/{name}.log"',
                timeout=20,
                check=False,
                show_output=False,
            )
            raw = cp.stdout or ""

            if cp.returncode != 0:
                errors.append(
                    f"{name}: remote cat rc={cp.returncode}; {raw.strip()}"
                )
                continue

            latest = _extract_latest_manager_session(raw)
            (dst / f"{name}.log").write_text(
                latest,
                encoding="utf-8",
            )

        except Exception as e:
            errors.append(f"{name}: {e!r}")

    if errors:
        (dst / "_snapshot_errors.txt").write_text(
            "\n".join(errors) + "\n",
            encoding="utf-8",
        )


def copy_case_logs(case_log_dir: Path, run_dir: Path) -> None:
    dst = run_dir / "process_logs"
    dst.mkdir(parents=True, exist_ok=True)
    for p in case_log_dir.glob("*.log"):
        try:
            shutil.copy2(p, dst / p.name)
        except OSError:
            pass


def write_runner_status(
    run_dir: Path | None,
    case_log_dir: Path,
    payload: dict,
) -> None:
    target_dir = run_dir if run_dir is not None else case_log_dir
    target_dir.mkdir(parents=True, exist_ok=True)
    with (target_dir / "runner_status.yaml").open(
        "w", encoding="utf-8"
    ) as f:
        yaml.safe_dump(
            payload,
            f,
            sort_keys=False,
            allow_unicode=True,
        )


def load_summary(run_dir: Path) -> dict:
    path = run_dir / "summary.yaml"
    if not path.exists():
        raise InfraError(
            "RESULT_COLLECTION",
            f"summary.yaml not found: {path}",
        )
    with path.open("r", encoding="utf-8") as f:
        return yaml.safe_load(f) or {}


def wait_logger_finish(
    logger_proc: subprocess.Popen,
    *,
    timeout_sec: float = 660.0,
    diagnostic_log: Path | None = None,
) -> None:
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        rc = logger_proc.poll()
        if rc is not None:
            if rc != 0:
                raise InfraError(
                    "LOGGER_RUNTIME",
                    f"benchmark_logger exited rc={rc}\n"
                    + log_tail(diagnostic_log),
                )
            return
        time.sleep(0.5)

    raise InfraError(
        "LOGGER_RUNTIME",
        f"benchmark_logger did not finish within {timeout_sec:.0f}s\n"
        + log_tail(diagnostic_log),
    )


def validate_paths() -> None:
    missing = [
        p for p in [RUN_CASE, LOGGER, SEND_GOAL]
        if not p.exists()
    ]
    if missing:
        raise SystemExit(
            "Missing PC benchmark script(s):\n"
            + "\n".join(f"  {p}" for p in missing)
        )


def preflight() -> None:
    print_stage(0, "PREFLIGHT")
    print(f"Benchmark suite: {SUITE_VERSION}")
    validate_paths()

    if not PI_HOST:
        raise SystemExit(
            "Raspberry Pi address is not set.\n"
            "Enter the current Pi IP in the QA GUI, or export "
            "NAV_BENCH_PI_HOST before running from a terminal."
        )

    try:
        cp = remote("true", timeout=8, show_output=False)
    except (RuntimeError, subprocess.TimeoutExpired) as e:
        raise SystemExit(
            "Passwordless SSH to Raspberry Pi is required for automatic "
            f"benchmarking.\nTarget: {PI_TARGET}\n\n"
            f"SSH error:\n{e}\n"
            "Check the Pi address/network and SSH key authentication, then "
            "run the runner again."
        ) from e

    # Verify the manager exists and hil_router is intentionally external.
    cp = remote(
        f"test -x {PI_MANAGER} && echo MANAGER_OK",
        timeout=10,
    )
    if "MANAGER_OK" not in (cp.stdout or ""):
        raise SystemExit("Pi stack manager was not found/executable.")

    assert_pi_zenoh_port()
    assert_local_router_port_free()

    print(f"[OK] PC scripts")
    print(f"[OK] SSH -> {PI_TARGET}")
    print(f"[OK] Pi stack manager")
    print("[INFO] hil_router must already be running on the Pi.")
    print("[INFO] A linked PC-local Zenoh router will be managed automatically.")


def bridge_command():
    return BRIDGE_ARGS


def logger_command(case_id: str):
    return [
        "python3", "-u", str(LOGGER), case_id,
        "--ros-args",
        "-p", "use_sim_time:=true",
        *TF_ARGS,
    ]


def sender_command(case_id: str, map_start: Pose2D):
    return [
        "python3", "-u", str(SEND_GOAL), case_id,
        "--map-start-x", f"{map_start.x:.9f}",
        "--map-start-y", f"{map_start.y:.9f}",
        "--map-start-yaw", f"{map_start.yaw:.9f}",
        "--ros-args",
        "-p", "use_sim_time:=true",
        *TF_ARGS,
    ]


def run_case_command(case_id: str):
    return ["python3", "-u", str(RUN_CASE), case_id]