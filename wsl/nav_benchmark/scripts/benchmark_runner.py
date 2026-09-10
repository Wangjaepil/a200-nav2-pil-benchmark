#!/usr/bin/env python3
"""
PC/WSL benchmark orchestrator v2.0 for the Gazebo <-> Raspberry Pi Nav2 benchmark.

What this runner manages automatically:
  PC:  one local Zenoh router linked to the Pi router
For EACH case:
  PC:  run_case.py -> Gazebo/Clearpath -> ros_gz_bridge
  Pi:  localization -> Nav2 processes -> lifecycle configure/activate
       -> Adaptive Escape overlays -> Far Goal Manager
  PC:  benchmark_logger.py -> validated send_case_goal.py
  End: result collection -> config/log snapshot -> clean shutdown

What it intentionally does NOT manage:
  Pi hil_router (must already be running)

Examples:
  python3 ~/nav_benchmark/scripts/benchmark_runner.py S1_01
  python3 ~/nav_benchmark/scripts/benchmark_runner.py --scenario S1
  python3 ~/nav_benchmark/scripts/benchmark_runner.py --all
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import yaml

from benchmark_common import suite_metadata
from benchmark_runner_support import (
    CASES,
    COSTMAP_TOPICS,
    PC_ZENOH_ENDPOINT,
    RUNNER_LOG_ROOT,
    SENSOR_TOPICS,
    ZENOH_ENDPOINT,
    InfraError,
    assert_map_pose_unchanged,
    assert_platform_odom_stationary,
    assert_pre_goal_stationary,
    bridge_command,
    cleanup_pc_benchmark_orphans,
    copy_case_logs,
    copy_pi_logs,
    current_run_dirs,
    force_platform_zero_burst,
    load_summary,
    logger_command,
    manager,
    preflight,
    print_stage,
    run_case_command,
    sender_command,
    slug_ts,
    snapshot_configs,
    start_pc_process,
    start_pc_zenoh_router,
    start_pre_goal_zero_hold,
    stop_pc_process,
    stop_pre_goal_zero_hold,
    ts,
    verify_nav2_active,
    wait_file_marker,
    wait_log_marker,
    wait_logger_finish,
    wait_map_to_base,
    wait_new_run_dir,
    wait_pc_zenoh_router,
    wait_stable_map_pose,
    wait_topic_once,
    write_runner_status,
)


def run_one_case(case_id: str, batch_dir: Path) -> dict:
    case_id = case_id.upper()
    scenario = case_id.split("_", 1)[0]
    case_log_dir = batch_dir / case_id
    case_log_dir.mkdir(parents=True, exist_ok=True)

    run_case_proc = None
    bridge_proc = None
    logger_proc = None
    sender_proc = None
    pre_goal_zero_hold_proc = None
    run_dir = None
    map_start_pose = None
    pi_logs_snapshotted = False
    stage = "INIT"
    started_wall = time.monotonic()
    started_at = ts()

    try:
        print()
        print("#" * 72)
        print(f"# CASE {case_id} START")
        print("#" * 72)

        stage = "CLEANUP_PREVIOUS"
        print_stage(1, f"{case_id}: CLEAN PREVIOUS CASE")
        # First clear any retained actuator command while an old Gazebo may
        # still exist, then remove both Pi-side and PC-side leftovers.
        try:
            force_platform_zero_burst("before previous-case cleanup", 0.6)
        except Exception as exc:
            print(f"[WARN] Initial zero burst skipped: {exc}")
        manager("stop", timeout=60)
        cleanup_pc_benchmark_orphans()

        stage = "SIMULATION_START"
        print_stage(2, f"{case_id}: START GAZEBO + ROBOT")
        run_case_log = case_log_dir / "run_case.log"
        run_case_proc = start_pc_process(
            "run_case",
            run_case_command(case_id),
            run_case_log,
        )
        # Do NOT wait for a text marker from run_case.py.
        # run_case.py is a long-lived launcher and its log text is not a
        # reliable readiness API.  The actual readiness gate is the ROS
        # data check after the bridge starts.
        time.sleep(2.0)
        if run_case_proc.poll() is not None:
            tail = ""
            if run_case_log.exists():
                tail = run_case_log.read_text(
                    encoding="utf-8", errors="replace"
                )[-4000:]
            raise InfraError(
                stage,
                f"run_case.py exited early (rc={run_case_proc.returncode}).\n"
                f"{tail}",
            )
        print("[STARTED] Gazebo launcher is alive; readiness will be checked by topics.")

        stage = "BRIDGE_START"
        print_stage(3, f"{case_id}: START SENSOR BRIDGE")
        bridge_proc = start_pc_process(
            "ros_gz_bridge",
            bridge_command(),
            case_log_dir / "bridge.log",
        )

        # Give Gazebo transport + Zenoh discovery a short warm-up before
        # opening per-topic readiness probes.
        time.sleep(2.0)
        if bridge_proc.poll() is not None:
            bridge_log = case_log_dir / "bridge.log"
            tail = ""
            if bridge_log.exists():
                tail = bridge_log.read_text(
                    encoding="utf-8", errors="replace"
                )[-4000:]
            raise InfraError(
                stage,
                f"ros_gz_bridge exited early (rc={bridge_proc.returncode}).\n"
                f"{tail}",
            )

        for topic in SENSOR_TOPICS:
            wait_topic_once(topic, timeout_sec=45)

        # Hold zero from the first moment the simulated actuator topic exists.
        # It stays active through localization, lifecycle activation, costmap
        # readiness, logger startup, and artifact snapshotting.
        pre_goal_zero_hold_proc = start_pre_goal_zero_hold(
            case_log_dir / "pre_goal_zero_hold.log"
        )

        # A fresh Gazebo/Clearpath instance must start from zero physical
        # velocity regardless of how a previous run terminated.
        force_platform_zero_burst("fresh Gazebo/Clearpath instance", 0.8)
        assert_platform_odom_stationary("after fresh simulation start")

        stage = "LOCALIZATION_START"
        print_stage(4, f"{case_id}: START PI LOCALIZATION")
        manager("start_localization", timeout=45)

        stage = "LOCALIZATION_READINESS"
        print_stage(5, f"{case_id}: WAIT FOR MAP -> BASE_LINK")
        first_map_pose = wait_map_to_base(timeout_sec=50)
        map_start_pose = wait_stable_map_pose(first_map_pose)

        stage = "NAV2_START"
        print_stage(6, f"{case_id}: START NAV2 PROCESSES")
        manager("start_nav2", timeout=80)

        assert_platform_odom_stationary("before lifecycle configure")

        stage = "NAV2_CONFIGURE"
        print_stage(7, f"{case_id}: CONFIGURE LIFECYCLE")
        manager("configure", timeout=90)
        # Raw Nav2 output must still be zero/no-message even though the final
        # actuator is protected by the setup interlock.
        assert_pre_goal_stationary("after lifecycle configure")
        assert_platform_odom_stationary("after lifecycle configure")

        stage = "NAV2_ACTIVATE"
        print_stage(8, f"{case_id}: ACTIVATE LIFECYCLE")
        manager("activate", timeout=90)
        assert_pre_goal_stationary("after Nav2 activation")
        assert_platform_odom_stationary("after Nav2 activation")

        stage = "COSTMAP_READINESS"
        print("[READY CHECK] Waiting for fresh local/global costmaps ...")
        for topic in COSTMAP_TOPICS:
            wait_topic_once(
                topic,
                timeout_sec=45,
                stage="COSTMAP_READINESS",
            )

        stage = "FAR_GOAL_START"
        print_stage(9, f"{case_id}: START FAR GOAL MANAGER")
        manager("start_far_goal", timeout=30)
        verify_nav2_active()

        # Starting Far Goal Manager itself must also be inert until it receives
        # this case's /far_goal_pose. This catches a stale/duplicate publisher.
        assert_pre_goal_stationary("after Far Goal Manager start")

        stage = "LOGGER_START"
        print_stage(10, f"{case_id}: START LOGGER")
        before = current_run_dirs(case_id)
        logger_log = case_log_dir / "benchmark_logger.log"
        logger_proc = start_pc_process(
            "benchmark_logger",
            logger_command(case_id),
            logger_log,
        )
        run_dir = wait_new_run_dir(
            case_id,
            before,
            logger_proc,
            timeout_sec=15,
            diagnostic_log=logger_log,
        )
        wait_log_marker(
            logger_proc,
            logger_log,
            "Waiting for /far_goal_pose ...",
            timeout=15,
            stage="LOGGER_READY",
        )
        print(f"[READY] Logger subscribed and waiting: {run_dir}")

        # Snapshot while the setup hold is still active. Slow SSH or Wi-Fi must
        # never create an unguarded delay immediately before goal publication.
        snapshot_configs(case_id, run_dir)
        assert map_start_pose is not None
        assert_map_pose_unchanged(
            map_start_pose,
            "after logger/config snapshot",
        )

        # Release the setup-only actuator interlock.  If any stale/duplicate
        # publisher is still trying to move the robot, it must reveal itself
        # now and the case is rejected BEFORE a final goal is sent.
        stop_pre_goal_zero_hold(pre_goal_zero_hold_proc)
        pre_goal_zero_hold_proc = None
        time.sleep(0.5)
        assert_pre_goal_stationary("immediately before goal send")
        assert_platform_odom_stationary("immediately before goal send")
        assert_map_pose_unchanged(map_start_pose, "immediately before goal send")

        stage = "GOAL_SEND"
        print_stage(11, f"{case_id}: CONVERT + VALIDATE + SEND GOAL")
        sender_log = case_log_dir / "send_case_goal.log"
        sender_proc = start_pc_process(
            "send_case_goal",
            sender_command(case_id, map_start_pose),
            sender_log,
        )
        wait_log_marker(
            sender_proc,
            sender_log,
            "Goal published to /far_goal_pose",
            timeout=30,
            stage=stage,
        )
        print("[READY] Goal sender published /far_goal_pose")

        # Confirm that the benchmark logger received this case's final goal
        # before stopping the one-shot sender.
        wait_log_marker(
            logger_proc,
            logger_log,
            f"START {case_id} / {run_dir.name}",
            timeout=12,
            stage="GOAL_DELIVERY",
        )
        print("[READY] Logger received /far_goal_pose")

        # The current sender may intentionally stay spinning after publish.
        # It no longer needs to remain alive once delivery is confirmed.
        stop_pc_process(
            "send_case_goal",
            sender_proc,
            grace=2.0,
        )
        sender_proc = None

        stage = "NAVIGATION_MONITOR"
        print_stage(12, f"{case_id}: WAIT FOR NAVIGATION RESULT")
        wait_file_marker(
            run_dir / "events.csv",
            "NAV_GOAL_NEW",
            proc=logger_proc,
            timeout_sec=20,
            stage="NAVIGATION_START",
            diagnostic_log=logger_log,
        )
        print("[READY] NavigateToPose goal observed")
        wait_logger_finish(
            logger_proc,
            timeout_sec=660,
            diagnostic_log=logger_log,
        )
        logger_proc = None

        stage = "RESULT_COLLECTION"
        print_stage(13, f"{case_id}: COLLECT RESULT")
        summary = load_summary(run_dir)

        # Capture Pi logs while the processes still exist and before cleanup
        # introduces SIGINT/shutdown messages.
        copy_pi_logs(run_dir)
        pi_logs_snapshotted = True

        benchmark_result = summary.get(
            "benchmark_result",
            summary.get("result", "UNKNOWN"),
        )
        nav2_result = summary.get(
            "nav2_result",
            summary.get("result", "UNKNOWN"),
        )
        runner_result = (
            "INFRA_ERROR"
            if benchmark_result == "INFRA_ERROR"
            else "COMPLETED"
        )

        payload = {
            **suite_metadata(),
            "case_id": case_id,
            "runner_result": runner_result,
            "benchmark_result": benchmark_result,
            "nav2_result": nav2_result,
            "started_at": started_at,
            "runner_wall_sec": round(
                time.monotonic() - started_wall, 3
            ),
            "result_dir": str(run_dir),
        }
        write_runner_status(run_dir, case_log_dir, payload)

        print()
        print("-" * 72)
        print(f"CASE             : {case_id}")
        print(f"NAV2 RESULT      : {nav2_result}")
        print(f"BENCHMARK RESULT : {benchmark_result}")
        print(
            f"SIM TIME         : "
            f"{summary.get('duration_sim_sec')} s"
        )
        print(
            f"TRAVEL DISTANCE  : "
            f"{summary.get('travel_distance_odom_m')} m"
        )
        print(
            f"FINAL XY ERROR   : "
            f"{summary.get('final_xy_error_m')} m"
        )
        print(
            f"FINAL YAW ERROR  : "
            f"{summary.get('final_yaw_error_deg')} deg"
        )
        print(
            f"ADAPTIVE ESCAPE  : "
            f"{summary.get('adaptive_escape_count')}"
        )
        print(
            f"COSTMAP CLEARS   : "
            f"{summary.get('costmap_clear_count')}"
        )
        print(f"RESULT DIRECTORY : {run_dir}")
        print("-" * 72)

        return {
            "case_id": case_id,
            "runner_result": runner_result,
            "benchmark_result": benchmark_result,
            "nav2_result": nav2_result,
            "result_dir": str(run_dir),
        }

    except KeyboardInterrupt:
        raise

    except InfraError as e:
        print()
        print(f"[INFRA_ERROR] stage={e.stage}")
        print(e.message)
        write_runner_status(
            run_dir,
            case_log_dir,
            {
                **suite_metadata(),
                "case_id": case_id,
                "runner_result": "INFRA_ERROR",
                "failure_stage": e.stage,
                "failure_reason": e.message,
                "runner_wall_sec": round(
                    time.monotonic() - started_wall, 3
                ),
            },
        )
        return {
            "case_id": case_id,
            "runner_result": "INFRA_ERROR",
            "benchmark_result": "INFRA_ERROR",
            "nav2_result": None,
            "failure_stage": e.stage,
            "failure_reason": e.message,
            "result_dir": str(run_dir) if run_dir else None,
        }

    except Exception as e:
        print()
        print(f"[INFRA_ERROR] stage={stage}")
        print(repr(e))
        write_runner_status(
            run_dir,
            case_log_dir,
            {
                **suite_metadata(),
                "case_id": case_id,
                "runner_result": "INFRA_ERROR",
                "failure_stage": stage,
                "failure_reason": repr(e),
                "runner_wall_sec": round(
                    time.monotonic() - started_wall, 3
                ),
            },
        )
        return {
            "case_id": case_id,
            "runner_result": "INFRA_ERROR",
            "benchmark_result": "INFRA_ERROR",
            "nav2_result": None,
            "failure_stage": stage,
            "failure_reason": repr(e),
            "result_dir": str(run_dir) if run_dir else None,
        }

    finally:
        print_stage(14, f"{case_id}: CLEANUP")

        stop_pre_goal_zero_hold(pre_goal_zero_hold_proc)
        pre_goal_zero_hold_proc = None
        stop_pc_process(
            "send_case_goal",
            sender_proc,
            grace=2.0,
        )
        stop_pc_process(
            "benchmark_logger",
            logger_proc,
            grace=5.0,
        )

        # If a run directory exists, preserve the Pi-side evidence before
        # stopping the stack.  This also covers INFRA_ERROR paths occurring
        # after the logger has created its result directory.
        if run_dir is not None and not pi_logs_snapshotted:
            try:
                copy_pi_logs(run_dir)
                pi_logs_snapshotted = True
            except Exception as e:
                print(f"[WARN] Pi log snapshot failed: {e}")

        try:
            force_platform_zero_burst("case cleanup", 0.6)
        except Exception as e:
            print(f"[WARN] Cleanup zero burst failed: {e}")

        try:
            manager("stop", timeout=70)
        except Exception as e:
            print(f"[WARN] Pi cleanup failed: {e}")

        stop_pc_process(
            "ros_gz_bridge",
            bridge_proc,
            grace=5.0,
        )
        stop_pc_process(
            "run_case",
            run_case_proc,
            grace=12.0,
        )

        if run_dir is not None:
            copy_case_logs(case_log_dir, run_dir)

        # Give ROS graph / Gazebo transport a short deterministic settling
        # period before a following case starts.
        time.sleep(2.0)


def discover_cases(args) -> list[str]:
    if args.cases:
        case_ids = []
        for raw_case_id in args.cases:
            case_id = raw_case_id.upper()
            scenario = case_id.split("_", 1)[0]
            path = CASES / scenario / f"{case_id}.yaml"
            if not path.exists():
                raise SystemExit(f"Case not found: {path}")
            case_ids.append(case_id)
        return case_ids

    if args.case_id:
        case_id = args.case_id.upper()
        scenario = case_id.split("_", 1)[0]
        path = CASES / scenario / f"{case_id}.yaml"
        if not path.exists():
            raise SystemExit(f"Case not found: {path}")
        return [case_id]

    if args.scenario:
        scenario = args.scenario.upper()
        folder = CASES / scenario
        if not folder.exists():
            raise SystemExit(f"Scenario folder not found: {folder}")
        paths = sorted(folder.glob(f"{scenario}_*.yaml"))
        if not paths:
            raise SystemExit(f"No cases found in {folder}")
        return [p.stem.upper() for p in paths]

    if args.all:
        paths = []
        for scenario_dir in sorted(CASES.glob("S*")):
            if not scenario_dir.is_dir():
                continue
            paths.extend(sorted(scenario_dir.glob("S*_*.yaml")))
        if not paths:
            raise SystemExit(f"No benchmark cases found under {CASES}")
        return [p.stem.upper() for p in paths]

    raise SystemExit("Specify CASE_ID, --scenario S1, or --all")


def save_batch_summary(batch_dir: Path, records: list[dict]) -> None:
    counts = {}
    for r in records:
        key = r.get("benchmark_result", "UNKNOWN")
        counts[key] = counts.get(key, 0) + 1

    completed = [
        r for r in records
        if r.get("runner_result") == "COMPLETED"
    ]
    strict_pass = sum(
        r.get("benchmark_result") == "PASS"
        for r in completed
    )
    practical_pass = sum(
        r.get("benchmark_result") in {"PASS", "NEAR_SUCCESS"}
        for r in completed
    )

    denom = len(completed)
    summary = {
        **suite_metadata(),
        "generated_at": ts(),
        "total_cases_requested": len(records),
        "completed_cases": denom,
        "infra_error_cases": sum(
            r.get("runner_result") == "INFRA_ERROR"
            for r in records
        ),
        "result_counts": counts,
        "strict_success_rate_percent": (
            round(100.0 * strict_pass / denom, 2)
            if denom else None
        ),
        "practical_success_rate_percent": (
            round(100.0 * practical_pass / denom, 2)
            if denom else None
        ),
        "cases": records,
    }

    with (batch_dir / "batch_summary.yaml").open(
        "w", encoding="utf-8"
    ) as f:
        yaml.safe_dump(
            summary,
            f,
            sort_keys=False,
            allow_unicode=True,
        )

    print()
    print("=" * 72)
    print("BENCHMARK BATCH COMPLETE")
    print("=" * 72)
    for r in records:
        print(
            f"{r['case_id']:<8} "
            f"{r.get('benchmark_result', 'UNKNOWN')}"
        )
    print("-" * 72)
    for key in sorted(counts):
        print(f"{key:<16}: {counts[key]}")
    if denom:
        print(
            f"Strict success     : "
            f"{summary['strict_success_rate_percent']} %"
        )
        print(
            f"Practical success  : "
            f"{summary['practical_success_rate_percent']} %"
        )
    print(f"Batch summary      : {batch_dir / 'batch_summary.yaml'}")


def _batch_exit_code(records: list[dict], *, interrupted: bool) -> int:
    """Process status: benchmark FAIL is valid data; infrastructure failure is not."""
    if interrupted:
        return 130
    if any(r.get("runner_result") == "INFRA_ERROR" for r in records):
        return 2
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Automated PC/Pi Nav2 benchmark runner."
    )
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument(
        "case_id",
        nargs="?",
        help="Single case, e.g. S1_01",
    )
    group.add_argument(
        "--cases",
        nargs="+",
        help="Run selected cases, e.g. --cases S1_01 S1_03 S2_02",
    )
    group.add_argument(
        "--scenario",
        help="Run all cases in a scenario, e.g. S1",
    )
    group.add_argument(
        "--all",
        action="store_true",
        help="Run every case under ~/nav_benchmark/cases",
    )
    parser.add_argument(
        "--stop-on-infra-error",
        action="store_true",
        help="Stop the batch if infrastructure setup fails.",
    )
    args = parser.parse_args()

    cases = discover_cases(args)
    preflight()

    batch_dir = RUNNER_LOG_ROOT / (
        f"{slug_ts()}_{cases[0]}_to_{cases[-1]}"
        if len(cases) > 1
        else f"{slug_ts()}_{cases[0]}"
    )
    batch_dir.mkdir(parents=True, exist_ok=True)

    print()
    print(f"Cases       : {len(cases)}")
    print(f"First       : {cases[0]}")
    print(f"Last        : {cases[-1]}")
    print(f"Runner logs : {batch_dir}")
    print()
    print("IMPORTANT: Pi hil_router must remain running.")
    print(
        "PC ROS route: "
        f"nodes -> {PC_ZENOH_ENDPOINT} -> {ZENOH_ENDPOINT}"
    )
    print("RViz is not required for automated benchmark mode.")

    records = []
    pc_router_proc = None
    interrupted = False

    try:
        print()
        print("=" * 72)
        print("[00B] START PC-LOCAL ZENOH ROUTER")
        print("=" * 72)
        pc_router_log = batch_dir / "pc_zenoh_router.log"
        pc_router_proc = start_pc_zenoh_router(pc_router_log)
        wait_pc_zenoh_router(pc_router_proc, pc_router_log)

        for index, case_id in enumerate(cases, start=1):
            print()
            print(
                f">>>>> CASE {index}/{len(cases)} : {case_id} <<<<<"
            )
            result = run_one_case(case_id, batch_dir)
            records.append(result)

            if (
                args.stop_on_infra_error
                and result.get("runner_result") == "INFRA_ERROR"
            ):
                print("[STOP] --stop-on-infra-error requested.")
                break

    except KeyboardInterrupt:
        interrupted = True
        print("\n[INTERRUPTED] User requested stop.")

    finally:
        # Last-resort Pi cleanup; hil_router is not touched.
        try:
            manager("stop", timeout=70)
        except Exception:
            pass

        stop_pc_process(
            "local_zenoh_router",
            pc_router_proc,
            grace=5.0,
        )

        if records:
            save_batch_summary(batch_dir, records)

    return _batch_exit_code(records, interrupted=interrupted)


if __name__ == "__main__":
    sys.exit(main())