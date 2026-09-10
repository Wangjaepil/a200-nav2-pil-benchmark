#!/usr/bin/env bash
set -Eeuo pipefail

# ============================================================
# Pi Nav Benchmark Stack Manager v1.6
# - Keeps hil_router OUTSIDE this manager.
# - Manages localization, Nav2 lifecycle servers, Adaptive Escape,
#   Collision Monitor, and Far Goal Manager.
# - Each managed process runs in its own process group so stop() can
#   reliably terminate the entire ros2 run process tree.
#
# Command surface is unchanged from v1.5 (the PC runner depends on it):
#   start_localization | start_nav2 | configure | activate
#   start_far_goal | status | verify | stop
#
# Fixes vs v1.5:
#   * SIGINT was ignored by EVERY managed node. bash gives asynchronous
#     ("&") commands SIG_IGN for SIGINT when job control is off, and that
#     survives exec, so `kill -INT -- -PGID` was a no-op: the graceful
#     window was always burned in full and every node was actually killed
#     by SIGTERM, never getting a clean rclpy/Zenoh shutdown. Verified
#     reproducible; nodes now stop on SIGINT in ~0.1s.
#   * set -E had no ERR trap; a failure over SSH gave no context.
#   * stop_process only watched the group LEADER, so a surviving child
#     was reported as "[STOPPED]". It now waits for the whole process
#     group to disappear.
#   * stop_process would signal "-PID" even when that PID was no longer
#     its own process-group leader (PID reuse) -> could signal an
#     unrelated group. Now guarded.
#   * A single transient rmw_zenoh CLI miss made activate() hard-fail
#     the whole case. State reads now retry and understand transition
#     states (configuring/activating/...).
#   * Every action now fits inside the PC runner's SSH timeouts
#     (stop<=70s, start_nav2<=80s, configure/activate<=90s) via global
#     phase budgets, while still allowing graceful SIGINT shutdown.
#   * sleep 0.4 could not detect a node that dies on a bad params file;
#     start_process now confirms the real node process appeared and
#     prints a log tail on failure.
#   * use_sim_time was only explicit on dual_gps_heading/far_goal. It is
#     now explicit on every managed node (silent wall-clock TF is fatal
#     in a sim benchmark).  <-- verify against your YAMLs.
#   * verify only checked Nav2; a dead EKF still passed. Localization is
#     now checked too.
#   * clean_stale_pids ran before status(), so STALE_PID was unreachable.
#   * Unbounded log growth across a 120-case batch; logs now rotate.
# ============================================================

BASE_DIR="${HOME}/nav_benchmark"
PID_DIR="${BASE_DIR}/pids"
LOG_DIR="${BASE_DIR}/logs"

mkdir -p "${PID_DIR}" "${LOG_DIR}"

# Common ROS environment for manager-side lifecycle/status commands.
set +u
source /opt/ros/jazzy/setup.bash
set -u
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ROS_DOMAIN_ID=10

ACTION="${1:-}"

# --- SIGINT disposition ------------------------------------------------
# bash sets SIGINT/SIGQUIT to SIG_IGN for asynchronous ("&") commands when
# job control is off, and SIG_IGN is inherited through exec. So every node
# launched as `setsid bash -lc "... exec ros2 run ..." &` inherited an
# ignored SIGINT: `kill -INT -- -PGID` did nothing, the graceful window was
# always wasted, and every Nav2 node was really killed by SIGTERM - i.e. it
# never got the clean rclpy shutdown that releases its Zenoh/DDS entities.
# This wrapper restores the default disposition immediately before exec.
SIGNAL_RESET_PY='import signal, os, sys
for _name in ("SIGINT", "SIGQUIT"):
    _sig = getattr(signal, _name, None)
    if _sig is not None:
        try:
            signal.signal(_sig, signal.SIG_DFL)
        except (OSError, ValueError):
            pass
os.execvp(sys.argv[1], sys.argv[1:])'

if command -v python3 >/dev/null 2>&1; then
  SIGNAL_RESET_LAUNCHER=(python3 -c "${SIGNAL_RESET_PY}")
else
  SIGNAL_RESET_LAUNCHER=()
  echo "[WARN] python3 not found: managed nodes will ignore SIGINT and will" >&2
  echo "       only stop via SIGTERM (no clean Nav2 shutdown)." >&2
fi

TF_REMAPS='-r /tf:=/a200_0000/tf -r /tf_static:=/a200_0000/tf_static'

# The whole benchmark runs on /clock. A node that silently falls back to
# wall time produces TF timestamps Nav2 cannot use, so this is forced on
# the command line where it overrides the params file.
SIM_TIME_ARG='-p use_sim_time:=true'

# --- time budgets -------------------------------------------------------
# These exist so the script always fails/returns BEFORE the PC runner's
# SSH timeout, which would otherwise hide the log output.
LOG_MAX_BYTES="${NAV_BENCH_LOG_MAX_BYTES:-33554432}"   # 32 MiB
START_CHILD_WAIT_SEC=6          # wait for the real node process to appear
START_SETTLE_SEC=1.5            # ... and confirm it is still alive after this
START_NAV2_BUDGET_SEC=65        # PC allows 80s
LIFECYCLE_PHASE_BUDGET_SEC=75   # PC allows 90s for configure/activate
# Two nested budgets, both shared across all 9 processes, so `stop` cannot
# be pushed past the PC's 70s SSH timeout no matter how many nodes hang:
#   - grace budget: how long SIGINT (clean Nav2 shutdown) may be waited on
#   - hard budget:  when it expires, escalate straight to SIGKILL
# Worst case ~= HARD + 9*SIGKILL + ~7s of zero-hold/sweep ~= 46s.
STOP_GRACE_BUDGET_SEC=20        # PC allows 70s for stop
STOP_HARD_BUDGET_SEC=34
STOP_SIGINT_MAX_SEC=6
STOP_SIGTERM_MAX_SEC=2
STOP_SIGKILL_SEC=1

# Effectively "no budget" unless a phase/stop sets one.
PHASE_DEADLINE=$((SECONDS + 86400))
STOP_DEADLINE=$((SECONDS + 86400))
STOP_HARD_DEADLINE=$((SECONDS + 86400))

LIFECYCLE_NODES=(
  controller_server
  planner_server
  behavior_server
  collision_monitor
  bt_navigator
)

LOCALIZATION_PROCESSES=(
  dual_gps_heading
  global_ekf
  navsat_transform
)

MANAGED_PROCESSES=(
  dual_gps_heading
  global_ekf
  navsat_transform
  controller_server
  planner_server
  behavior_server
  collision_monitor
  bt_navigator
  far_goal
)

# Single source of truth: managed name -> pgrep -f pattern of the REAL
# node executable (not the `ros2 run` wrapper). Used by the health check,
# the start confirmation, and the orphan sweep.
declare -A PROCESS_MATCH=(
  [dual_gps_heading]='[/]dual_gps_heading/heading_node'
  [global_ekf]='[/]robot_localization/ekf_node'
  [navsat_transform]='[/]robot_localization/navsat_transform_node'
  [controller_server]='[/]nav2_controller/controller_server'
  [planner_server]='[/]nav2_planner/planner_server'
  [behavior_server]='[/]nav2_behaviors/behavior_server'
  [collision_monitor]='[/]nav2_collision_monitor/collision_monitor'
  [bt_navigator]='[/]nav2_bt_navigator/bt_navigator'
  [far_goal]='[/]far_goal_manager/far_goal_manager'
)

# Reverse dependency order for clean shutdown.
STOP_ORDER=(
  far_goal
  bt_navigator
  collision_monitor
  behavior_server
  planner_server
  controller_server
  navsat_transform
  global_ekf
  dual_gps_heading
)

# Primary lifecycle states plus the transition states. Reporting a
# transition as UNAVAILABLE used to make configure/activate misbehave.
LIFECYCLE_STATE_RE='^(unconfigured|inactive|active|finalized|configuring|cleaningup|activating|deactivating|shuttingdown|errorprocessing|unknown)$'

on_error() {
  local rc=$?
  local line="${1:-?}"
  local cmd="${2:-?}"
  echo "[FATAL] action='${ACTION}' line=${line} rc=${rc}" >&2
  echo "[FATAL] command: ${cmd}" >&2
  return 0
}
trap 'on_error "${LINENO}" "${BASH_COMMAND}"' ERR

timestamp() {
  date '+%Y-%m-%d %H:%M:%S'
}

pid_file() {
  printf '%s/%s.pid\n' "${PID_DIR}" "$1"
}

log_file() {
  printf '%s/%s.log\n' "${LOG_DIR}" "$1"
}

# Remaining seconds in the current phase, clamped to [1, max].
capped_remaining() {
  local max="$1"
  local remaining=$((PHASE_DEADLINE - SECONDS))
  if (( remaining < 1 )); then
    remaining=1
  fi
  if (( remaining > max )); then
    remaining="${max}"
  fi
  printf '%s\n' "${remaining}"
}

rotate_log() {
  local lf="$1"
  local size

  [[ -f "${lf}" ]] || return 0
  size="$(stat -c '%s' "${lf}" 2>/dev/null || echo 0)"
  [[ "${size}" =~ ^[0-9]+$ ]] || return 0

  if (( size > LOG_MAX_BYTES )); then
    mv -f "${lf}" "${lf}.1" 2>/dev/null || true
  fi
  return 0
}

# Logs are append-only across runs (the PC parses the newest session
# block), so a plain `tail` shows mostly old banners. Print the tail of the
# CURRENT session only.
log_tail_current_session() {
  local lf="$1"
  local lines="${2:-40}"
  local from

  [[ -f "${lf}" ]] || return 0
  from="$(grep -n '^START [0-9]' "${lf}" 2>/dev/null | tail -1 | cut -d: -f1 || true)"

  if [[ -n "${from}" ]]; then
    tail -n "+${from}" "${lf}" | tail -n "${lines}" | sed 's/^/        | /'
  else
    tail -n "${lines}" "${lf}" | sed 's/^/        | /'
  fi
  return 0
}

process_group_of() {
  local pgid
  pgid="$(ps -o pgid= -p "$1" 2>/dev/null | tr -d ' \n' || true)"
  printf '%s\n' "${pgid}"
}

# `kill -0` succeeds on a zombie, so a killed-but-unreaped process used to
# look alive: stop() would burn its whole budget and then report failure,
# and status() would show it as RUNNING. A zombie cannot execute or publish
# ROS traffic. (The PC runner already applies this rule in
# cleanup_pc_benchmark_orphans; this is the Pi-side equivalent.)
pid_alive() {
  local state
  state="$(ps -o stat= -p "$1" 2>/dev/null | tr -d ' ' || true)"
  [[ -n "${state}" && "${state}" != Z* ]]
}

group_pids() {
  local pid
  for pid in $(pgrep -g "$1" 2>/dev/null || true); do
    if pid_alive "${pid}"; then
      printf '%s\n' "${pid}"
    fi
  done
}

group_gone() {
  [[ -z "$(group_pids "$1")" ]]
}

# The real node process for `name`, restricted to process group `pgid`.
node_pids_in_group() {
  local name="$1"
  local pgid="$2"
  local pattern="${PROCESS_MATCH[${name}]:-}"
  local pid

  [[ -n "${pattern}" ]] || return 0
  for pid in $(pgrep -g "${pgid}" -f -- "${pattern}" 2>/dev/null || true); do
    if pid_alive "${pid}"; then
      printf '%s\n' "${pid}"
    fi
  done
}

leader_pid() {
  local pf pid
  pf="$(pid_file "$1")"

  [[ -f "${pf}" ]] || return 1
  pid="$(cat "${pf}" 2>/dev/null || true)"
  [[ "${pid}" =~ ^[0-9]+$ ]] || return 1

  printf '%s\n' "${pid}"
}

is_running() {
  local pid
  pid="$(leader_pid "$1" 2>/dev/null || true)"

  [[ -n "${pid}" ]] || return 1
  pid_alive "${pid}"
}

# Stronger than is_running: the group leader is alive AND the actual ROS
# node executable is present inside its process group. `ros2 run` staying
# alive after its node died used to look healthy.
process_healthy() {
  local name="$1"
  local pid

  pid="$(leader_pid "${name}" 2>/dev/null || true)"
  [[ -n "${pid}" ]] || return 1
  pid_alive "${pid}" || return 1
  [[ -n "$(node_pids_in_group "${name}" "${pid}")" ]]
}

start_process() {
  local name="$1"
  local command="$2"

  local pf lf pid deadline child_seen
  pf="$(pid_file "${name}")"
  lf="$(log_file "${name}")"

  if is_running "${name}"; then
    echo "[SKIP] ${name} already running (PID $(cat "${pf}"))"
    return 0
  fi

  # Remove stale PID file.
  rm -f "${pf}"
  rotate_log "${lf}"

  {
    echo
    echo "============================================================"
    echo "START $(timestamp)"
    echo "PROCESS: ${name}"
    echo "============================================================"
  } >> "${lf}"

  # setsid => new session/process group.
  # We store the process-group leader PID and later signal the whole group.
  # SIGNAL_RESET_LAUNCHER execs in place, so the stored PID is still the
  # process-group leader.
  setsid "${SIGNAL_RESET_LAUNCHER[@]}" bash -lc "
    source /opt/ros/jazzy/setup.bash
    export RMW_IMPLEMENTATION=rmw_zenoh_cpp
    export ROS_DOMAIN_ID=10
    ${command}
  " >> "${lf}" 2>&1 &

  pid=$!
  echo "${pid}" > "${pf}"

  # v1.5 slept 0.4s and only checked the leader. A node that dies on a bad
  # params file takes 1-3s and leaves `ros2 run` alive briefly, so that
  # check almost never fired and the failure surfaced much later as a
  # vague map->base_link timeout on the PC.
  deadline=$((SECONDS + START_CHILD_WAIT_SEC))
  child_seen=0

  while (( SECONDS < deadline )); do
    if ! pid_alive "${pid}"; then
      echo "[ERROR] ${name} exited immediately."
      echo "        Log: ${lf}"
      log_tail_current_session "${lf}" 40
      rm -f "${pf}"
      return 1
    fi
    if [[ -n "$(node_pids_in_group "${name}" "${pid}")" ]]; then
      child_seen=1
      break
    fi
    sleep 0.25
  done

  if (( child_seen == 0 )); then
    # Leader is alive but the node executable has not appeared yet. Not
    # fatal on its own (slow overlay sourcing), but it is the single most
    # useful warning this script can emit.
    echo "[WARN] ${name}: node process not visible after ${START_CHILD_WAIT_SEC}s"
    echo "       Log: ${lf}"
    log_tail_current_session "${lf}" 20
  else
    # Appearing is not the same as surviving: a bad params file lets the
    # node start and then exit a second or two later. Without this settle
    # check the failure only surfaced on the PC as a vague readiness
    # timeout much further into the case.
    sleep "${START_SETTLE_SEC}"
    if ! pid_alive "${pid}" || [[ -z "$(node_pids_in_group "${name}" "${pid}")" ]]; then
      echo "[ERROR] ${name} exited ${START_SETTLE_SEC}s after startup."
      echo "        Log: ${lf}"
      log_tail_current_session "${lf}" 40
      rm -f "${pf}"
      return 1
    fi
  fi

  echo "[STARTED] ${name} (PID/PGID ${pid})"
  echo "          log: ${lf}"
}

stop_process() {
  local name="$1"

  local pf pid pgid spec sig window budget remaining ticks tick
  pf="$(pid_file "${name}")"

  if [[ ! -f "${pf}" ]]; then
    echo "[SKIP] ${name}: no PID file"
    return 0
  fi

  pid="$(cat "${pf}" 2>/dev/null || true)"

  if [[ ! "${pid}" =~ ^[0-9]+$ ]]; then
    echo "[WARN] ${name}: invalid PID file; removing"
    rm -f "${pf}"
    return 0
  fi

  if ! pid_alive "${pid}"; then
    echo "[SKIP] ${name}: stale/exited PID ${pid}; removing"
    rm -f "${pf}"
    return 0
  fi

  pgid="$(process_group_of "${pid}")"

  # kill -- -PID on a PID that is no longer its own group leader would
  # signal a completely unrelated process group. Refuse and let the
  # pattern-based sweep deal with it instead.
  if [[ -z "${pgid}" ]]; then
    echo "[SKIP] ${name}: PID ${pid} disappeared; removing PID file"
    rm -f "${pf}"
    return 0
  fi
  if [[ "${pgid}" != "${pid}" ]]; then
    echo "[WARN] ${name}: PID ${pid} is not its own process-group leader (pgid=${pgid})."
    echo "       Refusing to signal that group; leaving it to the orphan sweep."
    rm -f "${pf}"
    return 0
  fi

  # Every wait window is clamped to a budget shared by all processes, so a
  # hung node shortens the remaining graceful windows instead of pushing
  # the whole `stop` past the PC's SSH timeout.
  for spec in "INT:${STOP_SIGINT_MAX_SEC}:${STOP_DEADLINE}" \
              "TERM:${STOP_SIGTERM_MAX_SEC}:${STOP_HARD_DEADLINE}" \
              "KILL:${STOP_SIGKILL_SEC}:0"; do
    if group_gone "${pgid}"; then
      break
    fi

    sig="${spec%%:*}"
    window="$(printf '%s' "${spec}" | cut -d: -f2)"
    budget="$(printf '%s' "${spec}" | cut -d: -f3)"

    if [[ "${budget}" != "0" ]]; then
      remaining=$((budget - SECONDS))
      if (( remaining < 0 )); then
        remaining=0
      fi
      if (( remaining < window )); then
        window="${remaining}"
      fi
    fi

    # A signal is always delivered; only the wait can shrink to zero.
    echo "[STOP] ${name} (PGID ${pgid}) -> SIG${sig} (<=${window}s)"
    kill -"${sig}" -- "-${pgid}" 2>/dev/null || true

    ticks=$((window * 10))
    for tick in $(seq 1 "${ticks}"); do
      : "${tick}"
      if group_gone "${pgid}"; then
        break
      fi
      sleep 0.1
    done
  done

  if group_gone "${pgid}"; then
    rm -f "${pf}"
    echo "[STOPPED] ${name}"
    return 0
  fi

  echo "[ERROR] ${name}: process group ${pgid} survived SIGKILL: $(group_pids "${pgid}" | tr '\n' ' ')"
  rm -f "${pf}"
  return 1
}

wait_lifecycle_ready() {
  local node="$1"
  local timeout_sec="${2:-20}"
  local deadline=$((SECONDS + timeout_sec))

  while (( SECONDS < deadline )); do
    # A fresh ros2 CLI process can briefly miss the lifecycle service over
    # Zenoh. Bound each probe and retry until the overall deadline instead
    # of treating one transient miss as a node failure.
    if timeout 3s ros2 lifecycle get "/${node}" >/dev/null 2>&1; then
      return 0
    fi
    sleep 0.5
  done

  echo "[ERROR] Lifecycle service not ready: /${node}"
  return 1
}

get_lifecycle_state() {
  local node="$1"
  local attempts="${2:-2}"
  local output clean state attempt

  for attempt in $(seq 1 "${attempts}"); do
    : "${attempt}"
    output="$(timeout 3s ros2 lifecycle get "/${node}" 2>/dev/null || true)"

    if [[ -n "${output}" ]]; then
      # rmw_zenoh / ros2 CLI can occasionally emit a timestamp or another
      # diagnostic line on stdout before the actual lifecycle state. Never
      # return every first token; extract exactly one canonical
      # lifecycle-state line instead.
      clean="$(printf '%s\n' "${output}" | sed -E $'s/\x1B\[[0-9;]*[[:alpha:]]//g')"
      state="$(
        awk -v re="${LIFECYCLE_STATE_RE}" '
          $1 ~ re && (NF == 1 || $2 ~ /^\[[0-9]+\]$/) {
            print $1
            exit
          }
        ' <<< "${clean}"
      )"

      if [[ -n "${state}" ]]; then
        printf '%s\n' "${state}"
        return 0
      fi
    fi

    sleep 0.4
  done

  printf '%s\n' "UNAVAILABLE"
  return 1
}

# Resolve to a settled primary state, absorbing both mid-transition states
# and transient Zenoh CLI misses. v1.5 read the state exactly once and
# aborted the whole activate on a single miss.
wait_settled_state() {
  local node="$1"
  local timeout_sec="${2:-20}"
  local deadline=$((SECONDS + timeout_sec))
  local state="UNAVAILABLE"

  while (( SECONDS < deadline )); do
    state="$(get_lifecycle_state "${node}" 1 || true)"
    case "${state}" in
      unconfigured|inactive|active|finalized)
        printf '%s\n' "${state}"
        return 0
        ;;
    esac
    sleep 0.5
  done

  printf '%s\n' "${state}"
  return 1
}

require_state() {
  local node="$1"
  local expected="$2"
  local timeout_sec="${3:-45}"
  local deadline=$((SECONDS + timeout_sec))
  local actual="UNAVAILABLE"

  while (( SECONDS < deadline )); do
    actual="$(get_lifecycle_state "${node}" 1 || true)"
    if [[ "${actual}" == "${expected}" ]]; then
      echo "[OK] /${node}: ${actual}"
      return 0
    fi
    sleep 0.5
  done

  echo "[ERROR] /${node}: expected '${expected}', last state '${actual:-UNAVAILABLE}' after ${timeout_sec}s"
  return 1
}

start_localization() {
  echo "=== START LOCALIZATION STACK ==="

  start_process "dual_gps_heading" \
    "source ${HOME}/dual_gps_ws/install/setup.bash && \
     exec ros2 run dual_gps_heading heading_node \
       --ros-args \
       ${SIM_TIME_ARG}"

  start_process "global_ekf" \
    "exec ros2 run robot_localization ekf_node \
       --ros-args \
       -r __node:=ekf_filter_node_map \
       --params-file ${HOME}/gps_global.yaml \
       -r odometry/filtered:=/odometry/global \
       ${TF_REMAPS} \
       ${SIM_TIME_ARG}"

  start_process "navsat_transform" \
    "exec ros2 run robot_localization navsat_transform_node \
       --ros-args \
       -r __node:=navsat_transform \
       --params-file ${HOME}/gps_global.yaml \
       -r gps/fix:=/gps/front \
       -r imu:=/gps/heading \
       -r odometry/filtered:=/odometry/global \
       ${TF_REMAPS} \
       ${SIM_TIME_ARG}"

  echo "=== LOCALIZATION PROCESSES STARTED ==="
  echo "Runner should now wait for map -> base_link readiness."
}

start_nav2() {
  echo "=== START NAV2 PROCESSES (UNCONFIGURED) ==="

  PHASE_DEADLINE=$((SECONDS + START_NAV2_BUDGET_SEC))

  start_process "controller_server" \
    "exec ros2 run nav2_controller controller_server \
       --ros-args \
       --params-file ${HOME}/pil_controller.yaml \
       ${TF_REMAPS} \
       ${SIM_TIME_ARG} \
       -r /cmd_vel:=/cmd_vel_raw"

  start_process "planner_server" \
    "exec ros2 run nav2_planner planner_server \
       --ros-args \
       --params-file ${HOME}/pil_navigation.yaml \
       ${TF_REMAPS} \
       ${SIM_TIME_ARG}"

  start_process "behavior_server" \
    "source ${HOME}/adaptive_escape_ws/install/setup.bash && \
     exec ros2 run nav2_behaviors behavior_server \
       --ros-args \
       --params-file ${HOME}/pil_navigation.yaml \
       ${TF_REMAPS} \
       ${SIM_TIME_ARG} \
       -r /cmd_vel:=/cmd_vel_raw"

  start_process "collision_monitor" \
    "exec ros2 run nav2_collision_monitor collision_monitor \
       --ros-args \
       --params-file ${HOME}/collision_monitor.yaml \
       ${TF_REMAPS} \
       ${SIM_TIME_ARG}"

  start_process "bt_navigator" \
    "source ${HOME}/adaptive_escape_ws/install/setup.bash && \
     exec ros2 run nav2_bt_navigator bt_navigator \
       --ros-args \
       --params-file ${HOME}/pil_navigation.yaml \
       ${TF_REMAPS} \
       ${SIM_TIME_ARG}"

  echo
  echo "Waiting for lifecycle services..."
  for node in "${LIFECYCLE_NODES[@]}"; do
    wait_lifecycle_ready "${node}" "$(capped_remaining 20)"
    echo "[READY] /${node}"
  done

  echo "=== NAV2 PROCESSES READY FOR CONFIGURE ==="
}

configure_nav2() {
  echo "=== CONFIGURE NAV2 ==="

  PHASE_DEADLINE=$((SECONDS + LIFECYCLE_PHASE_BUDGET_SEC))

  local node state
  for node in "${LIFECYCLE_NODES[@]}"; do
    wait_lifecycle_ready "${node}" "$(capped_remaining 20)"

    state="$(wait_settled_state "${node}" "$(capped_remaining 15)" || true)"

    case "${state}" in
      inactive)
        echo "[SKIP] /${node} already inactive (configured)"
        continue
        ;;
      active)
        echo "[SKIP] /${node} already active"
        continue
        ;;
      unconfigured)
        ;;
      *)
        echo "[ERROR] /${node}: cannot configure from state '${state}'"
        return 1
        ;;
    esac

    echo "[CONFIGURE] /${node}"
    # The CLI can report failure for a transition the node still completes.
    # require_state is the authority.
    ros2 lifecycle set "/${node}" configure || true
    require_state "${node}" "inactive" "$(capped_remaining 45)"
  done

  echo "=== NAV2 CONFIGURED ==="
}

activate_nav2() {
  echo "=== ACTIVATE NAV2 ==="

  PHASE_DEADLINE=$((SECONDS + LIFECYCLE_PHASE_BUDGET_SEC))

  # bt_navigator is deliberately last (LIFECYCLE_NODES order).
  local node state
  for node in "${LIFECYCLE_NODES[@]}"; do
    wait_lifecycle_ready "${node}" "$(capped_remaining 20)"

    state="$(wait_settled_state "${node}" "$(capped_remaining 15)" || true)"

    case "${state}" in
      active)
        echo "[SKIP] /${node} already active"
        continue
        ;;
      inactive)
        ;;
      *)
        echo "[ERROR] /${node} is not configured: ${state}"
        return 1
        ;;
    esac

    echo "[ACTIVATE] /${node}"
    ros2 lifecycle set "/${node}" activate || true
    require_state "${node}" "active" "$(capped_remaining 45)"
  done

  echo "=== ALL NAV2 LIFECYCLE NODES ACTIVE ==="
}

start_far_goal() {
  echo "=== START FAR GOAL MANAGER ==="

  start_process "far_goal" \
    "source ${HOME}/far_goal_ws/install/setup.bash && \
     exec ros2 run far_goal_manager far_goal_manager \
       --ros-args \
       ${SIM_TIME_ARG} \
       ${TF_REMAPS}"

  echo "=== FAR GOAL MANAGER STARTED ==="
}

verify_nav2_ready() {
  echo "============================================================"
  echo "NAV2 READINESS VERIFY"
  echo "============================================================"

  local failed=0
  local name node state

  # A dead EKF used to pass verify and only surface as a mid-run
  # INFRA_ERROR when map->base_link went stale.
  for name in "${LOCALIZATION_PROCESSES[@]}"; do
    if process_healthy "${name}"; then
      printf '[OK] %-19s RUNNING\n' "${name}"
    else
      printf '[ERROR] %-16s not healthy\n' "${name}"
      failed=1
    fi
  done

  for node in "${LIFECYCLE_NODES[@]}"; do
    if ! process_healthy "${node}"; then
      printf '[ERROR] /%-16s process not healthy\n' "${node}"
      failed=1
      continue
    fi

    state="$(get_lifecycle_state "${node}" 1 || true)"
    if [[ "${state}" == "active" ]]; then
      printf '[OK] /%-19s active\n' "${node}"
    else
      printf '[ERROR] /%-16s expected active, got %s\n' \
        "${node}" "${state:-UNAVAILABLE}"
      failed=1
    fi
  done

  if process_healthy "far_goal"; then
    echo "[OK] far_goal RUNNING"
  else
    echo "[ERROR] far_goal is not RUNNING"
    failed=1
  fi

  if (( failed != 0 )); then
    echo "NAV2_NOT_READY"
    return 1
  fi

  echo "NAV2_READY"
}

status_processes() {
  echo "============================================================"
  echo "PROCESS STATUS"
  echo "============================================================"

  local name pf node state

  for name in "${MANAGED_PROCESSES[@]}"; do
    pf="$(pid_file "${name}")"

    if process_healthy "${name}"; then
      printf '%-22s RUNNING  PID=%s\n' "${name}" "$(cat "${pf}")"
    elif is_running "${name}"; then
      printf '%-22s RUNNING_NO_NODE  PID=%s\n' "${name}" "$(cat "${pf}")"
    elif [[ -f "${pf}" ]]; then
      printf '%-22s STALE_PID\n' "${name}"
    else
      printf '%-22s STOPPED\n' "${name}"
    fi
  done

  echo
  echo "============================================================"
  echo "LIFECYCLE STATUS"
  echo "============================================================"

  for node in "${LIFECYCLE_NODES[@]}"; do
    state="$(get_lifecycle_state "${node}" 1 || true)"
    printf '%-22s %s\n' "/${node}" "${state:-UNAVAILABLE}"
  done

  echo
  echo "hil_router is intentionally NOT managed by this script."
}

managed_pids_for_pattern() {
  local pattern="$1"
  local ancestors=" $$"
  local parent="${PPID}"
  local next_parent pid

  # Never select this manager shell or any SSH/login shell that invoked it,
  # even if a diagnostic/test command line happens to contain our pattern.
  while [[ "${parent}" =~ ^[0-9]+$ ]] && (( parent > 1 )); do
    ancestors+=" ${parent}"
    next_parent="$(ps -o ppid= -p "${parent}" 2>/dev/null | tr -d ' ' || true)"
    [[ "${next_parent}" =~ ^[0-9]+$ ]] || break
    parent="${next_parent}"
  done

  for pid in $(pgrep -f -- "${pattern}" 2>/dev/null || true); do
    if [[ " ${ancestors} " == *" ${pid} "* ]]; then
      continue
    fi
    # A zombie is already dead; counting it as a surviving orphan would
    # make the sweep report a false failure.
    if ! pid_alive "${pid}"; then
      continue
    fi
    printf '%s\n' "${pid}"
  done
}

stop_orphaned_managed_processes() {
  echo "=== ORPHAN SWEEP (benchmark-managed processes only) ==="

  local name pattern pids
  local found=0

  for name in "${MANAGED_PROCESSES[@]}"; do
    pattern="${PROCESS_MATCH[${name}]}"
    pids="$(managed_pids_for_pattern "${pattern}")"
    [[ -n "${pids}" ]] || continue

    found=1
    echo "[ORPHAN] ${name} pids=${pids//$'\n'/,}"
    # shellcheck disable=SC2086
    kill -INT ${pids} 2>/dev/null || true
  done

  if (( found == 0 )); then
    echo "[OK] no orphaned managed processes"
    return 0
  fi

  sleep 1.0

  for name in "${MANAGED_PROCESSES[@]}"; do
    pattern="${PROCESS_MATCH[${name}]}"
    pids="$(managed_pids_for_pattern "${pattern}")"
    [[ -n "${pids}" ]] || continue
    echo "[ORPHAN] still alive -> SIGTERM ${name} pids=${pids//$'\n'/,}"
    # shellcheck disable=SC2086
    kill -TERM ${pids} 2>/dev/null || true
  done

  sleep 1.0

  for name in "${MANAGED_PROCESSES[@]}"; do
    pattern="${PROCESS_MATCH[${name}]}"
    pids="$(managed_pids_for_pattern "${pattern}")"
    [[ -n "${pids}" ]] || continue
    echo "[ORPHAN] still alive -> SIGKILL ${name} pids=${pids//$'\n'/,}"
    # shellcheck disable=SC2086
    kill -KILL ${pids} 2>/dev/null || true
  done

  sleep 0.3

  local leftovers=0
  for name in "${MANAGED_PROCESSES[@]}"; do
    pattern="${PROCESS_MATCH[${name}]}"
    pids="$(managed_pids_for_pattern "${pattern}")"
    if [[ -n "${pids}" ]]; then
      echo "[ERROR] orphan remains ${name} pids=${pids//$'\n'/,}"
      leftovers=1
    fi
  done

  if (( leftovers != 0 )); then
    return 1
  fi

  echo "[OK] orphan sweep complete"
}

publish_zero_velocity_best_effort() {
  # Clear a retained actuator command even when Nav2 is about to disappear.
  # This is intentionally best-effort: stop must still work if Gazebo is gone.
  local msg="{header: {frame_id: ''}, twist: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}"

  timeout 1.2s ros2 topic pub --rate 20 \
    /cmd_vel_raw geometry_msgs/msg/TwistStamped "${msg}" \
    >/dev/null 2>&1 || true

  timeout 1.2s ros2 topic pub --rate 20 \
    /a200_0000/platform/cmd_vel geometry_msgs/msg/TwistStamped "${msg}" \
    >/dev/null 2>&1 || true
}

stop_all() {
  echo "=== STOP MANAGED STACK ==="

  local status=0
  local name

  STOP_DEADLINE=$((SECONDS + STOP_GRACE_BUDGET_SEC))
  STOP_HARD_DEADLINE=$((SECONDS + STOP_HARD_BUDGET_SEC))

  echo "[SAFETY] Clearing velocity commands before shutdown"
  publish_zero_velocity_best_effort

  for name in "${STOP_ORDER[@]}"; do
    if ! stop_process "${name}"; then
      status=1
    fi
  done

  # PID files cover normal benchmark shutdown. The sweep below handles a
  # process left behind by a crashed runner, a lost/stale PID file, or a
  # manually launched duplicate. hil_router is deliberately excluded.
  if ! stop_orphaned_managed_processes; then
    status=1
  fi

  # v1.5 let a failing sweep abort the function through set -e, which
  # skipped this final safety publish. Shutdown safety must not depend on
  # cleanup succeeding.
  echo "[SAFETY] Clearing final platform command after shutdown"
  publish_zero_velocity_best_effort

  if (( status != 0 )); then
    echo "=== MANAGED STACK STOP INCOMPLETE ==="
    echo "hil_router was NOT touched."
    return 1
  fi

  echo "=== MANAGED STACK STOPPED ==="
  echo "hil_router was NOT touched."
}

clean_stale_pids() {
  local pf pid

  for pf in "${PID_DIR}"/*.pid; do
    [[ -e "${pf}" ]] || continue
    pid="$(cat "${pf}" 2>/dev/null || true)"
    if [[ ! "${pid}" =~ ^[0-9]+$ ]] || ! pid_alive "${pid}"; then
      rm -f "${pf}"
    fi
  done
}

usage() {
  cat <<'EOF'
Usage:
  pi_stack_manager.sh start_localization
  pi_stack_manager.sh start_nav2
  pi_stack_manager.sh configure
  pi_stack_manager.sh activate
  pi_stack_manager.sh start_far_goal
  pi_stack_manager.sh status
  pi_stack_manager.sh verify
  pi_stack_manager.sh stop

Recommended benchmark-case sequence:

  1) pi_stack_manager.sh stop
  2) PC starts fresh Gazebo case + ros_gz_bridge
  3) pi_stack_manager.sh start_localization
  4) PC/Runner verifies map -> base_link
  5) pi_stack_manager.sh start_nav2
  6) pi_stack_manager.sh configure
  7) pi_stack_manager.sh activate
  8) pi_stack_manager.sh start_far_goal
  9) pi_stack_manager.sh verify
 10) PC starts benchmark_logger
 11) PC sends validated case goal

NOTE:
  hil_router must already be running and is never stopped by this script.
EOF
}

# status() must be able to report STALE_PID, so stale PID files are only
# reaped for actions that actually start or stop something.
case "${ACTION}" in
  status|verify) : ;;
  *) clean_stale_pids ;;
esac

case "${ACTION}" in
  start_localization)
    start_localization
    ;;
  start_nav2)
    start_nav2
    ;;
  configure)
    configure_nav2
    ;;
  activate)
    activate_nav2
    ;;
  start_far_goal)
    start_far_goal
    ;;
  status)
    status_processes
    ;;
  verify)
    verify_nav2_ready
    ;;
  stop)
    stop_all
    ;;
  *)
    usage
    exit 2
    ;;
esac