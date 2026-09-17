# A200 Dynamic Scan Filter v0.4 — Fast Moving→Stopped Handoff

## What v0.4 fixes

The observed failure was not that Collision Monitor stopped the robot.
Collision Monitor is supposed to stop when the raw LiDAR sees an obstacle too close.

The real ownership gap was:

```text
moving obstacle
  -> hidden from GLOBAL costmap marking
  -> local / predictive layers handle it

obstacle physically stops
  -> Collision Monitor / local costmap see it immediately from raw /scan
  -> but GLOBAL costmap can still be masking it as "dynamic"
  -> global path can continue through the stopped obstacle
  -> robot stops locally instead of receiving a clean static-obstacle detour
```

v0.4 shortens that moving→stopped recognition delay without weakening Collision Monitor.

## Design

### DYNAMIC entry: still conservative

A new / resumed moving object still requires:

- tracker speed >= `dynamic_enter_speed_mps`
- actual observed displacement
- after a previous static handoff, the long rolling motion estimator must confirm motion

So a stale high tracker velocity cannot immediately hide a stationary obstacle again.

### DYNAMIC -> STATIC exit: faster and asymmetric

While an object is already classified dynamic, v0.4 additionally evaluates a short
0.8 s observed-position window.

The short estimator:

1. uses actual tracked `(t, x, y)` history, not only tracker velocity,
2. applies a 3-point moving average to reduce cluster-centroid jitter,
3. measures:
   - short-window trend speed,
   - short-window path activity,
4. requires 2 of the latest 3 valid windows to agree before handoff.

Current defaults:

```yaml
quick_motion_window_sec: 0.80
quick_motion_min_span_sec: 0.65
quick_motion_min_samples: 7
quick_stationary_vote_window: 3
quick_stationary_votes_required: 2
quick_stationary_trend_speed_mps: 0.08
quick_stationary_activity_speed_mps: 0.11
```

The older 1.5 s rolling estimator and tracker-speed exit remain as fallback paths.

## Important anti-chatter fix

When `DYNAMIC -> STATIC` occurs, pre-handoff motion history is discarded.

Without this, the object could be handed to the static costmap and then immediately
flip back to DYNAMIC because the long estimator still contained its old moving samples.

After handoff, DYNAMIC re-entry must be proven from fresh observations.

## What is NOT changed

v0.4 does not change:

- Collision Monitor
- predictive safety
- DynamicObstacleCritic
- DWB parameters
- local costmap
- NavFn / planner parameters
- BT
- Far Goal Manager
- Adaptive Escape
- tracker parameters
- benchmark case logic

There are no S5/S6/case IDs or semantic states such as FOLLOW / HEAD_ON in the runtime code.

This change is specifically meant to let the normal navigation stack recognize
"this object is now effectively static" early enough for the global costmap/path
to react before the robot remains pinned behind it.

## Apply

Replace:

```text
~/predictive_safety_ws/src/a200_dynamic_scan_filter
```

with this package.

Then build:

```bash
cd ~/predictive_safety_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash 2>/dev/null || true

colcon build --symlink-install \
  --packages-select a200_dynamic_scan_filter

source install/setup.bash
```

No additional `pi_stack_manager.sh` change is required if the v1.8 manager that
already launches `dynamic_scan_filter` is installed.

No additional `pil_navigation.yaml` change is required if GLOBAL marking already
uses `/scan_static_mark` and GLOBAL clearing remains raw `/scan`.

## First validation

Run S5_06 first with all other parameters unchanged.

Check:

```bash
grep -E "ownership|active_tracks" \
  ~/nav_benchmark/logs/dynamic_scan_filter.log | tail -80
```

Expected sequence:

```text
ownership STATIC -> DYNAMIC
...
ownership DYNAMIC -> STATIC
```

The transition log now prints both the short and long estimators:

```text
quick_valid
quick_trend
quick_activity
quick_span
quick_samples
long_valid
long_trend
long_activity
long_span
long_samples
```

The important success condition is not merely seeing `DYNAMIC -> STATIC`.

After that transition:

1. the stopped obstacle should re-enter GLOBAL costmap marking,
2. the current path should become invalid / be replanned,
3. DWB should receive an actual detour,
4. the robot should continue to the destination.

If `DYNAMIC -> STATIC` appears correctly but the robot still cannot complete the
route, the next bottleneck is no longer stop recognition; then inspect Global Path,
DWB legal trajectories, and Collision Monitor geometry around the detour.

## QA scope

See `QA_REPORT_V04.md`.

The motion core was compiled and tested here with GCC and Clang, warnings-as-errors,
randomized stress tests, and Address/UndefinedBehavior sanitizers.

The full ROS 2 node still must be built on the Pi because this execution environment
does not contain the project's ROS 2 Jazzy / `prox_mpc_msgs` installation.
