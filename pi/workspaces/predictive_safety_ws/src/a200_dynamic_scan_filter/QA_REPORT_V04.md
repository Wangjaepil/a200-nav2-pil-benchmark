# QA Report — a200_dynamic_scan_filter v0.4

## Result

PASS for the standalone motion-ownership core.

## Compiler QA

Both compiler paths were checked with:

```text
-std=c++17 -Wall -Wextra -Wpedantic -Werror
```

- GCC: PASS
- Clang: PASS

## Deterministic QA

PASS:

- fast moving→stopped recognition with stale tracker velocity
- brief 0.3 s hesitation does not release dynamic ownership
- continuous 0.12 m/s motion remains dynamic
- direction reversal / oscillation remains dynamic
- no immediate DYNAMIC re-entry after static handoff
- actual resumed motion is eventually reacquired
- legacy low-tracker-speed exit still works
- timestamp reset fails safe
- static centroid noise does not enter dynamic ownership
- stale track pruning removes masking state
- LaserScan dynamic marking uses NaN while preserving +inf / NaN semantics

Result:

```text
DYNAMIC_SCAN_FILTER_V04_QA_OK
```

## Randomized stress QA

1000 deterministic-seed trials for each category:

```text
typical_stop_fast=1000/1000
extreme_stop_eventual=1000/1000
moving_retained=1000/1000
reversal_retained=1000/1000
brief_pause_retained=1000/1000
stop_resume_reacquired=1000/1000
DYNAMIC_SCAN_FILTER_V04_STRESS_OK
```

Interpretation:

- typical stopped object with up to ±3 cm synthetic centroid jitter:
  fast handoff within the tested 1.2 s bound
- extreme ±4 cm synthetic centroid jitter:
  eventual handoff still succeeds via short or retained long-window logic
- continuous 0.12–0.50 m/s motion:
  no false static handoff in the stress set
- reversing motion:
  no false static handoff
- 0.2–0.4 s hesitation:
  no false static handoff
- stop→static→resume:
  no chatter from stale velocity and dynamic ownership is reacquired from fresh motion

These synthetic noise envelopes are QA stimuli, not measured guarantees about the
real tracker distribution.

## Sanitizer QA

GCC AddressSanitizer + UndefinedBehaviorSanitizer:

```text
DYNAMIC_SCAN_FILTER_V04_SANITIZER_OK
```

## Package / script QA

- `package.xml` XML parse: PASS
- `run_core_standalone_test.sh` shell syntax: PASS
- runtime source scan for benchmark-specific S5/S05/case/scenario branching: no matches

## Important limitation

This environment does not have the user's ROS 2 Jazzy workspace or `prox_mpc_msgs`,
so the full ROS node executable cannot be linked here.

The Pi command below is the final integration compile gate:

```bash
cd ~/predictive_safety_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash 2>/dev/null || true
colcon build --symlink-install --packages-select a200_dynamic_scan_filter
```

After that, S5_06 is the required system-level QA because it validates the full chain:

```text
tracker -> fast static handoff -> global costmap -> IsPathValid/replan
-> DWB -> Collision Monitor -> destination completion
```
