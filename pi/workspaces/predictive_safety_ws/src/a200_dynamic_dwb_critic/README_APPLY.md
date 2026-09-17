# A200 Dynamic DWB Critic v0.1

## What this changes

This package adds one DWB `TrajectoryCritic`.

It does **not** replace or edit the existing:

- obstacle tracker
- `a200_predictive_collision_monitor`
- Collision Monitor
- Far Goal Manager
- S1-S4 costmap pipeline

The critic has no benchmark case ID and no explicit behavior labels such as
`FOLLOW`, `CROSSING`, or `HEAD_ON`.

Every DWB candidate is scored from the same physical quantities:

- future robot candidate poses
- tracked obstacle position / velocity
- IMM `predicted_positions`
- obstacle radius
- tracker position/velocity covariance as a bounded uncertainty margin
- relative closing speed
- future clearance
- TTC-like time to consume the remaining clearance

## 1. Copy this new package

Create:

```bash
mkdir -p ~/predictive_safety_ws/src/a200_dynamic_dwb_critic
```

Copy this package into that directory.

## 2. Build

```bash
cd ~/predictive_safety_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash 2>/dev/null || true

colcon build --symlink-install --packages-up-to a200_dynamic_dwb_critic
source install/setup.bash
```

## 3. One pi_stack_manager change

The controller process must source the overlay so pluginlib can discover this
new library.

Current beginning:

```bash
start_process "controller_server" \
  "exec ros2 run nav2_controller controller_server \
```

Change only that beginning to:

```bash
start_process "controller_server" \
  "source ${HOME}/predictive_safety_ws/install/setup.bash && \
   exec ros2 run nav2_controller controller_server \
```

Keep all existing params, TF remaps, sim-time arguments and the
`/cmd_vel:=/cmd_vel_predictive_in` remap unchanged.

## 4. pil_controller.yaml

Inside `FollowPath.critics`, add only `DynamicObstacle` immediately after
`ObstacleFootprint`:

```yaml
critics:
  - RotateToGoal
  - Oscillation
  - ObstacleFootprint
  - DynamicObstacle
  - GoalAlign
  - PathAlign
  - PathDist
  - GoalDist
```

Then add this under `FollowPath:`:

```yaml
DynamicObstacle.class: "a200_dynamic_dwb_critic::DynamicObstacleCritic"
DynamicObstacle.scale: 6.0

DynamicObstacle.tracked_obstacles_topic: /tracked_obstacles
DynamicObstacle.track_timeout_sec: 0.75
DynamicObstacle.future_stamp_tolerance_sec: 0.10
DynamicObstacle.minimum_dynamic_speed_mps: 0.15

DynamicObstacle.robot_half_length_m: 0.494
DynamicObstacle.robot_half_width_m: 0.335
DynamicObstacle.minimum_obstacle_radius_m: 0.10

DynamicObstacle.hard_safety_margin_m: 0.10
DynamicObstacle.soft_clearance_m: 1.20
DynamicObstacle.uncertainty_sigma_multiplier: 1.0
DynamicObstacle.maximum_uncertainty_margin_m: 0.35

# v0.1 reconstructs candidate timestamps from this value.
# Keep it equal to FollowPath.sim_time.
DynamicObstacle.trajectory_horizon_sec: 1.70

DynamicObstacle.ttc_horizon_sec: 4.0
DynamicObstacle.proximity_weight: 1.0
DynamicObstacle.ttc_weight: 1.5
DynamicObstacle.overlap_penalty: 25.0
DynamicObstacle.future_discount_per_sec: 0.15
```

Do not change the existing critic scales yet.

## 5. Validation phase A: additive only

Keep the current `crossing_yield_enabled: true`.

Run only:

```text
S1_01
S5_01
```

This phase is only to prove that the plugin loads and the known static/basic
dynamic baselines do not regress.

## 6. Validation phase B: DWB owns the motion choice

Only after phase A passes, change this **one** existing parameter:

```yaml
crossing_yield_enabled: false
```

Do not disable the predictive monitor. It still evaluates the command selected
by DWB and can enforce PASS/SLOW/STOP. The official Collision Monitor remains
after it.

Then run:

```text
S5_01
S5_06
S5_08
S1_01
```

If S5_01 becomes unsafe, immediately restore `crossing_yield_enabled: true`.

## Expected S5_06 behavior

There is no `FOLLOW` state.

When the actor merges and moves ahead:

- fast candidates close the gap -> high dynamic risk
- moderate candidates preserve gap -> lower dynamic risk
- predicted overlap -> very large candidate cost
- if the actor pulls away -> dynamic score approaches zero

So DWB can choose a safe non-zero velocity instead of waiting for the actor to
leave the corridor.

## Known next bottleneck: current local costmap

The current LiDAR obstacle layer still marks a moving obstacle at its
instantaneous position. Therefore the existing `ObstacleFootprint` critic may
still reject a DWB trajectory even when `DynamicObstacleCritic` predicts that
the obstacle will have moved away.

Do **not** remove or weaken `ObstacleFootprint` yet.

If S5_06 still waits in phase B, compare:

```text
/cmd_vel_predictive_in  # direct DWB output
/cmd_vel_raw            # after predictive safety
```

Interpretation:

- DWB nonzero, raw zero -> predictive safety is still the limiter.
- DWB itself zero -> instantaneous local-costmap marking is the next limiter.

Only in the second case should the next change be dynamic/static costmap
separation.
