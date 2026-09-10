#!/usr/bin/env python3
# Benchmark Logger v2.0 - mission-aware termination, health checks, and /plan recording.
import sys
import csv
import math
import time
from pathlib import Path

import yaml
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
    qos_profile_sensor_data,
)
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path as NavPath
from action_msgs.msg import GoalStatusArray, GoalStatus
from rcl_interfaces.msg import Log
from tf2_ros import Buffer, TransformListener

# Published by nav2_collision_monitor (state_topic in collision_monitor.yaml).
# It reports exactly when the safety layer overrode navigation, which is the
# most direct evidence of how hard the avoidance had to work.
try:  # pragma: no cover - depends on the installed nav2 version
    from nav2_msgs.msg import CollisionMonitorState
except ImportError:  # pragma: no cover
    CollisionMonitorState = None

from benchmark_common import (
    SUITE_VERSION,
    load_case_spec,
    make_run_dir,
    navigation_terminal_policy,
    normalize_angle,
    quaternion_to_yaw as quat_to_yaw,
    suite_metadata,
)

# rclpy on Jazzy does not export qos_profile_rosout_default (rclcpp has
# RosoutQoS, rclpy does not).  Importing it fails at module load, so rebuild
# rcl_qos_profile_rosout_default here: the subscription must match what the
# /rosout publishers offer, or the costmap-clear counter never sees anything.
try:  # pragma: no cover - distribution dependent
    from rclpy.qos import qos_profile_rosout_default as ROSOUT_QOS
except ImportError:  # pragma: no cover - Jazzy and friends
    ROSOUT_QOS = QoSProfile(
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=1000,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        lifespan=Duration(seconds=10),
    )


TERMINAL = {
    GoalStatus.STATUS_SUCCEEDED,
    GoalStatus.STATUS_CANCELED,
    GoalStatus.STATUS_ABORTED,
}
STATUS_NAME = {
    GoalStatus.STATUS_UNKNOWN: "UNKNOWN",
    GoalStatus.STATUS_ACCEPTED: "ACCEPTED",
    GoalStatus.STATUS_EXECUTING: "EXECUTING",
    GoalStatus.STATUS_CANCELING: "CANCELING",
    GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
    GoalStatus.STATUS_CANCELED: "CANCELED",
    GoalStatus.STATUS_ABORTED: "ABORTED",
}


def collision_action_names(message_class):
    """Read the action-type constants off the message class itself.

    The numeric values have moved between nav2 releases, so never hardcode
    them: whatever constants the installed message defines are authoritative.
    """
    names = {}
    if message_class is None:
        return names
    for attribute in dir(message_class):
        if not attribute.isupper():
            continue
        value = getattr(message_class, attribute, None)
        if isinstance(value, int) and not isinstance(value, bool):
            names.setdefault(value, attribute)
    return names


COLLISION_ACTION_NAMES = collision_action_names(CollisionMonitorState)


def stamp_to_ns(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def uuid_hex(goal_id):
    return bytes(goal_id.uuid).hex()


class BenchmarkLogger(Node):
    SAMPLE_PERIOD_SEC = 0.10
    TERMINAL_GRACE_SEC = 3.0
    INTERMEDIATE_TERMINAL_WAIT_SEC = 12.0
    TIMEOUT_WALL_SEC = 600.0
    FINAL_NEAR_DISTANCE_M = 0.50
    NEAR_SUCCESS_XY_M = 0.10
    NEAR_SUCCESS_YAW_DEG = 5.0
    ODOM_MAX_STEP_M = 0.50

    # --- Safety measurement -------------------------------------------
    # Clearance is measured from the ROBOT FOOTPRINT, not from the LiDAR
    # origin.  A 0.5 m LiDAR return in front of a 0.988 m long robot is only
    # 0.6 cm of real clearance, so min_lidar_range_m massively overstates
    # safety and must not be read as a clearance figure.
    # These must match the costmap footprint in pil_controller.yaml.
    FOOTPRINT_HALF_LENGTH_M = 0.494
    FOOTPRINT_HALF_WIDTH_M = 0.335
    # Returns closer than this are the robot seeing itself. The navigation
    # stack discards them too (obstacle_min_range in the costmap), so the
    # measurement stays consistent with what Nav2 actually reacted to.
    SELF_RETURN_MIN_M = 0.12
    # Time spent closer than this counts as "operating without margin".
    SAFETY_MARGIN_M = 0.20

    # --- Motion quality ------------------------------------------------
    # Path smoothness is measured from POSITION, not from the yaw estimate.
    # Summing |dyaw| at 10 Hz accumulates localization jitter: a real 60 deg
    # turn came out as 236 deg because 775 samples of small tremor add up.
    # Re-anchoring every SMOOTHNESS_STEP_M and taking the heading of the
    # displacement measures actual path curvature instead.
    SMOOTHNESS_STEP_M = 0.05
    STOPPED_SPEED_MPS = 0.02
    STOPPED_YAW_RATE_RADPS = 0.05
    CMD_EPS = 1e-3
    # /cmd_vel_raw and the final command arrive on different topics at
    # different instants, so "the two latest values differ" counts sampling
    # skew, not intervention. Measure how much speed was actually removed.
    CMD_MOVING_EPS = 0.01
    HEALTH_STARTUP_GRACE_SEC = 5.0
    DATA_STALE_WALL_SEC = 5.0
    STATUS_FUTURE_TOLERANCE_SEC = 2.0

    def __init__(self, case_id):
        super().__init__("benchmark_logger")

        case_spec = load_case_spec(case_id)
        self.case_id = case_spec.case_id
        self.scenario = case_spec.scenario
        self.case = case_spec.raw
        self.world_start_x = case_spec.robot.x
        self.world_start_y = case_spec.robot.y
        self.world_start_yaw = case_spec.robot.yaw
        self.world_goal_x = case_spec.goal.x
        self.world_goal_y = case_spec.goal.y
        self.world_goal_yaw = case_spec.goal.yaw
        self.direct_distance = case_spec.direct_distance_m

        case_root = (
            Path.home() / "nav_benchmark" / "results"
            / self.scenario / self.case_id
        )
        self.result_dir = make_run_dir(case_root)
        self.run_id = self.result_dir.name
        self.summary_path = self.result_dir / "summary.yaml"

        self.traj_file = (self.result_dir / "trajectory.csv").open(
            "w", newline="", encoding="utf-8"
        )
        self.traj_writer = csv.writer(self.traj_file)
        self.traj_writer.writerow([
            "time_sec",
            "map_x", "map_y", "map_yaw_rad",
            "odom_x", "odom_y", "odom_yaw_rad",
            "raw_vx", "raw_wz",
            "final_vx", "final_wz",
            "lidar_min_current_m",
            "clearance_m",
            "odom_vx_mps",
            "odom_wz_radps",
            "collision_monitor_action",
        ])

        self.event_file = (self.result_dir / "events.csv").open(
            "w", newline="", encoding="utf-8"
        )
        self.event_writer = csv.writer(self.event_file)
        self.event_writer.writerow(["time_sec", "event", "detail"])

        self.plan_file = (self.result_dir / "planned_paths.csv").open(
            "w", newline="", encoding="utf-8"
        )
        self.plan_writer = csv.writer(self.plan_file)
        self.plan_writer.writerow([
            "time_sec",
            "plan_id",
            "nav_goal_index",
            "frame_id",
            "source_topic",
            "pose_index",
            "x",
            "y",
            "yaw_rad",
        ])

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.started = False
        self.finished = False
        self.files_closed = False
        self.start_sim_ns = None
        self.end_sim_ns = None
        self.start_wall = None
        self.end_wall = None
        self.first_nav_sim_ns = None
        self.final_goal = None
        self.infra_failure_reason = None

        self.last_map_pose = None
        self.last_odom_pose = None
        self.prev_odom_xy = None
        self.travel_distance = 0.0
        self.odom_jump_count = 0

        self.latest_lidar_min = math.nan
        self.run_min_lidar = math.inf

        # Footprint clearance
        self.lidar_tf = None
        self.lidar_tf_frame = None
        self.lidar_tf_warned = False
        self.latest_clearance = math.nan
        self.run_min_clearance = math.inf
        self.clearance_sum = 0.0
        self.clearance_samples = 0
        self.clearance_values = []
        self.time_below_margin_sec = 0.0
        self.collision_suspected_count = 0
        self.min_clearance_at_sec = None
        self.min_clearance_pose = None

        # Actual motion (odometry twist), not commanded
        self.odom_vx = math.nan
        self.odom_wz = math.nan
        self.odom_twist_count = 0
        self.speed_sum = 0.0
        self.speed_samples = 0
        self.max_speed = 0.0
        self.abs_yaw_rate_sum = 0.0
        self.moving_time_sec = 0.0
        self.stopped_time_sec = 0.0
        self.prev_odom_vx = None
        self.prev_odom_wz = None
        self.accel_sq_sum = 0.0
        self.ang_accel_sq_sum = 0.0
        self.accel_samples = 0
        self.direction_reversal_count = 0
        self.heading_change_total = 0.0
        self.smoothness_anchor_xy = None
        self.smoothness_prev_heading = None
        self.last_sample_sim_sec = None

        # Safety-layer intervention
        self.collision_monitor_seen = False
        self.collision_monitor_action = ""
        self.collision_monitor_counts = {}
        self.collision_monitor_intervention_sec = 0.0
        self.cmd_stop_forced_samples = 0
        self.cmd_moving_samples = 0
        self.cmd_reduction_sum = 0.0
        self.cmd_reduction_max = 0.0
        self.raw_vx = math.nan
        self.raw_wz = math.nan
        self.final_vx = math.nan
        self.final_wz = math.nan

        self.scan_count = 0
        self.raw_cmd_count = 0
        self.final_cmd_count = 0
        self.trajectory_sample_count = 0
        self.plan_message_count = 0
        self.plan_pose_count = 0
        self.planned_path_topics = set()

        self.last_scan_wall = None
        self.last_map_tf_wall = None
        self.last_odom_tf_wall = None

        # Costmap clear recovery counting is observed from /rosout.
        # Nav2 ClearEntireCostmap service callbacks log a message when a
        # local/global costmap clear request is actually received.
        self.local_costmap_clear_count = 0
        self.global_costmap_clear_count = 0
        self._seen_clear_logs = set()

        self.behavior_counts = {
            "spin": 0,
            "backup": 0,
            "wait": 0,
            "adaptive_escape": 0,
        }
        self.behavior_states = {name: {} for name in self.behavior_counts}

        self.nav_states = {}
        self.latest_nav_stamp_ns = -1
        self.latest_nav_goal_id = None
        self.nav_goal_count = 0
        self.nav_goal_indices = {}
        self._ignored_nav_status_ids = set()
        self.pending_terminal = None
        self.nav_executing_seen = False

        self.create_subscription(
            PoseStamped, "/far_goal_pose", self.goal_cb, 10
        )
        self.create_subscription(
            LaserScan, "/scan", self.scan_cb, qos_profile_sensor_data
        )
        self.create_subscription(
            TwistStamped, "/cmd_vel_raw", self.raw_cmd_cb, 10
        )
        self.create_subscription(
            TwistStamped,
            "/a200_0000/platform/cmd_vel",
            self.final_cmd_cb,
            10,
        )
        self.create_subscription(
            GoalStatusArray,
            "/navigate_to_pose/_action/status",
            self.nav_status_cb,
            10,
        )
        self.create_subscription(NavPath, "/plan", self.plan_cb, 10)
        self.create_subscription(
            Odometry,
            "/a200_0000/platform/odom",
            self.odom_twist_cb,
            10,
        )
        if CollisionMonitorState is not None:
            self.create_subscription(
                CollisionMonitorState,
                "/collision_monitor_state",
                self.collision_monitor_cb,
                10,
            )
        self.create_subscription(
            Log,
            "/rosout",
            self.rosout_cb,
            ROSOUT_QOS,
        )

        self.add_behavior_status("spin", "/spin/_action/status")
        self.add_behavior_status("backup", "/backup/_action/status")
        self.add_behavior_status("wait", "/wait/_action/status")
        self.add_behavior_status(
            "adaptive_escape", "/adaptive_escape/_action/status"
        )

        self.timer = self.create_timer(
            self.SAMPLE_PERIOD_SEC, self.timer_cb
        )

        self.get_logger().info(
            f"Benchmark Logger ready: suite={SUITE_VERSION} / "
            f"{self.case_id} / {self.run_id}"
        )
        self.get_logger().info(
            f"Direct distance: {self.direct_distance:.2f} m"
        )
        self.get_logger().info(f"Output: {self.result_dir}")
        self.get_logger().info("Waiting for /far_goal_pose ...")

    def now_sim_ns(self):
        return self.get_clock().now().nanoseconds

    def elapsed_sim_sec(self, at_ns=None):
        if not self.started or self.start_sim_ns is None:
            return 0.0
        if at_ns is None:
            at_ns = self.now_sim_ns()
        return max(0.0, (at_ns - self.start_sim_ns) / 1e9)

    def elapsed_wall_sec(self):
        if not self.started or self.start_wall is None:
            return 0.0
        return max(0.0, time.monotonic() - self.start_wall)

    def event(self, name, detail="", at_sim_ns=None):
        self.event_writer.writerow([
            f"{self.elapsed_sim_sec(at_sim_ns):.3f}", name, detail
        ])
        self.event_file.flush()

    def goal_cb(self, msg):
        if self.finished:
            return
        if self.started:
            self.event(
                "EXTRA_FAR_GOAL_IGNORED",
                f"x={msg.pose.position.x:.3f}, y={msg.pose.position.y:.3f}",
            )
            self.get_logger().warning(
                "Second /far_goal_pose received during run; ignored."
            )
            return

        self.final_goal = msg
        self.start_sim_ns = self.now_sim_ns()
        self.start_wall = time.monotonic()
        self.started = True
        self.last_scan_wall = self.start_wall
        self.last_map_tf_wall = self.start_wall
        self.last_odom_tf_wall = self.start_wall

        gyaw = quat_to_yaw(msg.pose.orientation)
        self.event("BENCHMARK_START", self.case_id)
        self.event(
            "FINAL_GOAL",
            f"x={msg.pose.position.x:.3f}, "
            f"y={msg.pose.position.y:.3f}, yaw={gyaw:.4f}",
        )

        self.get_logger().info("=" * 56)
        self.get_logger().info(
            f"START {self.case_id} / {self.run_id}"
        )
        self.get_logger().info(
            f"Goal(map): x={msg.pose.position.x:.2f}, "
            f"y={msg.pose.position.y:.2f}, yaw={gyaw:.3f}"
        )
        self.get_logger().info("=" * 56)

    def scan_cb(self, msg):
        if not self.started or self.finished:
            return
        self.last_scan_wall = time.monotonic()
        # Count the message, not only the ones that happened to carry a
        # usable return, so scan_message_count stays a health figure.
        self.scan_count += 1

        valid = [
            r for r in msg.ranges
            if math.isfinite(r)
            and msg.range_min <= r <= msg.range_max
        ]
        if valid:
            self.latest_lidar_min = min(valid)
            self.run_min_lidar = min(
                self.run_min_lidar, self.latest_lidar_min
            )
        else:
            self.latest_lidar_min = math.nan

        clearance = self.footprint_clearance(msg)
        self.latest_clearance = clearance
        if not math.isfinite(clearance):
            return

        self.clearance_sum += clearance
        self.clearance_samples += 1
        self.clearance_values.append(clearance)

        if clearance <= 0.0:
            self.collision_suspected_count += 1
            if self.collision_suspected_count == 1:
                self.event(
                    "COLLISION_SUSPECTED",
                    "a laser return fell inside the robot footprint",
                )

        if clearance < self.run_min_clearance:
            self.run_min_clearance = clearance
            self.min_clearance_at_sec = round(self.elapsed_sim_sec(), 3)
            if self.last_map_pose is not None:
                x, y, yaw = self.last_map_pose
                self.min_clearance_pose = {
                    "x": round(x, 6),
                    "y": round(y, 6),
                    "yaw_rad": round(yaw, 6),
                }

    def lidar_to_base_link(self, frame_id):
        """Cache the LiDAR -> base_link transform (it is static)."""
        frame = str(frame_id).lstrip("/")
        if self.lidar_tf is not None and self.lidar_tf_frame == frame:
            return self.lidar_tf
        try:
            tf = self.tf_buffer.lookup_transform("base_link", frame, Time())
        except Exception:
            if not self.lidar_tf_warned:
                self.lidar_tf_warned = True
                self.event(
                    "LIDAR_TF_UNAVAILABLE",
                    f"frame={frame}; clearance will be recorded as nan",
                )
            return None
        self.lidar_tf_frame = frame
        self.lidar_tf = (
            float(tf.transform.translation.x),
            float(tf.transform.translation.y),
            quat_to_yaw(tf.transform.rotation),
        )
        self.event(
            "LIDAR_TF_RESOLVED",
            f"frame={frame}; x={self.lidar_tf[0]:.3f}; "
            f"y={self.lidar_tf[1]:.3f}; yaw={self.lidar_tf[2]:.4f}",
        )
        return self.lidar_tf

    def footprint_clearance(self, msg):
        """Smallest distance from the robot FOOTPRINT to any laser return.

        Returns 0.0 when a return lands inside the footprint, which is the
        LiDAR-based collision indicator.  Self-returns closer than
        SELF_RETURN_MIN_M are discarded exactly as the costmap discards them.
        """
        transform = self.lidar_to_base_link(msg.header.frame_id)
        if transform is None:
            return math.nan

        tx, ty, tyaw = transform
        cos_t = math.cos(tyaw)
        sin_t = math.sin(tyaw)
        lower = max(float(msg.range_min), self.SELF_RETURN_MIN_M)
        upper = float(msg.range_max)
        angle_min = float(msg.angle_min)
        angle_increment = float(msg.angle_increment)

        best = math.inf
        for index, raw_range in enumerate(msg.ranges):
            distance = float(raw_range)
            if not math.isfinite(distance) or distance < lower or distance > upper:
                continue

            angle = angle_min + index * angle_increment
            lx = distance * math.cos(angle)
            ly = distance * math.sin(angle)
            bx = tx + cos_t * lx - sin_t * ly
            by = ty + sin_t * lx + cos_t * ly

            dx = abs(bx) - self.FOOTPRINT_HALF_LENGTH_M
            dy = abs(by) - self.FOOTPRINT_HALF_WIDTH_M
            clearance = math.hypot(max(dx, 0.0), max(dy, 0.0))
            if clearance < best:
                best = clearance
                if best <= 0.0:
                    break

        return best if math.isfinite(best) else math.nan

    def odom_twist_cb(self, msg):
        if not self.started or self.finished:
            return
        self.odom_vx = float(msg.twist.twist.linear.x)
        self.odom_wz = float(msg.twist.twist.angular.z)
        self.odom_twist_count += 1

    def collision_monitor_cb(self, msg):
        if not self.started or self.finished:
            return
        self.collision_monitor_seen = True
        action_value = int(getattr(msg, "action_type", 0))
        name = COLLISION_ACTION_NAMES.get(action_value, f"ACTION_{action_value}")
        polygon = str(getattr(msg, "polygon_name", ""))
        if name != self.collision_monitor_action:
            self.collision_monitor_counts[name] = (
                self.collision_monitor_counts.get(name, 0) + 1
            )
            self.event(
                "COLLISION_MONITOR_STATE",
                f"action={name}; polygon={polygon}",
            )
        self.collision_monitor_action = name

    def raw_cmd_cb(self, msg):
        if not self.started or self.finished:
            return
        self.raw_vx = float(msg.twist.linear.x)
        self.raw_wz = float(msg.twist.angular.z)
        self.raw_cmd_count += 1

    def final_cmd_cb(self, msg):
        if not self.started or self.finished:
            return
        self.final_vx = float(msg.twist.linear.x)
        self.final_wz = float(msg.twist.angular.z)
        self.final_cmd_count += 1

    def plan_cb(self, msg):
        """Record only Nav2's canonical global /plan topic in its own frame."""
        if not self.started or self.finished or not msg.poses:
            return

        self.plan_message_count += 1
        plan_id = self.plan_message_count
        nav_goal_index = 0
        if self.latest_nav_goal_id is not None:
            nav_goal_index = self.nav_goal_indices.get(
                self.latest_nav_goal_id, 0
            )
        frame_id = str(msg.header.frame_id).lstrip("/")
        source_topic = "/plan"
        self.planned_path_topics.add(source_topic)
        elapsed = f"{self.elapsed_sim_sec():.3f}"

        for pose_index, pose_stamped in enumerate(msg.poses):
            pose = pose_stamped.pose
            self.plan_writer.writerow([
                elapsed,
                plan_id,
                nav_goal_index,
                frame_id,
                source_topic,
                pose_index,
                f"{float(pose.position.x):.6f}",
                f"{float(pose.position.y):.6f}",
                f"{quat_to_yaw(pose.orientation):.6f}",
            ])
            self.plan_pose_count += 1

        self.plan_file.flush()
        self.event(
            "GLOBAL_PLAN",
            f"plan_id={plan_id}; nav_goal_index={nav_goal_index}; "
            f"frame={frame_id}; poses={len(msg.poses)}",
        )

    def rosout_cb(self, msg):
        """
        Count actual local/global costmap clear requests.

        We intentionally count the costmap server's own ROS log message, not
        the BT node's intention to clear. This means the counter increments
        only when the request reached the costmap service callback.

        Historical /rosout messages are ignored because logging begins only
        after /far_goal_pose starts the benchmark.
        """
        if not self.started or self.finished:
            return

        message = str(msg.msg)
        lower = message.lower()

        if "received request to clear" not in lower:
            return

        # Deduplicate an identical rosout record just in case the transport
        # re-delivers it.
        key = (
            int(msg.stamp.sec),
            int(msg.stamp.nanosec),
            str(msg.name),
            message,
        )
        if key in self._seen_clear_logs:
            return
        self._seen_clear_logs.add(key)

        node_name = str(msg.name).lower()

        is_local = (
            "local_costmap" in lower
            or "local_costmap" in node_name
        )
        is_global = (
            "global_costmap" in lower
            or "global_costmap" in node_name
        )

        if is_local and not is_global:
            self.local_costmap_clear_count += 1
            self.event(
                "LOCAL_COSTMAP_CLEAR",
                f"count={self.local_costmap_clear_count}; "
                f"source={msg.name}; msg={message}",
            )
            return

        if is_global and not is_local:
            self.global_costmap_clear_count += 1
            self.event(
                "GLOBAL_COSTMAP_CLEAR",
                f"count={self.global_costmap_clear_count}; "
                f"source={msg.name}; msg={message}",
            )
            return

        # Fallback for unexpected naming. Preserve the event for diagnosis
        # without assigning it to the wrong counter.
        self.event(
            "COSTMAP_CLEAR_UNCLASSIFIED",
            f"source={msg.name}; msg={message}",
        )

    def add_behavior_status(self, name, topic):
        self.create_subscription(
            GoalStatusArray,
            topic,
            lambda msg, n=name: self.behavior_status_cb(n, msg),
            10,
        )

    def behavior_status_cb(self, name, msg):
        if not self.started or self.finished:
            return

        for s in msg.status_list:
            stamp_ns = stamp_to_ns(s.goal_info.stamp)
            if (
                self.start_sim_ns is not None
                and stamp_ns < self.start_sim_ns
            ):
                continue

            goal_id = uuid_hex(s.goal_info.goal_id)
            state = int(s.status)
            previous = self.behavior_states[name].get(goal_id)
            if previous == state:
                continue

            if previous is None and state != GoalStatus.STATUS_UNKNOWN:
                self.behavior_counts[name] += 1
                self.event(
                    f"{name.upper()}_START",
                    f"goal={goal_id[:8]} "
                    f"first_seen={STATUS_NAME.get(state, state)}",
                    at_sim_ns=stamp_ns,
                )

            self.behavior_states[name][goal_id] = state

            if state in TERMINAL:
                self.event(
                    f"{name.upper()}_{STATUS_NAME.get(state, state)}",
                    f"goal={goal_id[:8]}",
                )

    def nav_status_cb(self, msg):
        if not self.started or self.finished:
            return

        now_sim_ns = self.now_sim_ns()
        min_stamp_ns = self.start_sim_ns - 500_000_000
        max_stamp_ns = now_sim_ns + int(
            self.STATUS_FUTURE_TOLERANCE_SEC * 1e9
        )
        fresh = []
        for s in msg.status_list:
            stamp_ns = stamp_to_ns(s.goal_info.stamp)
            goal_id = uuid_hex(s.goal_info.goal_id)
            if stamp_ns < min_stamp_ns or stamp_ns > max_stamp_ns:
                if goal_id not in self._ignored_nav_status_ids:
                    self._ignored_nav_status_ids.add(goal_id)
                    self.event(
                        "STALE_NAV_STATUS_IGNORED",
                        f"goal={goal_id[:8]} stamp_ns={stamp_ns} "
                        f"valid=[{min_stamp_ns},{max_stamp_ns}]",
                    )
                continue

            state = int(s.status)
            fresh.append((stamp_ns, goal_id, state))

        if not fresh:
            return

        # Register every unseen action goal in timestamp order. Goal ids, not
        # timestamp comparisons alone, distinguish rapid goals created in the
        # same simulation clock tick.
        for stamp_ns, goal_id, state in sorted(fresh, key=lambda item: item[0]):
            if state == GoalStatus.STATUS_UNKNOWN:
                continue
            if goal_id in self.nav_goal_indices:
                continue

            if self.pending_terminal is not None:
                self.event(
                    "NAV_TERMINAL_SUPERSEDED",
                    f"old_goal={self.pending_terminal['goal_id'][:8]} "
                    f"new_goal={goal_id[:8]}",
                )
                self.pending_terminal = None

            self.nav_goal_count += 1
            self.nav_goal_indices[goal_id] = self.nav_goal_count
            self.latest_nav_goal_id = goal_id
            self.latest_nav_stamp_ns = max(self.latest_nav_stamp_ns, stamp_ns)
            if self.first_nav_sim_ns is None:
                self.first_nav_sim_ns = stamp_ns

            self.event(
                "NAV_GOAL_NEW",
                f"index={self.nav_goal_count} goal={goal_id[:8]} "
                f"status={STATUS_NAME.get(state, state)}",
                at_sim_ns=stamp_ns,
            )
            self.get_logger().info(
                f"NAV_GOAL_NEW index={self.nav_goal_count} "
                f"goal={goal_id[:8]} "
                f"status={STATUS_NAME.get(state, state)}"
            )

        latest_state = None
        for _stamp_ns, goal_id, state in fresh:
            previous = self.nav_states.get(goal_id)
            if previous != state:
                self.nav_states[goal_id] = state
                self.event(
                    "NAV_STATUS",
                    f"goal={goal_id[:8]} "
                    f"status={STATUS_NAME.get(state, state)}",
                )
            if (
                goal_id == self.latest_nav_goal_id
                and state == GoalStatus.STATUS_EXECUTING
            ):
                self.nav_executing_seen = True
            if goal_id == self.latest_nav_goal_id:
                latest_state = state

        if latest_state in TERMINAL:
            goal_id = self.latest_nav_goal_id
            if (
                self.pending_terminal is None
                or self.pending_terminal["goal_id"] != goal_id
                or self.pending_terminal["status"] != latest_state
            ):
                final_distance = self.final_xy_error_now()
                policy = navigation_terminal_policy(
                    STATUS_NAME.get(latest_state, str(latest_state)),
                    final_distance,
                    final_near_distance_m=self.FINAL_NEAR_DISTANCE_M,
                    final_grace_sec=self.TERMINAL_GRACE_SEC,
                    intermediate_wait_sec=self.INTERMEDIATE_TERMINAL_WAIT_SEC,
                )
                self.pending_terminal = {
                    "goal_id": goal_id,
                    "status": latest_state,
                    "is_intermediate": policy.is_intermediate,
                    "final_distance": final_distance,
                    "seen_wall": time.monotonic(),
                    "wait_sec": policy.wait_sec,
                    "timeout_result": policy.timeout_result,
                }
                self.get_logger().warning(
                    f"NAV_TERMINAL_PENDING goal={goal_id[:8]} "
                    f"status={STATUS_NAME.get(latest_state, latest_state)}; "
                    f"intermediate={policy.is_intermediate}; "
                    f"wait={policy.wait_sec:.1f}s"
                )

    def lookup_pose(self, parent_frame):
        try:
            tf = self.tf_buffer.lookup_transform(
                parent_frame, "base_link", Time()
            )
        except Exception:
            return None
        return (
            float(tf.transform.translation.x),
            float(tf.transform.translation.y),
            quat_to_yaw(tf.transform.rotation),
        )

    def final_xy_error_now(self):
        if self.final_goal is None or self.last_map_pose is None:
            return None
        x, y, _ = self.last_map_pose
        gx = float(self.final_goal.pose.position.x)
        gy = float(self.final_goal.pose.position.y)
        return math.hypot(x - gx, y - gy)

    def timer_cb(self):
        if self.finished or not self.started:
            return

        now_wall = time.monotonic()
        map_pose = self.lookup_pose("map")
        odom_pose = self.lookup_pose("odom")

        if map_pose is not None:
            self.last_map_pose = map_pose
            self.last_map_tf_wall = now_wall

        if odom_pose is not None:
            self.last_odom_pose = odom_pose
            self.last_odom_tf_wall = now_wall
            ox, oy, _ = odom_pose

            if self.prev_odom_xy is not None:
                step = math.hypot(
                    ox - self.prev_odom_xy[0],
                    oy - self.prev_odom_xy[1],
                )
                if step <= self.ODOM_MAX_STEP_M:
                    self.travel_distance += step
                else:
                    self.odom_jump_count += 1
                    self.event(
                        "ODOM_JUMP_IGNORED",
                        f"step={step:.3f}m",
                    )

            self.prev_odom_xy = (ox, oy)

        if map_pose is not None or odom_pose is not None:
            mx, my, myaw = (
                map_pose if map_pose is not None
                else (math.nan, math.nan, math.nan)
            )
            ox, oy, oyaw = (
                odom_pose if odom_pose is not None
                else (math.nan, math.nan, math.nan)
            )

            self.traj_writer.writerow([
                f"{self.elapsed_sim_sec():.3f}",
                f"{mx:.6f}", f"{my:.6f}", f"{myaw:.6f}",
                f"{ox:.6f}", f"{oy:.6f}", f"{oyaw:.6f}",
                f"{self.raw_vx:.6f}", f"{self.raw_wz:.6f}",
                f"{self.final_vx:.6f}", f"{self.final_wz:.6f}",
                (
                    f"{self.latest_lidar_min:.6f}"
                    if math.isfinite(self.latest_lidar_min)
                    else "nan"
                ),
                (
                    f"{self.latest_clearance:.6f}"
                    if math.isfinite(self.latest_clearance)
                    else "nan"
                ),
                (
                    f"{self.odom_vx:.6f}"
                    if math.isfinite(self.odom_vx) else "nan"
                ),
                (
                    f"{self.odom_wz:.6f}"
                    if math.isfinite(self.odom_wz) else "nan"
                ),
                self.collision_monitor_action,
            ])
            self.trajectory_sample_count += 1
            self.accumulate_motion_quality()
            if self.trajectory_sample_count % 10 == 0:
                self.traj_file.flush()

        if self.elapsed_wall_sec() >= self.HEALTH_STARTUP_GRACE_SEC:
            health_ages = {
                "scan": now_wall - self.last_scan_wall,
                "map_tf": now_wall - self.last_map_tf_wall,
                "odom_tf": now_wall - self.last_odom_tf_wall,
            }
            stale = {
                name: age
                for name, age in health_ages.items()
                if age >= self.DATA_STALE_WALL_SEC
            }
            if stale:
                detail = ", ".join(
                    f"{name}={age:.1f}s" for name, age in stale.items()
                )
                self.infra_failure_reason = (
                    "Runtime ROS data became stale: " + detail
                )
                self.event("RUNTIME_INFRA_FAILURE", self.infra_failure_reason)
                self.finish("INFRA_ERROR")
                return

        if self.elapsed_wall_sec() >= self.TIMEOUT_WALL_SEC:
            self.event(
                "BENCHMARK_TIMEOUT",
                f"wall_timeout={self.TIMEOUT_WALL_SEC:.1f}s",
            )
            self.finish("TIMEOUT")
            return

        if self.pending_terminal is not None:
            elapsed_pending = now_wall - self.pending_terminal["seen_wall"]

            if elapsed_pending >= self.pending_terminal["wait_sec"]:
                result = self.pending_terminal["timeout_result"]
                if self.pending_terminal["is_intermediate"]:
                    distance = self.final_xy_error_now()
                    distance_text = (
                        f"{distance:.3f}m" if distance is not None else "unknown"
                    )
                    self.event(
                        "INTERMEDIATE_TERMINAL_NO_REPLACEMENT",
                        f"goal={self.pending_terminal['goal_id'][:8]} "
                        f"status={result}; final_distance={distance_text}; "
                        f"waited={elapsed_pending:.1f}s",
                    )
                self.finish(result)

    def accumulate_motion_quality(self):
        """Integrate per-sample motion and intervention statistics.

        Everything here is derived from data the logger already receives;
        it exists so a report can quote safety and smoothness figures rather
        than only "the robot arrived".
        """
        now_sim = self.elapsed_sim_sec()
        previous = self.last_sample_sim_sec
        self.last_sample_sim_sec = now_sim
        dt = 0.0 if previous is None else max(0.0, now_sim - previous)

        # Time spent operating without margin, and under safety-layer control.
        if dt > 0.0:
            if (
                math.isfinite(self.latest_clearance)
                and self.latest_clearance < self.SAFETY_MARGIN_M
            ):
                self.time_below_margin_sec += dt
            if self.collision_monitor_action not in ("", "DO_NOTHING"):
                self.collision_monitor_intervention_sec += dt

        # How much speed the safety layer actually removed. Comparing the two
        # latest values for equality would count sampling skew between the
        # topics; a relative reduction is what "the monitor slowed us down"
        # actually means, and forced stops are skew-insensitive.
        if math.isfinite(self.raw_vx) and math.isfinite(self.final_vx):
            raw_speed = abs(self.raw_vx)
            final_speed = abs(self.final_vx)
            if raw_speed > self.CMD_MOVING_EPS:
                self.cmd_moving_samples += 1
                reduction = max(0.0, (raw_speed - final_speed) / raw_speed)
                self.cmd_reduction_sum += reduction
                self.cmd_reduction_max = max(self.cmd_reduction_max, reduction)
                if (
                    final_speed <= self.CMD_EPS
                    and abs(self.final_wz) <= self.CMD_EPS
                ):
                    self.cmd_stop_forced_samples += 1

        # Actual motion, from odometry rather than from the command.
        if math.isfinite(self.odom_vx) and math.isfinite(self.odom_wz):
            speed = abs(self.odom_vx)
            self.speed_sum += speed
            self.abs_yaw_rate_sum += abs(self.odom_wz)
            self.speed_samples += 1
            self.max_speed = max(self.max_speed, speed)

            if dt > 0.0:
                if (
                    speed <= self.STOPPED_SPEED_MPS
                    and abs(self.odom_wz) <= self.STOPPED_YAW_RATE_RADPS
                ):
                    self.stopped_time_sec += dt
                else:
                    self.moving_time_sec += dt

                if self.prev_odom_vx is not None:
                    linear_accel = (self.odom_vx - self.prev_odom_vx) / dt
                    angular_accel = (self.odom_wz - self.prev_odom_wz) / dt
                    self.accel_sq_sum += linear_accel * linear_accel
                    self.ang_accel_sq_sum += angular_accel * angular_accel
                    self.accel_samples += 1
                    # A sign flip in commanded travel direction is a hesitation
                    # the robot had to make; count it rather than average it away.
                    if (
                        self.prev_odom_vx * self.odom_vx < 0.0
                        and abs(self.prev_odom_vx) > self.STOPPED_SPEED_MPS
                        and abs(self.odom_vx) > self.STOPPED_SPEED_MPS
                    ):
                        self.direction_reversal_count += 1

            self.prev_odom_vx = self.odom_vx
            self.prev_odom_wz = self.odom_wz

        # Path curvature from position: re-anchor every SMOOTHNESS_STEP_M and
        # accumulate the change in travel direction. Divided by distance this
        # is a standard smoothness figure (rad per metre) and, unlike summing
        # yaw estimates, it does not accumulate localization jitter.
        if self.last_map_pose is not None:
            x, y, _ = self.last_map_pose
            if math.isfinite(x) and math.isfinite(y):
                if self.smoothness_anchor_xy is None:
                    self.smoothness_anchor_xy = (x, y)
                else:
                    ax, ay = self.smoothness_anchor_xy
                    if math.hypot(x - ax, y - ay) >= self.SMOOTHNESS_STEP_M:
                        heading = math.atan2(y - ay, x - ax)
                        if self.smoothness_prev_heading is not None:
                            self.heading_change_total += abs(
                                normalize_angle(
                                    heading - self.smoothness_prev_heading
                                )
                            )
                        self.smoothness_prev_heading = heading
                        self.smoothness_anchor_xy = (x, y)

    def classify_benchmark_result(
        self,
        nav2_result,
        final_xy_error,
        final_yaw_error,
    ):
        """
        Keep the raw Nav2 result and derive a separate benchmark verdict.

        PASS:
          Nav2 itself returned SUCCEEDED.

        NEAR_SUCCESS:
          Nav2 returned ABORTED, but the robot ended within 10 cm and 5 deg
          of the requested final pose.

        FAIL:
          Other ABORTED/CANCELED outcomes.

        TIMEOUT / INTERRUPTED:
          Preserved as their own benchmark result.
        """
        if nav2_result == "INFRA_ERROR":
            return "INFRA_ERROR", self.infra_failure_reason or "Runtime infrastructure failure"

        if nav2_result == "SUCCEEDED":
            return "PASS", "Nav2 returned SUCCEEDED"

        if nav2_result == "ABORTED":
            if (
                final_xy_error is not None
                and final_yaw_error is not None
                and final_xy_error <= self.NEAR_SUCCESS_XY_M
                and math.degrees(final_yaw_error)
                    <= self.NEAR_SUCCESS_YAW_DEG
            ):
                return (
                    "NEAR_SUCCESS",
                    f"Nav2 ABORTED, but final pose error is within "
                    f"{self.NEAR_SUCCESS_XY_M:.2f} m / "
                    f"{self.NEAR_SUCCESS_YAW_DEG:.1f} deg",
                )
            return "FAIL", "Nav2 returned ABORTED outside NEAR_SUCCESS limits"

        if nav2_result == "CANCELED":
            return "FAIL", "Nav2 goal was CANCELED"

        if nav2_result == "TIMEOUT":
            return "TIMEOUT", "Benchmark wall-clock timeout"

        if nav2_result == "INTERRUPTED":
            return "INTERRUPTED", "Benchmark logger interrupted"

        if nav2_result == "MISSION_STALLED":
            return (
                "FAIL",
                "An intermediate Nav2 goal succeeded, but Far Goal Manager "
                "did not send the next goal",
            )

        return "FAIL", f"Unhandled Nav2 result: {nav2_result}"

    @staticmethod
    def percentile(values, fraction):
        if not values:
            return None
        ordered = sorted(values)
        index = int(round(fraction * (len(ordered) - 1)))
        return ordered[min(max(index, 0), len(ordered) - 1)]

    def build_summary(self, result):
        end_sim_ns = (
            self.end_sim_ns
            if self.end_sim_ns is not None
            else self.now_sim_ns()
        )
        end_wall = (
            self.end_wall
            if self.end_wall is not None
            else time.monotonic()
        )

        duration_sim = (
            max(0.0, (end_sim_ns - self.start_sim_ns) / 1e9)
            if self.start_sim_ns is not None else None
        )
        duration_wall = (
            max(0.0, end_wall - self.start_wall)
            if self.start_wall is not None else None
        )
        goal_to_nav = (
            max(
                0.0,
                (self.first_nav_sim_ns - self.start_sim_ns) / 1e9,
            )
            if self.start_sim_ns is not None
            and self.first_nav_sim_ns is not None
            else None
        )

        final_xy_error = None
        final_yaw_error = None
        final_map_pose = None
        final_odom_pose = None
        requested_goal_map_pose = None

        if self.final_goal is not None:
            requested_goal_map_pose = {
                "x": round(float(self.final_goal.pose.position.x), 6),
                "y": round(float(self.final_goal.pose.position.y), 6),
                "yaw_rad": round(
                    quat_to_yaw(self.final_goal.pose.orientation), 6
                ),
                "frame_id": str(self.final_goal.header.frame_id),
            }

        if self.last_map_pose is not None:
            x, y, yaw = self.last_map_pose
            final_map_pose = {
                "x": round(x, 6),
                "y": round(y, 6),
                "yaw_rad": round(yaw, 6),
            }

            if self.final_goal is not None:
                gx = float(self.final_goal.pose.position.x)
                gy = float(self.final_goal.pose.position.y)
                gyaw = quat_to_yaw(
                    self.final_goal.pose.orientation
                )
                final_xy_error = math.hypot(x - gx, y - gy)
                final_yaw_error = abs(
                    normalize_angle(yaw - gyaw)
                )

        if self.last_odom_pose is not None:
            x, y, yaw = self.last_odom_pose
            final_odom_pose = {
                "x": round(x, 6),
                "y": round(y, 6),
                "yaw_rad": round(yaw, 6),
            }

        path_ratio = (
            self.travel_distance / self.direct_distance
            if self.direct_distance > 0.0 else None
        )
        path_efficiency = (
            self.direct_distance / self.travel_distance
            if self.travel_distance > 0.0 else None
        )

        benchmark_result, benchmark_result_reason = (
            self.classify_benchmark_result(
                result,
                final_xy_error,
                final_yaw_error,
            )
        )

        return {
            **suite_metadata(),
            "case_id": self.case_id,
            "scenario": self.scenario,
            "run_id": self.run_id,
            # Keep "result" for backward compatibility.
            "result": result,
            "nav2_result": result,
            "benchmark_result": benchmark_result,
            "benchmark_result_reason": benchmark_result_reason,

            "duration_sim_sec": (
                round(duration_sim, 3)
                if duration_sim is not None else None
            ),
            "duration_wall_sec": (
                round(duration_wall, 3)
                if duration_wall is not None else None
            ),
            "goal_to_first_nav_sec": (
                round(goal_to_nav, 3)
                if goal_to_nav is not None else None
            ),

            "direct_distance_m": round(self.direct_distance, 3),
            "travel_distance_odom_m": round(
                self.travel_distance, 3
            ),
            "path_ratio_travel_over_direct": (
                round(path_ratio, 4)
                if path_ratio is not None else None
            ),
            "path_efficiency_direct_over_travel": (
                round(path_efficiency, 4)
                if path_efficiency is not None else None
            ),

            "final_xy_error_m": (
                round(final_xy_error, 4)
                if final_xy_error is not None else None
            ),
            "final_yaw_error_deg": (
                round(math.degrees(final_yaw_error), 3)
                if final_yaw_error is not None else None
            ),
            "min_lidar_range_m": (
                round(self.run_min_lidar, 3)
                if math.isfinite(self.run_min_lidar)
                else None
            ),

            # --- Safety: clearance measured from the robot footprint -----
            "min_clearance_m": (
                round(self.run_min_clearance, 4)
                if math.isfinite(self.run_min_clearance) else None
            ),
            "mean_clearance_m": (
                round(self.clearance_sum / self.clearance_samples, 4)
                if self.clearance_samples else None
            ),
            "p05_clearance_m": (
                round(self.percentile(self.clearance_values, 0.05), 4)
                if self.clearance_values else None
            ),
            "clearance_sample_count": self.clearance_samples,
            "time_below_safety_margin_sec": round(
                self.time_below_margin_sec, 3
            ),
            "time_below_safety_margin_ratio": (
                round(self.time_below_margin_sec / duration_sim, 4)
                if duration_sim else None
            ),
            "collision_suspected_scan_count":
                self.collision_suspected_count,
            "collision_suspected": self.collision_suspected_count > 0,
            "min_clearance_at_sec": self.min_clearance_at_sec,
            "min_clearance_map_pose": self.min_clearance_pose,

            # --- Safety-layer intervention ------------------------------
            # Distinguishes "nav2_msgs is not installed on this PC" from
            # "the topic was never published".
            "collision_monitor_msg_type_available":
                CollisionMonitorState is not None,
            "collision_monitor_state_available":
                self.collision_monitor_seen,
            "collision_monitor_action_counts": dict(
                sorted(self.collision_monitor_counts.items())
            ),
            "collision_monitor_intervention_sec": round(
                self.collision_monitor_intervention_sec, 3
            ),
            "cmd_speed_reduction_mean": (
                round(self.cmd_reduction_sum / self.cmd_moving_samples, 4)
                if self.cmd_moving_samples else None
            ),
            "cmd_speed_reduction_max": round(self.cmd_reduction_max, 4),
            "cmd_forced_stop_ratio": (
                round(
                    self.cmd_stop_forced_samples / self.cmd_moving_samples, 4
                )
                if self.cmd_moving_samples else None
            ),
            "cmd_moving_sample_count": self.cmd_moving_samples,

            # --- Motion quality (from odometry, not from commands) ------
            "mean_speed_mps": (
                round(self.speed_sum / self.speed_samples, 4)
                if self.speed_samples else None
            ),
            "max_speed_mps": round(self.max_speed, 4),
            "mean_abs_yaw_rate_radps": (
                round(self.abs_yaw_rate_sum / self.speed_samples, 4)
                if self.speed_samples else None
            ),
            "moving_time_sec": round(self.moving_time_sec, 3),
            "stopped_time_sec": round(self.stopped_time_sec, 3),
            "stopped_time_ratio": (
                round(self.stopped_time_sec / duration_sim, 4)
                if duration_sim else None
            ),
            "linear_accel_rms_mps2": (
                round(math.sqrt(self.accel_sq_sum / self.accel_samples), 4)
                if self.accel_samples else None
            ),
            "angular_accel_rms_radps2": (
                round(math.sqrt(self.ang_accel_sq_sum / self.accel_samples), 4)
                if self.accel_samples else None
            ),
            "direction_reversal_count": self.direction_reversal_count,
            "heading_change_total_rad": round(self.heading_change_total, 4),
            "heading_change_per_meter_radpm": (
                round(self.heading_change_total / self.travel_distance, 4)
                if self.travel_distance > 0.0 else None
            ),
            "odom_twist_message_count": self.odom_twist_count,

            "navigate_to_pose_goal_count": self.nav_goal_count,
            "navigate_to_pose_executing_seen": self.nav_executing_seen,
            "spin_count": self.behavior_counts["spin"],
            "backup_count": self.behavior_counts["backup"],
            "wait_count": self.behavior_counts["wait"],
            "adaptive_escape_count":
                self.behavior_counts["adaptive_escape"],

            "behavior_recovery_count":
                sum(self.behavior_counts.values()),

            "local_costmap_clear_count":
                self.local_costmap_clear_count,
            "global_costmap_clear_count":
                self.global_costmap_clear_count,
            "costmap_clear_count":
                self.local_costmap_clear_count
                + self.global_costmap_clear_count,

            "total_recovery_event_count":
                sum(self.behavior_counts.values())
                + self.local_costmap_clear_count
                + self.global_costmap_clear_count,

            "scan_message_count": self.scan_count,
            "raw_cmd_message_count": self.raw_cmd_count,
            "final_cmd_message_count": self.final_cmd_count,
            "trajectory_sample_count":
                self.trajectory_sample_count,
            "odom_jump_ignored_count": self.odom_jump_count,
            "planned_path_message_count": self.plan_message_count,
            "planned_path_pose_count": self.plan_pose_count,
            "planned_path_topics": sorted(self.planned_path_topics),

            "requested_goal_map_pose": requested_goal_map_pose,
            "final_map_pose": final_map_pose,
            "final_odom_pose": final_odom_pose,

            "notes": {
                "travel_distance_source": "odom->base_link TF",
                "final_error_source":
                    "map->base_link TF vs /far_goal_pose",
                "min_lidar_range_is_clearance": False,
                "clearance_source":
                    "per-beam distance from the robot footprint rectangle "
                    "to each laser return, in base_link",
                "clearance_footprint_half_length_m":
                    self.FOOTPRINT_HALF_LENGTH_M,
                "clearance_footprint_half_width_m":
                    self.FOOTPRINT_HALF_WIDTH_M,
                "clearance_self_return_min_m": self.SELF_RETURN_MIN_M,
                "safety_margin_m": self.SAFETY_MARGIN_M,
                "collision_suspected_is_lidar_based": True,
                "collision_ground_truth_available": False,
                "motion_quality_source": "odom twist, sampled at 10 Hz sim",
                "smoothness_source":
                    "change in travel direction between position anchors "
                    f"{0.05:.2f} m apart (not summed yaw estimates)",
                "cmd_reduction_source":
                    "relative speed removed between /cmd_vel_raw and the "
                    "final platform command",
                "stopped_speed_threshold_mps": self.STOPPED_SPEED_MPS,
                "behavior_recovery_count_includes_costmap_clear":
                    False,
                "costmap_clear_count_source":
                    "/rosout costmap service callback log",
                "near_success_xy_threshold_m":
                    self.NEAR_SUCCESS_XY_M,
                "near_success_yaw_threshold_deg":
                    self.NEAR_SUCCESS_YAW_DEG,
                "near_success_collision_checked": False,
                "timeout_wall_sec": self.TIMEOUT_WALL_SEC,
                "terminal_grace_sec": self.TERMINAL_GRACE_SEC,
                "intermediate_terminal_wait_sec":
                    self.INTERMEDIATE_TERMINAL_WAIT_SEC,
                "runtime_data_stale_wall_sec": self.DATA_STALE_WALL_SEC,
            },
        }

    def finish(
        self,
        result,
        end_sim_ns=None,
        end_wall=None,
    ):
        if self.finished:
            return

        self.end_sim_ns = (
            end_sim_ns
            if end_sim_ns is not None
            else self.now_sim_ns()
        )
        self.end_wall = (
            end_wall
            if end_wall is not None
            else time.monotonic()
        )

        self.event(
            "BENCHMARK_END",
            result,
            at_sim_ns=self.end_sim_ns,
        )

        summary = self.build_summary(result)

        self.event(
            "BENCHMARK_CLASSIFICATION",
            f"nav2={summary['nav2_result']}; "
            f"benchmark={summary['benchmark_result']}; "
            f"reason={summary['benchmark_result_reason']}",
            at_sim_ns=self.end_sim_ns,
        )

        with self.summary_path.open(
            "w", encoding="utf-8"
        ) as f:
            yaml.safe_dump(
                summary,
                f,
                sort_keys=False,
                allow_unicode=True,
            )

        self.traj_file.flush()
        self.event_file.flush()
        self.plan_file.flush()
        self.finished = True

        self.get_logger().info("=" * 56)
        self.get_logger().info(f"NAV2 RESULT: {result}")
        self.get_logger().info(
            "BENCHMARK RESULT: "
            f"{summary['benchmark_result']}"
        )
        self.get_logger().info(
            f"Sim time: {summary['duration_sim_sec']} s"
        )
        self.get_logger().info(
            "Travel distance(odom): "
            f"{summary['travel_distance_odom_m']} m"
        )
        self.get_logger().info(
            f"Final XY error: {summary['final_xy_error_m']} m"
        )
        self.get_logger().info(
            "Final Yaw error: "
            f"{summary['final_yaw_error_deg']} deg"
        )
        self.get_logger().info(
            "Min LiDAR range: "
            f"{summary['min_lidar_range_m']} m"
        )
        self.get_logger().info(
            "Behaviors: "
            f"spin={summary['spin_count']}, "
            f"backup={summary['backup_count']}, "
            f"wait={summary['wait_count']}, "
            "adaptive_escape="
            f"{summary['adaptive_escape_count']}"
        )
        self.get_logger().info(
            "Costmap clears: "
            f"local={summary['local_costmap_clear_count']}, "
            f"global={summary['global_costmap_clear_count']}"
        )
        self.get_logger().info(
            "Global plans: "
            f"messages={summary['planned_path_message_count']}, "
            f"poses={summary['planned_path_pose_count']}"
        )
        self.get_logger().info(f"Saved: {self.result_dir}")
        self.get_logger().info("=" * 56)

    def interrupt(self):
        if not self.finished and self.started:
            self.finish("INTERRUPTED")

    def close_files(self):
        if self.files_closed:
            return
        self.traj_file.flush()
        self.event_file.flush()
        self.plan_file.flush()
        self.traj_file.close()
        self.event_file.close()
        self.plan_file.close()
        self.files_closed = True


def main():
    if len(sys.argv) < 2:
        print(
            "Usage:\n"
            "  python3 benchmark_logger.py S1_01 "
            "--ros-args -p use_sim_time:=true "
            "-r /tf:=/a200_0000/tf "
            "-r /tf_static:=/a200_0000/tf_static"
        )
        return

    case_id = sys.argv[1].upper()
    ros_args = sys.argv[2:]

    rclpy.init(args=ros_args)
    node = BenchmarkLogger(case_id)

    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.interrupt()
    finally:
        node.close_files()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()