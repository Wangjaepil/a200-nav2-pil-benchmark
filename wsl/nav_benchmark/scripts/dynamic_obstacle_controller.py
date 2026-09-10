#!/usr/bin/env python3
"""Simulation-time controller and ground-truth recorder for S5 obstacles.

The controller never chooses a synthetic path.  It waits until Nav2 publishes
an actual global ``/plan`` intersecting each obstacle's configured motion
segment, freezes that pre-stimulus plan, and only then applies the case trigger.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path as NavPath
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.time import Time
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener

from benchmark_common import (
    Pose2D,
    load_case_spec,
    quaternion_to_yaw,
    transform_world_goal_to_map,
)
from benchmark_dynamic import (
    DYNAMIC_SCHEMA_VERSION,
    DynamicObstacleSpec,
    PlanConflict,
    parse_dynamic_obstacles,
    point_polyline_distance,
    segment_polyline_conflict,
)


ROBOT_BOUNDING_RADIUS_M = math.hypot(0.494, 0.335)
CONTROL_PERIOD_SEC = 0.05
ODOM_STALE_WALL_SEC = 5.0
MOTION_TIMEOUT_FACTOR = 2.5
MOTION_TIMEOUT_PADDING_SEC = 4.0
MAX_VALID_ODOM_STEP_M = 0.50


@dataclass
class ObstacleRuntime:
    spec: DynamicObstacleSpec
    state: str = "WAITING_FOR_POSE"
    pose_world: Pose2D | None = None
    first_pose_world: Pose2D | None = None
    last_pose_wall: float | None = None
    initial_pose_error_m: float | None = None
    cmd_publisher: object | None = None
    touch_subscription: object | None = None
    frozen_plan: list[tuple[float, float]] = field(default_factory=list)
    frozen_plan_id: int | None = None
    conflict: PlanConflict | None = None
    command_started: bool = False
    motion_completed: bool = False
    touched: bool = False
    command_start_sim_sec: float | None = None
    command_end_sim_sec: float | None = None
    touch_sim_sec: float | None = None
    plan_count_at_start: int | None = None
    last_motion_xy: tuple[float, float] | None = None
    observed_travel_m: float = 0.0
    odom_jump_count: int = 0
    max_progress: float = 0.0
    min_distance_to_frozen_path_m: float = math.inf
    first_path_intersection_sim_sec: float | None = None
    min_robot_center_distance_m: float = math.inf
    first_temporal_interaction_sim_sec: float | None = None
    last_cmd_body_x: float = 0.0
    last_cmd_body_y: float = 0.0


class DynamicObstacleController(Node):
    def __init__(self, case_id: str, result_dir: Path, map_start: Pose2D):
        super().__init__("dynamic_obstacle_controller")
        case_spec = load_case_spec(case_id)
        specs = parse_dynamic_obstacles(case_spec.raw)
        if not specs:
            raise ValueError(f"{case_spec.case_id} has no dynamic_obstacles")

        self.case_spec = case_spec
        self.case_id = case_spec.case_id
        self.result_dir = Path(result_dir)
        self.result_dir.mkdir(parents=True, exist_ok=True)
        self.map_start = map_start
        self.runtimes = {spec.name: ObstacleRuntime(spec) for spec in specs}

        self.ready = False
        self.ready_wall = None
        self.fatal_reason = None
        self.goal_received = False
        self.goal_start_sim_ns = None
        self.goal_received_wall = None
        self.plan_message_count = 0
        self.latest_plan_id = None
        self.robot_map_pose: Pose2D | None = None
        self.collision_detected = False
        self.files_closed = False
        self.last_summary_wall = 0.0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.event_file = (self.result_dir / "dynamic_events.csv").open(
            "w", newline="", encoding="utf-8"
        )
        self.event_writer = csv.writer(self.event_file)
        self.event_writer.writerow(["time_sec", "obstacle", "event", "detail"])

        self.trajectory_file = (
            self.result_dir / "dynamic_obstacles.csv"
        ).open("w", newline="", encoding="utf-8")
        self.trajectory_writer = csv.writer(self.trajectory_file)
        self.trajectory_writer.writerow([
            "time_sec", "obstacle", "state",
            "world_x", "world_y", "world_yaw_rad",
            "map_x", "map_y", "map_yaw_rad",
            "cmd_body_vx", "cmd_body_vy",
            "progress", "distance_to_frozen_path_m",
            "robot_center_distance_m", "estimated_shape_clearance_m",
            "touched",
        ])

        self.status_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.status_publisher = self.create_publisher(
            String, "/benchmark/dynamic_status", self.status_qos
        )

        self.create_subscription(PoseStamped, "/far_goal_pose", self.goal_cb, 10)
        self.create_subscription(NavPath, "/plan", self.plan_cb, 10)

        for runtime in self.runtimes.values():
            prefix = runtime.spec.topic_prefix
            runtime.cmd_publisher = self.create_publisher(
                Twist, f"{prefix}/cmd_vel", 10
            )
            self.create_subscription(
                Odometry,
                f"{prefix}/odometry",
                lambda msg, name=runtime.spec.name: self.odom_cb(name, msg),
                qos_profile_sensor_data,
            )
            runtime.touch_subscription = self.create_subscription(
                Bool,
                f"{prefix}/touched",
                lambda msg, name=runtime.spec.name: self.touch_cb(name, msg),
                10,
            )

        self.timer = self.create_timer(CONTROL_PERIOD_SEC, self.timer_cb)
        self.event("", "CONTROLLER_START", f"case={self.case_id}")
        self.write_summary()
        print(
            f"DYNAMIC_CONTROLLER_WAITING case={self.case_id} "
            f"obstacles={len(self.runtimes)}",
            flush=True,
        )

    def now_sim_ns(self) -> int:
        return self.get_clock().now().nanoseconds

    def elapsed_sim_sec(self) -> float:
        if self.goal_start_sim_ns is None:
            return 0.0
        return max(0.0, (self.now_sim_ns() - self.goal_start_sim_ns) / 1e9)

    def event(self, obstacle: str, name: str, detail: str = "") -> None:
        self.event_writer.writerow([
            f"{self.elapsed_sim_sec():.3f}", obstacle, name, detail
        ])
        self.event_file.flush()
        rendered = f"[{name}]"
        if obstacle:
            rendered += f" {obstacle}"
        if detail:
            rendered += f" {detail}"
        print(rendered, flush=True)

    def world_to_map(self, pose: Pose2D) -> Pose2D:
        return transform_world_goal_to_map(
            self.case_spec.robot, pose, self.map_start
        )

    def goal_cb(self, _msg: PoseStamped) -> None:
        if self.goal_received:
            return
        self.goal_received = True
        self.goal_start_sim_ns = self.now_sim_ns()
        self.goal_received_wall = time.monotonic()
        self.event("", "GOAL_RECEIVED", "waiting for matching Nav2 plans")
        self.publish_status()
        self.write_summary()

    def plan_cb(self, msg: NavPath) -> None:
        if not self.ready or not self.goal_received or len(msg.poses) < 2:
            return
        frame_id = str(msg.header.frame_id).lstrip("/")
        if frame_id != "map":
            self.event("", "PLAN_IGNORED", f"frame={frame_id!r}")
            return

        points = [
            (float(item.pose.position.x), float(item.pose.position.y))
            for item in msg.poses
        ]
        self.plan_message_count += 1
        plan_id = self.plan_message_count
        self.latest_plan_id = plan_id

        for runtime in self.runtimes.values():
            if runtime.frozen_plan or runtime.command_started:
                continue
            start_map = self.world_to_map(runtime.spec.pose)
            end_map = self.world_to_map(runtime.spec.end)
            conflict = segment_polyline_conflict(
                (start_map.x, start_map.y),
                (end_map.x, end_map.y),
                points,
            )
            if conflict is None:
                continue
            if conflict.distance_m > runtime.spec.path_intersection_tolerance_m:
                continue

            runtime.frozen_plan = points
            runtime.frozen_plan_id = plan_id
            runtime.conflict = conflict
            runtime.state = "ARMED"
            self.event(
                runtime.spec.name,
                "PLAN_INTERSECTION_ARMED",
                f"plan_id={plan_id}; min_distance={conflict.distance_m:.3f}; "
                f"plan_point=({conflict.plan_point[0]:.3f},"
                f"{conflict.plan_point[1]:.3f})",
            )

        self.publish_status()
        self.write_summary()

    def odom_cb(self, name: str, msg: Odometry) -> None:
        runtime = self.runtimes[name]
        pose = Pose2D(
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y),
            quaternion_to_yaw(msg.pose.pose.orientation),
        )
        runtime.pose_world = pose
        runtime.last_pose_wall = time.monotonic()

        if runtime.first_pose_world is None:
            runtime.first_pose_world = pose
            runtime.initial_pose_error_m = math.hypot(
                pose.x - runtime.spec.pose.x,
                pose.y - runtime.spec.pose.y,
            )
            runtime.state = "WAITING_FOR_PLAN"
            self.event(
                name,
                "INITIAL_POSE",
                f"x={pose.x:.3f}; y={pose.y:.3f}; "
                f"error={runtime.initial_pose_error_m:.3f}",
            )

        # Freeze travelled-distance evidence at the first confirmed contact.
        # Counting post-contact odometry jitter could otherwise make a barely
        # moving obstacle look like a valid dynamic stimulus.
        if runtime.command_started and runtime.state == "MOVING":
            current_xy = (pose.x, pose.y)
            if runtime.last_motion_xy is not None:
                step = math.hypot(
                    current_xy[0] - runtime.last_motion_xy[0],
                    current_xy[1] - runtime.last_motion_xy[1],
                )
                if step <= MAX_VALID_ODOM_STEP_M:
                    runtime.observed_travel_m += step
                else:
                    runtime.odom_jump_count += 1
                    self.event(name, "ODOM_JUMP_IGNORED", f"step={step:.3f}")
            runtime.last_motion_xy = current_xy

        if runtime.command_started:
            start = runtime.spec.pose
            end = runtime.spec.end
            dx = end.x - start.x
            dy = end.y - start.y
            denom = dx * dx + dy * dy
            if denom > 0.0:
                progress = (
                    (pose.x - start.x) * dx + (pose.y - start.y) * dy
                ) / denom
                runtime.max_progress = max(runtime.max_progress, progress)

            if runtime.frozen_plan:
                map_pose = self.world_to_map(pose)
                path_distance = point_polyline_distance(
                    (map_pose.x, map_pose.y), runtime.frozen_plan
                )
                runtime.min_distance_to_frozen_path_m = min(
                    runtime.min_distance_to_frozen_path_m, path_distance
                )
                if (
                    runtime.first_path_intersection_sim_sec is None
                    and path_distance
                    <= runtime.spec.path_intersection_tolerance_m
                ):
                    runtime.first_path_intersection_sim_sec = self.elapsed_sim_sec()
                    self.event(
                        name,
                        "PATH_INTERSECTION_OBSERVED",
                        f"distance={path_distance:.3f}",
                    )

    def touch_cb(self, name: str, msg: Bool) -> None:
        if not bool(msg.data):
            return
        runtime = self.runtimes[name]
        if runtime.touched:
            return
        runtime.touched = True
        runtime.touch_sim_sec = self.elapsed_sim_sec()
        runtime.state = "COLLIDED"
        runtime.command_end_sim_sec = runtime.touch_sim_sec
        self.collision_detected = True
        self.publish_zero(runtime)
        self.event(
            name,
            "ROBOT_CONTACT_GROUND_TRUTH",
            "TouchPlugin target=a200_0000",
        )
        self.publish_status()
        self.write_summary()

    def lookup_robot_pose(self) -> Pose2D | None:
        try:
            transform = self.tf_buffer.lookup_transform(
                "map", "base_link", Time()
            )
        except Exception:
            return None
        return Pose2D(
            float(transform.transform.translation.x),
            float(transform.transform.translation.y),
            quaternion_to_yaw(transform.transform.rotation),
        )

    def bridges_ready(self, runtime: ObstacleRuntime) -> bool:
        cmd_ready = runtime.cmd_publisher.get_subscription_count() > 0
        touch_ready = runtime.touch_subscription.get_publisher_count() > 0
        return runtime.pose_world is not None and cmd_ready and touch_ready

    def maybe_become_ready(self) -> None:
        if self.ready:
            return
        if not all(self.bridges_ready(runtime) for runtime in self.runtimes.values()):
            return

        bad = []
        for runtime in self.runtimes.values():
            error = runtime.initial_pose_error_m
            if error is None or error > runtime.spec.initial_pose_tolerance_m:
                bad.append(
                    f"{runtime.spec.name}: initial error={error}; "
                    f"limit={runtime.spec.initial_pose_tolerance_m:.3f}"
                )
        if bad:
            self.fatal_reason = "Dynamic obstacle initial pose mismatch: " + "; ".join(bad)
            self.event("", "FATAL", self.fatal_reason)
            self.write_summary()
            return

        self.ready = True
        self.ready_wall = time.monotonic()
        self.event("", "CONTROLLER_READY", "odom/cmd/touch bridges verified")
        self.publish_status()
        self.write_summary()
        print("DYNAMIC_CONTROLLER_READY", flush=True)

    def trigger_satisfied(self, runtime: ObstacleRuntime) -> bool:
        trigger = runtime.spec.trigger
        if trigger.kind == "elapsed_after_goal":
            return self.elapsed_sim_sec() >= float(trigger.delay_sec)
        if self.robot_map_pose is None or runtime.conflict is None:
            return False
        distance = math.hypot(
            self.robot_map_pose.x - runtime.conflict.plan_point[0],
            self.robot_map_pose.y - runtime.conflict.plan_point[1],
        )
        return distance <= float(trigger.distance_m)

    def start_motion(self, runtime: ObstacleRuntime) -> None:
        runtime.command_started = True
        runtime.state = "MOVING"
        runtime.command_start_sim_sec = self.elapsed_sim_sec()
        runtime.plan_count_at_start = self.plan_message_count
        if runtime.pose_world is not None:
            runtime.last_motion_xy = (runtime.pose_world.x, runtime.pose_world.y)
        self.event(
            runtime.spec.name,
            "MOTION_START",
            f"speed={runtime.spec.speed_mps:.3f}; "
            f"frozen_plan_id={runtime.frozen_plan_id}",
        )

    def publish_zero(self, runtime: ObstacleRuntime) -> None:
        runtime.last_cmd_body_x = 0.0
        runtime.last_cmd_body_y = 0.0
        runtime.cmd_publisher.publish(Twist())

    def publish_motion(self, runtime: ObstacleRuntime) -> None:
        pose = runtime.pose_world
        if pose is None:
            self.publish_zero(runtime)
            return

        spec = runtime.spec
        distance_to_end = math.hypot(spec.end.x - pose.x, spec.end.y - pose.y)
        if distance_to_end <= spec.end_tolerance_m or runtime.max_progress >= 0.995:
            runtime.motion_completed = True
            runtime.state = "COMPLETED"
            runtime.command_end_sim_sec = self.elapsed_sim_sec()
            self.publish_zero(runtime)
            self.event(
                spec.name,
                "MOTION_COMPLETE",
                f"travel={runtime.observed_travel_m:.3f}; "
                f"progress={runtime.max_progress:.3f}",
            )
            self.publish_status()
            self.write_summary()
            return

        expected_sec = spec.segment_length_m / spec.speed_mps
        if (
            runtime.command_start_sim_sec is not None
            and self.elapsed_sim_sec() - runtime.command_start_sim_sec
            > expected_sec * MOTION_TIMEOUT_FACTOR + MOTION_TIMEOUT_PADDING_SEC
        ):
            runtime.state = "MOTION_FAILED"
            runtime.command_end_sim_sec = self.elapsed_sim_sec()
            self.publish_zero(runtime)
            self.event(
                spec.name,
                "MOTION_TIMEOUT",
                f"travel={runtime.observed_travel_m:.3f}; "
                f"progress={runtime.max_progress:.3f}",
            )
            self.publish_status()
            self.write_summary()
            return

        world_dx = spec.end.x - spec.pose.x
        world_dy = spec.end.y - spec.pose.y
        length = spec.segment_length_m
        world_vx = spec.speed_mps * world_dx / length
        world_vy = spec.speed_mps * world_dy / length

        # VelocityControl consumes body-fixed velocity. Rotate the desired
        # world-frame segment velocity into the obstacle's observed body frame.
        cosine = math.cos(pose.yaw)
        sine = math.sin(pose.yaw)
        body_vx = cosine * world_vx + sine * world_vy
        body_vy = -sine * world_vx + cosine * world_vy

        message = Twist()
        message.linear.x = body_vx
        message.linear.y = body_vy
        runtime.last_cmd_body_x = body_vx
        runtime.last_cmd_body_y = body_vy
        runtime.cmd_publisher.publish(message)

    def update_interaction_metric(self, runtime: ObstacleRuntime) -> None:
        if not runtime.command_started or runtime.pose_world is None:
            return
        if runtime.state not in {"MOVING", "COLLIDED"}:
            validation = runtime.spec.raw.get("validation") or {}
            if not bool(validation.get("allow_interaction_after_motion", False)):
                return
        if self.robot_map_pose is None:
            return

        obstacle_map = self.world_to_map(runtime.pose_world)
        center_distance = math.hypot(
            obstacle_map.x - self.robot_map_pose.x,
            obstacle_map.y - self.robot_map_pose.y,
        )
        runtime.min_robot_center_distance_m = min(
            runtime.min_robot_center_distance_m, center_distance
        )
        if (
            runtime.first_temporal_interaction_sim_sec is None
            and center_distance
            <= runtime.spec.max_robot_interaction_distance_m
        ):
            runtime.first_temporal_interaction_sim_sec = self.elapsed_sim_sec()
            self.event(
                runtime.spec.name,
                "TEMPORAL_INTERACTION_OBSERVED",
                f"robot_center_distance={center_distance:.3f}",
            )

    def record_trajectory(self, runtime: ObstacleRuntime) -> None:
        pose = runtime.pose_world
        if pose is None:
            return
        map_pose = self.world_to_map(pose)
        estimated_clearance = (
            runtime.min_robot_center_distance_m
            - ROBOT_BOUNDING_RADIUS_M
            - runtime.spec.bounding_radius_m
        )
        self.trajectory_writer.writerow([
            f"{self.elapsed_sim_sec():.3f}",
            runtime.spec.name,
            runtime.state,
            f"{pose.x:.6f}", f"{pose.y:.6f}", f"{pose.yaw:.6f}",
            f"{map_pose.x:.6f}", f"{map_pose.y:.6f}", f"{map_pose.yaw:.6f}",
            f"{runtime.last_cmd_body_x:.6f}",
            f"{runtime.last_cmd_body_y:.6f}",
            f"{runtime.max_progress:.6f}",
            (
                f"{runtime.min_distance_to_frozen_path_m:.6f}"
                if math.isfinite(runtime.min_distance_to_frozen_path_m) else "nan"
            ),
            (
                f"{runtime.min_robot_center_distance_m:.6f}"
                if math.isfinite(runtime.min_robot_center_distance_m) else "nan"
            ),
            (
                f"{estimated_clearance:.6f}"
                if math.isfinite(estimated_clearance) else "nan"
            ),
            runtime.touched,
        ])

    def timer_cb(self) -> None:
        if self.fatal_reason is not None:
            return

        self.maybe_become_ready()
        if self.fatal_reason is not None:
            return

        self.robot_map_pose = self.lookup_robot_pose()
        now_wall = time.monotonic()

        for runtime in self.runtimes.values():
            if self.ready and runtime.last_pose_wall is not None:
                age = now_wall - runtime.last_pose_wall
                if age >= ODOM_STALE_WALL_SEC:
                    self.fatal_reason = (
                        f"Dynamic obstacle odometry stale: {runtime.spec.name} "
                        f"age={age:.1f}s"
                    )
                    self.event(runtime.spec.name, "FATAL", self.fatal_reason)
                    self.publish_zero(runtime)
                    self.write_summary()
                    return

            if not self.ready or not self.goal_received:
                self.publish_zero(runtime)
            elif runtime.state == "ARMED" and self.trigger_satisfied(runtime):
                self.start_motion(runtime)
                self.publish_motion(runtime)
            elif runtime.state == "MOVING":
                self.publish_motion(runtime)
            else:
                self.publish_zero(runtime)

            self.update_interaction_metric(runtime)
            self.record_trajectory(runtime)

        self.trajectory_file.flush()
        self.publish_status()
        if now_wall - self.last_summary_wall >= 1.0:
            self.write_summary()

    def obstacle_summary(self, runtime: ObstacleRuntime) -> dict:
        path_seen = runtime.frozen_plan_id is not None
        path_crossed = (
            math.isfinite(runtime.min_distance_to_frozen_path_m)
            and runtime.min_distance_to_frozen_path_m
            <= runtime.spec.path_intersection_tolerance_m
        )
        temporal = (
            math.isfinite(runtime.min_robot_center_distance_m)
            and runtime.min_robot_center_distance_m
            <= runtime.spec.max_robot_interaction_distance_m
        )
        completed_motion = (
            runtime.motion_completed
            and runtime.observed_travel_m >= runtime.spec.min_travel_m
        )
        collision_motion = (
            runtime.touched
            and runtime.observed_travel_m
            >= runtime.spec.min_motion_before_collision_valid_m
        )
        motion_valid = completed_motion or collision_motion
        initial_ok = (
            runtime.initial_pose_error_m is not None
            and runtime.initial_pose_error_m <= runtime.spec.initial_pose_tolerance_m
        )

        reasons = []
        if not initial_ok:
            reasons.append("initial_pose_not_verified")
        if not path_seen:
            reasons.append("no_intersecting_nav2_plan")
        if not runtime.command_started:
            reasons.append("motion_never_started")
        if not motion_valid:
            reasons.append("required_motion_not_observed")
        if not path_crossed:
            reasons.append("frozen_plan_not_crossed")
        if not temporal:
            reasons.append("no_temporal_robot_interaction")

        valid = not reasons
        center_distance = (
            runtime.min_robot_center_distance_m
            if math.isfinite(runtime.min_robot_center_distance_m) else None
        )
        path_distance = (
            runtime.min_distance_to_frozen_path_m
            if math.isfinite(runtime.min_distance_to_frozen_path_m) else None
        )
        estimated_clearance = None
        if center_distance is not None:
            estimated_clearance = (
                center_distance
                - ROBOT_BOUNDING_RADIUS_M
                - runtime.spec.bounding_radius_m
            )

        return {
            "name": runtime.spec.name,
            "required": runtime.spec.required,
            "state": runtime.state,
            "shape": runtime.spec.shape,
            "speed_mps": runtime.spec.speed_mps,
            "configured_segment_length_m": round(runtime.spec.segment_length_m, 4),
            "initial_pose_error_m": (
                round(runtime.initial_pose_error_m, 4)
                if runtime.initial_pose_error_m is not None else None
            ),
            "frozen_plan_id": runtime.frozen_plan_id,
            "frozen_plan_pose_count": len(runtime.frozen_plan),
            "configured_plan_intersection_tolerance_m":
                runtime.spec.path_intersection_tolerance_m,
            "configured_max_robot_interaction_distance_m":
                runtime.spec.max_robot_interaction_distance_m,
            "configured_min_travel_m": runtime.spec.min_travel_m,
            "command_started": runtime.command_started,
            "motion_completed": runtime.motion_completed,
            "command_start_sim_sec": self._rounded(runtime.command_start_sim_sec),
            "command_end_sim_sec": self._rounded(runtime.command_end_sim_sec),
            "observed_travel_m": round(runtime.observed_travel_m, 4),
            "max_segment_progress": round(runtime.max_progress, 4),
            "odom_jump_ignored_count": runtime.odom_jump_count,
            "min_distance_to_frozen_path_m": self._rounded(path_distance, 4),
            "path_intersection_observed": path_crossed,
            "first_path_intersection_sim_sec":
                self._rounded(runtime.first_path_intersection_sim_sec),
            "min_robot_center_distance_m": self._rounded(center_distance, 4),
            "estimated_min_shape_clearance_m":
                self._rounded(estimated_clearance, 4),
            "temporal_interaction_observed": temporal,
            "first_temporal_interaction_sim_sec":
                self._rounded(runtime.first_temporal_interaction_sim_sec),
            "collision_ground_truth": runtime.touched,
            "collision_sim_sec": self._rounded(runtime.touch_sim_sec),
            "valid": valid,
            "invalid_reasons": reasons,
        }

    @staticmethod
    def _rounded(value, digits: int = 3):
        return round(value, digits) if value is not None else None

    def build_summary(self) -> dict:
        obstacle_summaries = [
            self.obstacle_summary(runtime) for runtime in self.runtimes.values()
        ]
        required = [item for item in obstacle_summaries if item["required"]]
        invalid_reasons = []
        if not self.ready:
            invalid_reasons.append("dynamic_controller_not_ready")
        if not self.goal_received:
            invalid_reasons.append("far_goal_not_received")
        if self.fatal_reason:
            invalid_reasons.append(self.fatal_reason)
        for item in required:
            if not item["valid"]:
                invalid_reasons.append(
                    f"{item['name']}:" + ",".join(item["invalid_reasons"])
                )
        stimulus_valid = bool(required) and not invalid_reasons

        return {
            "dynamic_schema_version": DYNAMIC_SCHEMA_VERSION,
            "case_id": self.case_id,
            "controller_ready": self.ready,
            "fatal_reason": self.fatal_reason,
            "goal_received": self.goal_received,
            "plan_message_count_seen": self.plan_message_count,
            "dynamic_obstacle_count": len(obstacle_summaries),
            "required_dynamic_obstacle_count": len(required),
            "stimulus_valid": stimulus_valid,
            "collision_ground_truth": any(
                item["collision_ground_truth"] for item in obstacle_summaries
            ),
            "invalid_reasons": invalid_reasons,
            "world_to_map_anchor": {
                "world_start": {
                    "x": self.case_spec.robot.x,
                    "y": self.case_spec.robot.y,
                    "yaw_rad": self.case_spec.robot.yaw,
                },
                "map_start": {
                    "x": self.map_start.x,
                    "y": self.map_start.y,
                    "yaw_rad": self.map_start.yaw,
                },
            },
            "obstacles": obstacle_summaries,
            "notes": {
                "plan_validation":
                    "observed obstacle pose vs frozen pre-stimulus Nav2 /plan",
                "temporal_validation":
                    "robot and obstacle center distance while the obstacle is moving",
                "collision_source":
                    "Gazebo Contact system + TouchPlugin target a200_0000",
                "estimated_min_shape_clearance_is_conservative": True,
            },
        }

    def publish_status(self) -> None:
        message = String()
        message.data = json.dumps(
            self.build_summary(), separators=(",", ":"), sort_keys=True
        )
        self.status_publisher.publish(message)

    def write_summary(self) -> None:
        target = self.result_dir / "dynamic_summary.yaml"
        temporary = target.with_suffix(".yaml.tmp")
        with temporary.open("w", encoding="utf-8") as handle:
            yaml.safe_dump(
                self.build_summary(),
                handle,
                sort_keys=False,
                allow_unicode=True,
            )
        os.replace(temporary, target)
        self.last_summary_wall = time.monotonic()

    def shutdown(self) -> None:
        for runtime in self.runtimes.values():
            self.publish_zero(runtime)
        self.publish_status()
        self.write_summary()
        self.close_files()

    def close_files(self) -> None:
        if self.files_closed:
            return
        self.event_file.flush()
        self.trajectory_file.flush()
        self.event_file.close()
        self.trajectory_file.close()
        self.files_closed = True


def parse_args(argv: list[str]) -> tuple[argparse.Namespace, list[str]]:
    parser = argparse.ArgumentParser(
        description="Control and validate S5 Gazebo dynamic obstacles."
    )
    parser.add_argument("case_id")
    parser.add_argument("--result-dir", required=True, type=Path)
    parser.add_argument("--map-start-x", required=True, type=float)
    parser.add_argument("--map-start-y", required=True, type=float)
    parser.add_argument("--map-start-yaw", required=True, type=float)
    return parser.parse_known_args(argv)


def main() -> int:
    args, ros_args = parse_args(sys.argv[1:])
    rclpy.init(args=ros_args)
    node = DynamicObstacleController(
        args.case_id.upper(),
        args.result_dir,
        Pose2D(args.map_start_x, args.map_start_y, args.map_start_yaw),
    )
    exit_code = 0
    try:
        while rclpy.ok() and node.fatal_reason is None:
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.fatal_reason is not None:
            exit_code = 2
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
