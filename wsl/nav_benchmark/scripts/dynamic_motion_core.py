#!/usr/bin/env python3
"""Deterministic, ROS-free state machine for S5 obstacle motion.

The ROS node supplies observations and publishes the returned commands.  All
state transitions live here so they can be regression-tested without Gazebo,
Zenoh, ROS discovery, or simulation time.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum

from benchmark_common import CaseSpec, Pose2D, transform_world_goal_to_map
from benchmark_dynamic import (
    DYNAMIC_SCHEMA_VERSION,
    DynamicObstacleSpec,
    PlanConflict,
    body_velocity_towards,
    point_polyline_distance,
    segment_polyline_conflict,
    segment_progress,
)


ROBOT_BOUNDING_RADIUS_M = math.hypot(0.494, 0.335)
MAX_ODOM_STEP_M = 0.75
ODOM_STALE_TIMEOUT_SEC = 2.0
MOTION_TIMEOUT_FACTOR = 2.0
MOTION_TIMEOUT_PADDING_SEC = 3.0


class MotionPhase(str, Enum):
    WAITING_FOR_ODOMETRY = "WAITING_FOR_ODOMETRY"
    WAITING_FOR_READY = "WAITING_FOR_READY"
    WAITING_FOR_GOAL = "WAITING_FOR_GOAL"
    WAITING_FOR_PLAN = "WAITING_FOR_PLAN"
    ARMED = "ARMED"
    MOVING = "MOVING"
    COMPLETED = "COMPLETED"
    COLLIDED = "COLLIDED"
    STOPPED_EARLY = "STOPPED_EARLY"
    FAILED = "FAILED"


@dataclass(frozen=True)
class VelocityCommand:
    body_x_mps: float = 0.0
    body_y_mps: float = 0.0


@dataclass(frozen=True)
class MotionEvent:
    elapsed_sim_sec: float
    obstacle: str
    name: str
    detail: str = ""


@dataclass
class ObstacleRuntime:
    spec: DynamicObstacleSpec
    phase: MotionPhase = MotionPhase.WAITING_FOR_ODOMETRY
    pose_world: Pose2D | None = None
    first_pose_world: Pose2D | None = None
    last_odom_wall_sec: float | None = None
    initial_pose_error_m: float | None = None
    frozen_plan: tuple[tuple[float, float], ...] = ()
    frozen_plan_id: int | None = None
    conflict: PlanConflict | None = None
    command_started: bool = False
    motion_completed: bool = False
    touched: bool = False
    command_start_sim_sec: float | None = None
    command_end_sim_sec: float | None = None
    collision_sim_sec: float | None = None
    last_motion_xy: tuple[float, float] | None = None
    observed_travel_m: float = 0.0
    odom_jump_count: int = 0
    max_segment_progress: float = 0.0
    min_distance_to_frozen_path_m: float = math.inf
    first_path_intersection_sim_sec: float | None = None
    min_robot_center_distance_m: float = math.inf
    first_temporal_interaction_sim_sec: float | None = None
    failure_reason: str | None = None
    last_command: VelocityCommand = field(default_factory=VelocityCommand)


class DynamicMotionCoordinator:
    """Own the complete state of one S5 case's moving obstacles."""

    def __init__(
        self,
        case_spec: CaseSpec,
        obstacle_specs: list[DynamicObstacleSpec],
        map_start: Pose2D,
    ) -> None:
        if not obstacle_specs:
            raise ValueError(f"{case_spec.case_id} has no dynamic obstacles")

        self.case_spec = case_spec
        self.map_start = map_start
        self.runtimes = {
            spec.name: ObstacleRuntime(spec) for spec in obstacle_specs
        }
        self.controller_ready = False
        self.goal_received = False
        self.goal_start_sim_sec: float | None = None
        self.plan_message_count = 0
        self.robot_map_pose: Pose2D | None = None
        self.fatal_reason: str | None = None
        self.stop_requested = False
        self._events: list[MotionEvent] = []
        self._emit(0.0, "", "CONTROLLER_CREATED", case_spec.case_id)

    @staticmethod
    def _zero_commands(
        runtimes: dict[str, ObstacleRuntime],
    ) -> dict[str, VelocityCommand]:
        return {name: VelocityCommand() for name in runtimes}

    def elapsed(self, sim_sec: float) -> float:
        if self.goal_start_sim_sec is None:
            return 0.0
        return max(0.0, sim_sec - self.goal_start_sim_sec)

    def _emit(
        self,
        sim_sec: float,
        obstacle: str,
        name: str,
        detail: str = "",
    ) -> None:
        self._events.append(MotionEvent(
            elapsed_sim_sec=self.elapsed(sim_sec),
            obstacle=obstacle,
            name=name,
            detail=detail,
        ))

    def drain_events(self) -> list[MotionEvent]:
        events = self._events
        self._events = []
        return events

    def world_to_map(self, pose: Pose2D) -> Pose2D:
        return transform_world_goal_to_map(
            self.case_spec.robot,
            pose,
            self.map_start,
        )

    def observe_obstacle(
        self,
        name: str,
        pose_world: Pose2D,
        *,
        sim_sec: float,
        wall_sec: float,
    ) -> None:
        runtime = self.runtimes[name]
        runtime.pose_world = pose_world
        runtime.last_odom_wall_sec = wall_sec

        if runtime.first_pose_world is None:
            runtime.first_pose_world = pose_world
            runtime.initial_pose_error_m = math.hypot(
                pose_world.x - runtime.spec.start_world.x,
                pose_world.y - runtime.spec.start_world.y,
            )
            runtime.phase = MotionPhase.WAITING_FOR_READY
            self._emit(
                sim_sec,
                name,
                "INITIAL_POSE_RECEIVED",
                f"error_m={runtime.initial_pose_error_m:.4f}",
            )

        if runtime.phase == MotionPhase.MOVING:
            current_xy = (pose_world.x, pose_world.y)
            if runtime.last_motion_xy is not None:
                step = math.hypot(
                    current_xy[0] - runtime.last_motion_xy[0],
                    current_xy[1] - runtime.last_motion_xy[1],
                )
                if step <= MAX_ODOM_STEP_M:
                    runtime.observed_travel_m += step
                else:
                    runtime.odom_jump_count += 1
                    self._emit(
                        sim_sec,
                        name,
                        "ODOMETRY_JUMP_IGNORED",
                        f"step_m={step:.4f}",
                    )
            runtime.last_motion_xy = current_xy

        if runtime.command_started:
            runtime.max_segment_progress = max(
                runtime.max_segment_progress,
                segment_progress(runtime.spec, pose_world),
            )
            if runtime.frozen_plan:
                map_pose = self.world_to_map(pose_world)
                path_distance = point_polyline_distance(
                    (map_pose.x, map_pose.y), runtime.frozen_plan
                )
                runtime.min_distance_to_frozen_path_m = min(
                    runtime.min_distance_to_frozen_path_m,
                    path_distance,
                )
                if (
                    runtime.first_path_intersection_sim_sec is None
                    and path_distance
                    <= runtime.spec.validation.path_intersection_tolerance_m
                ):
                    runtime.first_path_intersection_sim_sec = self.elapsed(sim_sec)
                    self._emit(
                        sim_sec,
                        name,
                        "FROZEN_PATH_CROSSED",
                        f"separation_m={path_distance:.4f}",
                    )

    def activate(self, *, sim_sec: float) -> bool:
        """Accept transport readiness after every initial pose was observed."""
        if self.controller_ready:
            return True

        problems = []
        for runtime in self.runtimes.values():
            error = runtime.initial_pose_error_m
            tolerance = runtime.spec.validation.initial_pose_tolerance_m
            if error is None:
                problems.append(f"{runtime.spec.name}: no odometry")
            elif runtime.touched:
                problems.append(
                    f"{runtime.spec.name}: robot contact before readiness"
                )
            elif error > tolerance:
                problems.append(
                    f"{runtime.spec.name}: initial error {error:.3f} m "
                    f"> {tolerance:.3f} m"
                )

        if problems:
            self.fail(
                "Dynamic obstacle readiness failed: " + "; ".join(problems),
                sim_sec=sim_sec,
            )
            return False

        self.controller_ready = True
        next_phase = (
            MotionPhase.WAITING_FOR_PLAN
            if self.goal_received
            else MotionPhase.WAITING_FOR_GOAL
        )
        for runtime in self.runtimes.values():
            if runtime.phase not in {
                MotionPhase.COLLIDED,
                MotionPhase.FAILED,
            }:
                runtime.phase = next_phase
        self._emit(sim_sec, "", "CONTROLLER_READY", "all bridges verified")
        return True

    def receive_goal(self, *, sim_sec: float) -> bool:
        """Latch the first final benchmark goal; ignore duplicate publishers."""
        if self.goal_received:
            return False
        self.goal_received = True
        self.goal_start_sim_sec = sim_sec
        if self.controller_ready:
            for runtime in self.runtimes.values():
                if runtime.phase == MotionPhase.WAITING_FOR_GOAL:
                    runtime.phase = MotionPhase.WAITING_FOR_PLAN
        self._emit(sim_sec, "", "GOAL_RECEIVED", "waiting for intersecting /plan")
        return True

    def receive_plan(
        self,
        points_map: list[tuple[float, float]],
        *,
        frame_id: str,
        sim_sec: float,
    ) -> int:
        if not self.controller_ready or not self.goal_received:
            return 0
        if frame_id.lstrip("/") != "map" or len(points_map) < 2:
            return 0

        self.plan_message_count += 1
        plan_id = self.plan_message_count
        armed_count = 0

        for runtime in self.runtimes.values():
            if runtime.phase != MotionPhase.WAITING_FOR_PLAN:
                continue

            start_map = self.world_to_map(runtime.spec.start_world)
            end_map = self.world_to_map(runtime.spec.end_world)
            conflict = segment_polyline_conflict(
                (start_map.x, start_map.y),
                (end_map.x, end_map.y),
                points_map,
            )
            if (
                conflict is None
                or conflict.separation_m
                > runtime.spec.validation.path_intersection_tolerance_m
            ):
                continue

            runtime.frozen_plan = tuple(points_map)
            runtime.frozen_plan_id = plan_id
            runtime.conflict = conflict
            runtime.phase = MotionPhase.ARMED
            armed_count += 1
            self._emit(
                sim_sec,
                runtime.spec.name,
                "PLAN_CONFLICT_ARMED",
                f"plan_id={plan_id}; separation_m={conflict.separation_m:.4f}",
            )

        return armed_count

    def update_robot_pose(self, pose_map: Pose2D | None) -> None:
        self.robot_map_pose = pose_map

    def observe_contact(self, name: str, *, sim_sec: float) -> bool:
        runtime = self.runtimes[name]
        if runtime.touched:
            return False
        runtime.touched = True
        runtime.collision_sim_sec = self.elapsed(sim_sec)
        runtime.command_end_sim_sec = runtime.collision_sim_sec
        runtime.phase = MotionPhase.COLLIDED
        runtime.last_command = VelocityCommand()
        self._emit(
            sim_sec,
            name,
            "ROBOT_CONTACT",
            "Gazebo TouchPlugin target=a200_0000",
        )
        return True

    def fail(self, reason: str, *, sim_sec: float) -> None:
        if self.fatal_reason is not None:
            return
        self.fatal_reason = reason
        for runtime in self.runtimes.values():
            if runtime.phase not in {
                MotionPhase.COMPLETED,
                MotionPhase.COLLIDED,
            }:
                runtime.phase = MotionPhase.FAILED
                runtime.failure_reason = reason
            runtime.last_command = VelocityCommand()
        self._emit(sim_sec, "", "CONTROLLER_FATAL", reason)

    def request_stop(self, *, sim_sec: float) -> None:
        if self.stop_requested:
            return
        self.stop_requested = True
        for runtime in self.runtimes.values():
            if runtime.phase == MotionPhase.MOVING:
                runtime.phase = MotionPhase.STOPPED_EARLY
                runtime.command_end_sim_sec = self.elapsed(sim_sec)
            runtime.last_command = VelocityCommand()
        self._emit(sim_sec, "", "CONTROLLER_STOP", "zero command requested")

    def _trigger_satisfied(self, runtime: ObstacleRuntime, sim_sec: float) -> bool:
        trigger = runtime.spec.trigger
        if trigger.mode == "elapsed_after_goal":
            assert trigger.delay_sec is not None
            return self.elapsed(sim_sec) >= trigger.delay_sec

        if self.robot_map_pose is None or runtime.conflict is None:
            return False
        assert trigger.robot_distance_m is not None
        return math.hypot(
            self.robot_map_pose.x - runtime.conflict.plan_point[0],
            self.robot_map_pose.y - runtime.conflict.plan_point[1],
        ) <= trigger.robot_distance_m

    def _start_motion(self, runtime: ObstacleRuntime, sim_sec: float) -> None:
        runtime.phase = MotionPhase.MOVING
        runtime.command_started = True
        runtime.command_start_sim_sec = self.elapsed(sim_sec)
        if runtime.pose_world is not None:
            runtime.last_motion_xy = (
                runtime.pose_world.x,
                runtime.pose_world.y,
            )
        self._emit(
            sim_sec,
            runtime.spec.name,
            "MOTION_STARTED",
            f"speed_mps={runtime.spec.speed_mps:.3f}; "
            f"plan_id={runtime.frozen_plan_id}",
        )

    def _motion_command(
        self,
        runtime: ObstacleRuntime,
        sim_sec: float,
    ) -> VelocityCommand:
        pose = runtime.pose_world
        if pose is None:
            self.fail(
                f"Missing odometry while moving {runtime.spec.name}",
                sim_sec=sim_sec,
            )
            return VelocityCommand()

        remaining = math.hypot(
            runtime.spec.end_world.x - pose.x,
            runtime.spec.end_world.y - pose.y,
        )
        if remaining <= runtime.spec.validation.end_tolerance_m:
            runtime.phase = MotionPhase.COMPLETED
            runtime.motion_completed = True
            runtime.command_end_sim_sec = self.elapsed(sim_sec)
            self._emit(
                sim_sec,
                runtime.spec.name,
                "MOTION_COMPLETED",
                f"travel_m={runtime.observed_travel_m:.4f}",
            )
            return VelocityCommand()

        assert runtime.command_start_sim_sec is not None
        expected_duration = (
            runtime.spec.segment_length_m / runtime.spec.speed_mps
        )
        motion_elapsed = self.elapsed(sim_sec) - runtime.command_start_sim_sec
        timeout = (
            expected_duration * MOTION_TIMEOUT_FACTOR
            + MOTION_TIMEOUT_PADDING_SEC
        )
        if motion_elapsed > timeout:
            self.fail(
                f"Motion timeout for {runtime.spec.name}: "
                f"elapsed={motion_elapsed:.2f}s limit={timeout:.2f}s",
                sim_sec=sim_sec,
            )
            return VelocityCommand()

        # Keep the configured cruise speed, but taper it close to the endpoint
        # so odometry latency cannot produce a large overshoot.
        speed = min(runtime.spec.speed_mps, max(0.05, 2.0 * remaining))
        body_x, body_y = body_velocity_towards(
            pose,
            runtime.spec.end_world,
            speed,
        )
        return VelocityCommand(body_x, body_y)

    def _update_interaction(self, runtime: ObstacleRuntime, sim_sec: float) -> None:
        if not runtime.command_started or runtime.pose_world is None:
            return
        if (
            runtime.phase != MotionPhase.MOVING
            and not runtime.touched
            and not runtime.spec.validation.allow_interaction_after_motion
        ):
            return
        if self.robot_map_pose is None:
            return

        obstacle_map = self.world_to_map(runtime.pose_world)
        distance = math.hypot(
            obstacle_map.x - self.robot_map_pose.x,
            obstacle_map.y - self.robot_map_pose.y,
        )
        runtime.min_robot_center_distance_m = min(
            runtime.min_robot_center_distance_m,
            distance,
        )
        if (
            runtime.first_temporal_interaction_sim_sec is None
            and distance
            <= runtime.spec.validation.max_robot_interaction_distance_m
        ):
            runtime.first_temporal_interaction_sim_sec = self.elapsed(sim_sec)
            self._emit(
                sim_sec,
                runtime.spec.name,
                "TEMPORAL_INTERACTION",
                f"center_distance_m={distance:.4f}",
            )

    def tick(
        self,
        *,
        sim_sec: float,
        wall_sec: float,
    ) -> dict[str, VelocityCommand]:
        if self.stop_requested or self.fatal_reason is not None:
            return self._zero_commands(self.runtimes)

        if self.controller_ready:
            for runtime in self.runtimes.values():
                if runtime.last_odom_wall_sec is None:
                    self.fail(
                        f"Odometry missing for {runtime.spec.name}",
                        sim_sec=sim_sec,
                    )
                    return self._zero_commands(self.runtimes)
                age = wall_sec - runtime.last_odom_wall_sec
                if age > ODOM_STALE_TIMEOUT_SEC:
                    self.fail(
                        f"Odometry stale for {runtime.spec.name}: "
                        f"age={age:.2f}s",
                        sim_sec=sim_sec,
                    )
                    return self._zero_commands(self.runtimes)

        commands: dict[str, VelocityCommand] = {}
        for name, runtime in self.runtimes.items():
            if (
                self.controller_ready
                and self.goal_received
                and runtime.phase == MotionPhase.ARMED
                and self._trigger_satisfied(runtime, sim_sec)
            ):
                self._start_motion(runtime, sim_sec)

            if runtime.phase == MotionPhase.MOVING:
                command = self._motion_command(runtime, sim_sec)
            else:
                command = VelocityCommand()

            runtime.last_command = command
            commands[name] = command
            self._update_interaction(runtime, sim_sec)

        if self.fatal_reason is not None:
            return self._zero_commands(self.runtimes)
        return commands

    @staticmethod
    def _finite_or_none(value: float, digits: int = 4) -> float | None:
        return round(value, digits) if math.isfinite(value) else None

    def obstacle_summary(self, runtime: ObstacleRuntime) -> dict:
        validation = runtime.spec.validation
        initial_ok = (
            runtime.initial_pose_error_m is not None
            and runtime.initial_pose_error_m <= validation.initial_pose_tolerance_m
        )
        path_seen = runtime.frozen_plan_id is not None
        path_crossed = (
            math.isfinite(runtime.min_distance_to_frozen_path_m)
            and runtime.min_distance_to_frozen_path_m
            <= validation.path_intersection_tolerance_m
        )
        temporal_interaction = (
            math.isfinite(runtime.min_robot_center_distance_m)
            and runtime.min_robot_center_distance_m
            <= validation.max_robot_interaction_distance_m
        )
        completed_motion = (
            runtime.motion_completed
            and runtime.observed_travel_m >= validation.min_travel_m
        )
        collision_motion = (
            runtime.touched
            and runtime.observed_travel_m
            >= validation.min_motion_before_collision_valid_m
        )

        reasons = []
        if not initial_ok:
            reasons.append("initial_pose_not_verified")
        if not path_seen:
            reasons.append("no_intersecting_nav2_plan")
        if not runtime.command_started:
            reasons.append("motion_never_started")
        if not (completed_motion or collision_motion):
            reasons.append("required_motion_not_observed")
        # A physical robot contact can stop the obstacle before its centre
        # reaches the path centreline.  In that case TouchPlugin plus the
        # minimum observed motion is stronger evidence than centreline
        # crossing and must not invalidate the stimulus.
        if not path_crossed and not collision_motion:
            reasons.append("frozen_plan_not_crossed")
        if not temporal_interaction:
            reasons.append("no_temporal_robot_interaction")

        center_distance = self._finite_or_none(
            runtime.min_robot_center_distance_m
        )
        estimated_clearance = (
            center_distance
            - ROBOT_BOUNDING_RADIUS_M
            - runtime.spec.bounding_radius_m
            if center_distance is not None
            else None
        )

        return {
            "name": runtime.spec.name,
            "required": runtime.spec.required,
            "state": runtime.phase.value,
            "shape": runtime.spec.shape,
            "speed_mps": runtime.spec.speed_mps,
            "configured_segment_length_m": round(
                runtime.spec.segment_length_m, 4
            ),
            "configured_plan_intersection_tolerance_m": round(
                validation.path_intersection_tolerance_m, 4
            ),
            "configured_max_robot_interaction_distance_m": round(
                validation.max_robot_interaction_distance_m, 4
            ),
            "configured_min_travel_m": round(validation.min_travel_m, 4),
            "configured_min_motion_before_collision_valid_m": round(
                validation.min_motion_before_collision_valid_m, 4
            ),
            "configured_end_tolerance_m": round(
                validation.end_tolerance_m, 4
            ),
            "configured_initial_pose_tolerance_m": round(
                validation.initial_pose_tolerance_m, 4
            ),
            "trigger_type": runtime.spec.trigger.mode,
            "trigger_distance_m": runtime.spec.trigger.robot_distance_m,
            "trigger_delay_sec": runtime.spec.trigger.delay_sec,
            "initial_pose_error_m": (
                round(runtime.initial_pose_error_m, 4)
                if runtime.initial_pose_error_m is not None else None
            ),
            "frozen_plan_id": runtime.frozen_plan_id,
            "frozen_plan_pose_count": len(runtime.frozen_plan),
            "command_started": runtime.command_started,
            "motion_completed": runtime.motion_completed,
            "command_start_sim_sec": runtime.command_start_sim_sec,
            "command_end_sim_sec": runtime.command_end_sim_sec,
            "observed_travel_m": round(runtime.observed_travel_m, 4),
            "max_segment_progress": round(runtime.max_segment_progress, 4),
            "odom_jump_ignored_count": runtime.odom_jump_count,
            "min_distance_to_frozen_path_m": self._finite_or_none(
                runtime.min_distance_to_frozen_path_m
            ),
            "path_intersection_observed": path_crossed,
            "first_path_intersection_sim_sec":
                runtime.first_path_intersection_sim_sec,
            "min_robot_center_distance_m": center_distance,
            "estimated_min_shape_clearance_m": (
                round(estimated_clearance, 4)
                if estimated_clearance is not None else None
            ),
            "temporal_interaction_observed": temporal_interaction,
            "first_temporal_interaction_sim_sec":
                runtime.first_temporal_interaction_sim_sec,
            "collision_ground_truth": runtime.touched,
            "collision_sim_sec": runtime.collision_sim_sec,
            "valid": not reasons,
            "invalid_reasons": reasons,
            "failure_reason": runtime.failure_reason,
        }

    def build_summary(self) -> dict:
        obstacles = [
            self.obstacle_summary(runtime)
            for runtime in self.runtimes.values()
        ]
        required = [item for item in obstacles if item["required"]]
        invalid_reasons: list[str] = []
        if not self.controller_ready:
            invalid_reasons.append("dynamic_controller_not_ready")
        if not self.goal_received:
            invalid_reasons.append("far_goal_not_received")
        if self.fatal_reason is not None:
            invalid_reasons.append(self.fatal_reason)
        for item in required:
            if not item["valid"]:
                invalid_reasons.append(
                    f"{item['name']}:" + ",".join(item["invalid_reasons"])
                )

        return {
            "dynamic_schema_version": DYNAMIC_SCHEMA_VERSION,
            "case_id": self.case_spec.case_id,
            "controller_ready": self.controller_ready,
            "fatal_reason": self.fatal_reason,
            "goal_received": self.goal_received,
            "plan_message_count_seen": self.plan_message_count,
            "dynamic_obstacle_count": len(obstacles),
            "required_dynamic_obstacle_count": len(required),
            "stimulus_valid": bool(required) and not invalid_reasons,
            "collision_ground_truth": any(
                item["collision_ground_truth"] for item in obstacles
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
            "obstacles": obstacles,
        }
