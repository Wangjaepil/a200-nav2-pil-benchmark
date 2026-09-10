#!/usr/bin/env python3
"""ROS-independent schema and geometry helpers for S5 dynamic obstacles."""

from __future__ import annotations

import math
import re
from dataclasses import dataclass
from typing import Mapping, Sequence

from benchmark_common import Pose2D, require_finite


DYNAMIC_SCHEMA_VERSION = 1
DYNAMIC_NAME_RE = re.compile(r"^[A-Za-z][A-Za-z0-9_]*$")


@dataclass(frozen=True)
class DynamicTrigger:
    kind: str
    distance_m: float | None = None
    delay_sec: float | None = None


@dataclass(frozen=True)
class DynamicObstacleSpec:
    name: str
    shape: str
    pose: Pose2D
    end: Pose2D
    speed_mps: float
    mass_kg: float
    trigger: DynamicTrigger
    required: bool
    path_intersection_tolerance_m: float
    max_robot_interaction_distance_m: float
    min_travel_m: float
    min_motion_before_collision_valid_m: float
    end_tolerance_m: float
    initial_pose_tolerance_m: float
    raw: dict

    @property
    def segment_length_m(self) -> float:
        return math.hypot(self.end.x - self.pose.x, self.end.y - self.pose.y)

    @property
    def topic_prefix(self) -> str:
        return f"/benchmark/dynamic/{self.name}"

    @property
    def bounding_radius_m(self) -> float:
        if self.shape == "cylinder":
            return float(self.raw["radius"])
        sx, sy, _ = (float(value) for value in self.raw["size"])
        return 0.5 * math.hypot(sx, sy)


@dataclass(frozen=True)
class PlanConflict:
    distance_m: float
    motion_point: tuple[float, float]
    plan_point: tuple[float, float]
    plan_segment_index: int


def _positive_float(label: str, value, *, allow_zero: bool = False) -> float:
    try:
        parsed = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{label} must be a number") from exc
    require_finite(label, parsed)
    if parsed < 0.0 or (not allow_zero and parsed <= 0.0):
        relation = "non-negative" if allow_zero else "positive"
        raise ValueError(f"{label} must be {relation}, got {parsed}")
    return parsed


def _pose(label: str, value: Mapping, *, default_yaw: float = 0.0) -> Pose2D:
    if not isinstance(value, Mapping):
        raise ValueError(f"{label} must be a mapping")
    try:
        pose = Pose2D(
            float(value["x"]),
            float(value["y"]),
            float(value.get("yaw", default_yaw)),
        )
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError(f"Invalid {label}: {exc}") from exc
    require_finite(label, pose.x, pose.y, pose.yaw)
    return pose


def _shape_fields(index: int, raw: Mapping) -> str:
    shape = str(raw.get("shape", "")).strip().lower()
    label = f"dynamic_obstacles[{index}]"
    if shape == "cylinder":
        _positive_float(f"{label}.radius", raw.get("radius"))
        _positive_float(f"{label}.height", raw.get("height"))
        return shape
    if shape == "box":
        size = raw.get("size")
        if not isinstance(size, Sequence) or isinstance(size, (str, bytes)):
            raise ValueError(f"{label}.size must contain [x, y, z]")
        if len(size) != 3:
            raise ValueError(f"{label}.size must contain exactly 3 values")
        for axis, value in zip("xyz", size):
            _positive_float(f"{label}.size.{axis}", value)
        return shape
    raise ValueError(
        f"{label}.shape must be 'box' or 'cylinder'; dynamic polygon "
        "collisions are intentionally unsupported"
    )


def parse_dynamic_obstacles(case: Mapping) -> list[DynamicObstacleSpec]:
    """Validate and normalize a case's ``dynamic_obstacles`` list."""
    raw_items = case.get("dynamic_obstacles", [])
    if raw_items is None:
        return []
    if not isinstance(raw_items, list):
        raise ValueError("dynamic_obstacles must be a list")

    parsed: list[DynamicObstacleSpec] = []
    names: set[str] = set()
    for index, item in enumerate(raw_items):
        label = f"dynamic_obstacles[{index}]"
        if not isinstance(item, Mapping):
            raise ValueError(f"{label} must be a mapping")

        raw = dict(item)
        name = str(raw.get("name", "")).strip()
        if not DYNAMIC_NAME_RE.fullmatch(name):
            raise ValueError(
                f"{label}.name must match {DYNAMIC_NAME_RE.pattern!r}: {name!r}"
            )
        if name in names:
            raise ValueError(f"Duplicate dynamic obstacle name: {name}")
        names.add(name)

        shape = _shape_fields(index, raw)
        start = _pose(f"{label}.pose", raw.get("pose") or {})
        motion = raw.get("motion")
        if not isinstance(motion, Mapping):
            raise ValueError(f"{label}.motion must be a mapping")
        end = _pose(f"{label}.motion.end", motion.get("end") or {})
        length = math.hypot(end.x - start.x, end.y - start.y)
        if length < 0.50:
            raise ValueError(
                f"{label} motion segment is too short ({length:.3f} m); "
                "use at least 0.50 m"
            )

        speed = _positive_float(f"{label}.motion.speed_mps", motion.get("speed_mps"))
        trigger_raw = motion.get("trigger") or {}
        if not isinstance(trigger_raw, Mapping):
            raise ValueError(f"{label}.motion.trigger must be a mapping")
        trigger_kind = str(trigger_raw.get("type", "robot_distance")).strip().lower()
        if trigger_kind == "robot_distance":
            trigger = DynamicTrigger(
                trigger_kind,
                distance_m=_positive_float(
                    f"{label}.motion.trigger.distance_m",
                    trigger_raw.get("distance_m", 5.0),
                ),
            )
        elif trigger_kind == "elapsed_after_goal":
            trigger = DynamicTrigger(
                trigger_kind,
                delay_sec=_positive_float(
                    f"{label}.motion.trigger.delay_sec",
                    trigger_raw.get("delay_sec", 3.0),
                    allow_zero=True,
                ),
            )
        else:
            raise ValueError(
                f"{label}.motion.trigger.type is unsupported: {trigger_kind!r}"
            )

        validation = raw.get("validation") or {}
        if not isinstance(validation, Mapping):
            raise ValueError(f"{label}.validation must be a mapping")

        min_travel_default = max(0.50, 0.80 * length)
        parsed.append(DynamicObstacleSpec(
            name=name,
            shape=shape,
            pose=start,
            end=end,
            speed_mps=speed,
            mass_kg=_positive_float(f"{label}.mass", raw.get("mass", 40.0)),
            trigger=trigger,
            required=bool(raw.get("required", True)),
            path_intersection_tolerance_m=_positive_float(
                f"{label}.validation.path_intersection_tolerance_m",
                validation.get("path_intersection_tolerance_m", 0.30),
            ),
            max_robot_interaction_distance_m=_positive_float(
                f"{label}.validation.max_robot_interaction_distance_m",
                validation.get("max_robot_interaction_distance_m", 3.50),
            ),
            min_travel_m=_positive_float(
                f"{label}.validation.min_travel_m",
                validation.get("min_travel_m", min_travel_default),
            ),
            min_motion_before_collision_valid_m=_positive_float(
                f"{label}.validation.min_motion_before_collision_valid_m",
                validation.get("min_motion_before_collision_valid_m", 0.50),
            ),
            end_tolerance_m=_positive_float(
                f"{label}.validation.end_tolerance_m",
                validation.get("end_tolerance_m", 0.15),
            ),
            initial_pose_tolerance_m=_positive_float(
                f"{label}.validation.initial_pose_tolerance_m",
                validation.get("initial_pose_tolerance_m", 0.35),
            ),
            raw=raw,
        ))

    return parsed


def dynamic_bridge_arguments(case: Mapping) -> list[str]:
    """Return case-specific parameter_bridge arguments."""
    arguments: list[str] = []
    for obstacle in parse_dynamic_obstacles(case):
        prefix = obstacle.topic_prefix
        arguments.extend([
            f"{prefix}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
            f"{prefix}/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            f"{prefix}/touched@std_msgs/msg/Bool[gz.msgs.Boolean",
        ])
    return arguments


def closest_point_on_segment(
    point: tuple[float, float],
    start: tuple[float, float],
    end: tuple[float, float],
) -> tuple[tuple[float, float], float]:
    sx, sy = start
    ex, ey = end
    px, py = point
    dx = ex - sx
    dy = ey - sy
    denom = dx * dx + dy * dy
    if denom <= 1e-18:
        closest = start
    else:
        ratio = ((px - sx) * dx + (py - sy) * dy) / denom
        ratio = min(1.0, max(0.0, ratio))
        closest = (sx + ratio * dx, sy + ratio * dy)
    distance = math.hypot(px - closest[0], py - closest[1])
    return closest, distance


def _cross(a: tuple[float, float], b: tuple[float, float]) -> float:
    return a[0] * b[1] - a[1] * b[0]


def _segment_intersection(
    a: tuple[float, float],
    b: tuple[float, float],
    c: tuple[float, float],
    d: tuple[float, float],
) -> tuple[float, float] | None:
    r = (b[0] - a[0], b[1] - a[1])
    s = (d[0] - c[0], d[1] - c[1])
    denominator = _cross(r, s)
    c_minus_a = (c[0] - a[0], c[1] - a[1])
    if abs(denominator) <= 1e-12:
        return None
    t = _cross(c_minus_a, s) / denominator
    u = _cross(c_minus_a, r) / denominator
    if -1e-12 <= t <= 1.0 + 1e-12 and -1e-12 <= u <= 1.0 + 1e-12:
        return (a[0] + t * r[0], a[1] + t * r[1])
    return None


def closest_points_between_segments(
    a: tuple[float, float],
    b: tuple[float, float],
    c: tuple[float, float],
    d: tuple[float, float],
) -> tuple[float, tuple[float, float], tuple[float, float]]:
    intersection = _segment_intersection(a, b, c, d)
    if intersection is not None:
        return 0.0, intersection, intersection

    candidates = []
    point, distance = closest_point_on_segment(a, c, d)
    candidates.append((distance, a, point))
    point, distance = closest_point_on_segment(b, c, d)
    candidates.append((distance, b, point))
    point, distance = closest_point_on_segment(c, a, b)
    candidates.append((distance, point, c))
    point, distance = closest_point_on_segment(d, a, b)
    candidates.append((distance, point, d))
    return min(candidates, key=lambda item: item[0])


def segment_polyline_conflict(
    segment_start: tuple[float, float],
    segment_end: tuple[float, float],
    polyline: Sequence[tuple[float, float]],
) -> PlanConflict | None:
    if not polyline:
        return None
    if len(polyline) == 1:
        motion_point, distance = closest_point_on_segment(
            polyline[0], segment_start, segment_end
        )
        return PlanConflict(distance, motion_point, polyline[0], 0)

    best: PlanConflict | None = None
    for index in range(len(polyline) - 1):
        distance, motion_point, plan_point = closest_points_between_segments(
            segment_start,
            segment_end,
            polyline[index],
            polyline[index + 1],
        )
        candidate = PlanConflict(distance, motion_point, plan_point, index)
        if best is None or candidate.distance_m < best.distance_m:
            best = candidate
            if best.distance_m <= 1e-9:
                break
    return best


def point_polyline_distance(
    point: tuple[float, float],
    polyline: Sequence[tuple[float, float]],
) -> float:
    if not polyline:
        return math.inf
    if len(polyline) == 1:
        return math.hypot(point[0] - polyline[0][0], point[1] - polyline[0][1])
    return min(
        closest_point_on_segment(point, polyline[index], polyline[index + 1])[1]
        for index in range(len(polyline) - 1)
    )
