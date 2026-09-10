#!/usr/bin/env python3
"""Shared, ROS-independent primitives for the Nav benchmark suite.

Keep this module importable on a normal Python installation.  Runner, ROS
nodes, the GUI, and tests all use the same version, case-path, run-id, and
coordinate-conversion rules from here.
"""

from __future__ import annotations

import math
import re
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Mapping

import yaml


SUITE_VERSION = "2.0.0"
RESULT_SCHEMA_VERSION = 3
PLAN_SCHEMA_VERSION = 1

CASE_ID_RE = re.compile(r"^S(?P<scenario>\d+)_(?P<case>\d+)$", re.IGNORECASE)
RUN_ID_RE = re.compile(r"^run_(?P<number>\d+)$")


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float = 0.0


@dataclass(frozen=True)
class CaseSpec:
    case_id: str
    scenario: str
    robot: Pose2D
    goal: Pose2D
    raw: dict

    @property
    def direct_distance_m(self) -> float:
        return math.hypot(
            self.goal.x - self.robot.x,
            self.goal.y - self.robot.y,
        )


@dataclass(frozen=True)
class TerminalPolicy:
    is_intermediate: bool
    wait_sec: float
    timeout_result: str


def normalize_angle(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


def quaternion_to_yaw(quaternion) -> float:
    siny = 2.0 * (
        float(quaternion.w) * float(quaternion.z)
        + float(quaternion.x) * float(quaternion.y)
    )
    cosy = 1.0 - 2.0 * (
        float(quaternion.y) ** 2 + float(quaternion.z) ** 2
    )
    return math.atan2(siny, cosy)


def yaw_to_quaternion_zw(yaw: float) -> tuple[float, float]:
    return math.sin(float(yaw) / 2.0), math.cos(float(yaw) / 2.0)


def require_finite(label: str, *values: float) -> None:
    if not all(math.isfinite(float(value)) for value in values):
        rendered = ", ".join(repr(value) for value in values)
        raise ValueError(f"{label} contains a non-finite value: {rendered}")


def parse_case_id(case_id: str) -> tuple[str, str]:
    normalized = str(case_id).strip().upper()
    match = CASE_ID_RE.fullmatch(normalized)
    if match is None:
        raise ValueError(
            f"Invalid case id '{case_id}'. Expected a value such as S1_01."
        )
    return normalized, f"S{int(match.group('scenario'))}"


def _pose_from_mapping(label: str, value: Mapping) -> Pose2D:
    try:
        pose = Pose2D(
            float(value["x"]),
            float(value["y"]),
            float(value.get("yaw", 0.0)),
        )
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError(f"Invalid {label} pose in case YAML: {exc}") from exc
    require_finite(label, pose.x, pose.y, pose.yaw)
    return pose


def load_case_spec(case_id: str, cases_root: Path | None = None) -> CaseSpec:
    normalized, scenario = parse_case_id(case_id)
    root = Path(cases_root) if cases_root is not None else (
        Path.home() / "nav_benchmark" / "cases"
    )
    case_path = root / scenario / f"{normalized}.yaml"
    if not case_path.exists():
        raise FileNotFoundError(f"Case not found: {case_path}")

    with case_path.open("r", encoding="utf-8") as handle:
        raw = yaml.safe_load(handle) or {}
    if not isinstance(raw, dict):
        raise ValueError(f"Case YAML root must be a mapping: {case_path}")

    robot = _pose_from_mapping("robot", raw.get("robot") or {})
    goal = _pose_from_mapping("goal", raw.get("goal") or {})
    return CaseSpec(normalized, scenario, robot, goal, raw)


def transform_world_goal_to_map(
    world_start: Pose2D,
    world_goal: Pose2D,
    map_start: Pose2D,
) -> Pose2D:
    """Apply the rigid world->map transform fixed at the case start pose."""
    require_finite(
        "world/map poses",
        world_start.x,
        world_start.y,
        world_start.yaw,
        world_goal.x,
        world_goal.y,
        world_goal.yaw,
        map_start.x,
        map_start.y,
        map_start.yaw,
    )

    yaw_offset = normalize_angle(map_start.yaw - world_start.yaw)
    dx = world_goal.x - world_start.x
    dy = world_goal.y - world_start.y
    c = math.cos(yaw_offset)
    s = math.sin(yaw_offset)
    return Pose2D(
        map_start.x + c * dx - s * dy,
        map_start.y + s * dx + c * dy,
        normalize_angle(world_goal.yaw + yaw_offset),
    )


def pose_error(current: Pose2D, reference: Pose2D) -> tuple[float, float]:
    xy = math.hypot(current.x - reference.x, current.y - reference.y)
    yaw = abs(normalize_angle(current.yaw - reference.yaw))
    return xy, yaw


def navigation_terminal_policy(
    status_name: str,
    final_distance_m: float | None,
    *,
    final_near_distance_m: float = 0.50,
    final_grace_sec: float = 3.0,
    intermediate_wait_sec: float = 12.0,
) -> TerminalPolicy:
    """Choose how long to wait before treating a Nav2 terminal as mission end.

    Far Goal Manager creates several internal NavigateToPose goals. A terminal
    far from the final mission pose is therefore provisional: a replacement
    goal gets a longer window to appear.
    """
    status = str(status_name).upper()
    is_intermediate = (
        final_distance_m is None
        or not math.isfinite(float(final_distance_m))
        or float(final_distance_m) > float(final_near_distance_m)
    )
    if not is_intermediate:
        return TerminalPolicy(False, float(final_grace_sec), status)
    timeout_result = "MISSION_STALLED" if status == "SUCCEEDED" else status
    return TerminalPolicy(True, float(intermediate_wait_sec), timeout_result)


def make_run_dir(case_root: Path) -> Path:
    """Create a monotonically increasing run_NNN directory.

    Deleted holes are intentionally not reused; a run id must never appear to
    move backward.  mkdir(exist_ok=False) keeps concurrent starts safe.
    """
    root = Path(case_root)
    root.mkdir(parents=True, exist_ok=True)
    numbers = []
    for child in root.iterdir():
        if not child.is_dir():
            continue
        match = RUN_ID_RE.fullmatch(child.name)
        if match is not None:
            numbers.append(int(match.group("number")))

    candidate = max(numbers, default=0) + 1
    while True:
        path = root / f"run_{candidate:03d}"
        try:
            path.mkdir(exist_ok=False)
            return path
        except FileExistsError:
            candidate += 1


def suite_metadata() -> dict:
    return {
        "suite_version": SUITE_VERSION,
        "result_schema_version": RESULT_SCHEMA_VERSION,
        "plan_schema_version": PLAN_SCHEMA_VERSION,
        "generated_at": datetime.now().astimezone().isoformat(timespec="seconds"),
    }