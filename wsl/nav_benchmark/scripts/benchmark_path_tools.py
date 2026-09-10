#!/usr/bin/env python3
"""Pure helpers for benchmark path/result visualization.

This module deliberately contains no Tkinter or ROS dependencies.  It can be
unit-tested in isolation and is shared by the QA GUI's Path View.
"""

from __future__ import annotations

import csv
import math
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import yaml

from benchmark_common import Pose2D, normalize_angle


@dataclass(frozen=True)
class PlanSeries:
    plan_id: int
    nav_goal_index: int
    frame_id: str
    source_topic: str
    time_sec: float
    points: tuple[Pose2D, ...]


@dataclass(frozen=True)
class WorldToMapTransform:
    world_start: Pose2D
    map_start: Pose2D
    yaw_offset: float

    def point(self, x: float, y: float) -> tuple[float, float]:
        dx = float(x) - self.world_start.x
        dy = float(y) - self.world_start.y
        c = math.cos(self.yaw_offset)
        s = math.sin(self.yaw_offset)
        return (
            self.map_start.x + c * dx - s * dy,
            self.map_start.y + s * dx + c * dy,
        )

    def yaw(self, world_yaw: float) -> float:
        return normalize_angle(float(world_yaw) + self.yaw_offset)


def _float(value, default=math.nan) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def _finite(value: float) -> bool:
    return math.isfinite(value)


def load_case(case_path: Path) -> dict:
    with Path(case_path).open("r", encoding="utf-8") as handle:
        return yaml.safe_load(handle) or {}


def load_summary(run_dir: Path) -> dict:
    path = Path(run_dir) / "summary.yaml"
    if not path.exists():
        return {}
    with path.open("r", encoding="utf-8") as handle:
        return yaml.safe_load(handle) or {}


def load_actual_trajectory(run_dir: Path) -> list[Pose2D]:
    path = Path(run_dir) / "trajectory.csv"
    if not path.exists():
        return []

    poses: list[Pose2D] = []
    with path.open("r", newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            x = _float(row.get("map_x"))
            y = _float(row.get("map_y"))
            yaw = _float(row.get("map_yaw_rad"), 0.0)
            if _finite(x) and _finite(y):
                poses.append(Pose2D(x, y, yaw if _finite(yaw) else 0.0))
    return poses


def load_plans(run_dir: Path) -> list[PlanSeries]:
    path = Path(run_dir) / "planned_paths.csv"
    if not path.exists():
        return []

    grouped: dict[int, dict] = {}
    with path.open("r", newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            try:
                plan_id = int(row["plan_id"])
                nav_goal_index = int(row.get("nav_goal_index", 0) or 0)
                pose_index = int(row["pose_index"])
            except (KeyError, TypeError, ValueError):
                continue

            x = _float(row.get("x"))
            y = _float(row.get("y"))
            yaw = _float(row.get("yaw_rad"), 0.0)
            if not (_finite(x) and _finite(y)):
                continue

            bucket = grouped.setdefault(
                plan_id,
                {
                    "nav_goal_index": nav_goal_index,
                    "frame_id": row.get("frame_id", ""),
                    # v3 files do not have source_topic.  Keep them readable.
                    "source_topic": row.get("source_topic", ""),
                    "time_sec": _float(row.get("time_sec"), 0.0),
                    "points": [],
                },
            )
            bucket["points"].append(
                (pose_index, Pose2D(x, y, yaw if _finite(yaw) else 0.0))
            )

    plans: list[PlanSeries] = []
    for plan_id in sorted(grouped):
        bucket = grouped[plan_id]
        ordered = tuple(
            pose for _, pose in sorted(bucket["points"], key=lambda item: item[0])
        )
        if not ordered:
            continue
        plans.append(
            PlanSeries(
                plan_id=plan_id,
                nav_goal_index=int(bucket["nav_goal_index"]),
                frame_id=str(bucket["frame_id"]),
                source_topic=str(bucket["source_topic"]),
                time_sec=float(bucket["time_sec"]),
                points=ordered,
            )
        )
    return plans


def is_canonical_global_plan(plan: PlanSeries) -> bool:
    """Accept current /plan(map) rows and unambiguous legacy map rows."""
    topic = str(plan.source_topic).strip()
    frame = str(plan.frame_id).strip().lstrip("/")
    return topic in {"", "/plan"} and frame == "map"


def latest_plan_per_nav_goal(plans: Iterable[PlanSeries]) -> list[PlanSeries]:
    """Return one clean plan per Far-Goal/Nav2 segment.

    Nav2 may publish multiple replans for one NavigateToPose goal.  The viewer
    normally wants the last plan used for each segment rather than every
    historical revision on top of one another.
    """
    latest: dict[int, PlanSeries] = {}
    fallback: list[PlanSeries] = []

    for plan in plans:
        if plan.nav_goal_index > 0:
            latest[plan.nav_goal_index] = plan
        else:
            fallback.append(plan)

    if latest:
        return [latest[key] for key in sorted(latest)]
    return fallback[-1:] if fallback else []


def first_plan_per_nav_goal(plans: Iterable[PlanSeries]) -> list[PlanSeries]:
    """Return the FIRST plan of each Far-Goal/Nav2 segment.

    A Nav2 global plan always starts at the robot's current pose, so the
    *last* replan of a segment is by definition the short leftover piece near
    that segment's target.  The *first* plan of each segment starts where the
    goal was issued, so the segments tile the whole mission: this is the
    planner's intended route, which is what an avoidance benchmark wants to
    look at.
    """
    first: dict[int, PlanSeries] = {}
    fallback: list[PlanSeries] = []

    for plan in plans:
        if plan.nav_goal_index > 0:
            first.setdefault(plan.nav_goal_index, plan)
        else:
            fallback.append(plan)

    if first:
        return [first[key] for key in sorted(first)]
    return fallback[:1] if fallback else []


def plan_counts_per_nav_goal(plans: Iterable[PlanSeries]) -> dict[int, int]:
    """How many /plan messages were recorded for each Nav2 goal."""
    counts: dict[int, int] = {}
    for plan in plans:
        counts[plan.nav_goal_index] = counts.get(plan.nav_goal_index, 0) + 1
    return counts


_FINAL_GOAL_RE = re.compile(
    r"x=([-+0-9.eE]+),\s*y=([-+0-9.eE]+),\s*yaw=([-+0-9.eE]+)"
)


def requested_goal_from_result(run_dir: Path, summary: dict) -> Pose2D | None:
    value = summary.get("requested_goal_map_pose")
    if isinstance(value, dict):
        x = _float(value.get("x"))
        y = _float(value.get("y"))
        yaw = _float(value.get("yaw_rad"), 0.0)
        if _finite(x) and _finite(y):
            return Pose2D(x, y, yaw if _finite(yaw) else 0.0)

    events = Path(run_dir) / "events.csv"
    if events.exists():
        with events.open("r", newline="", encoding="utf-8") as handle:
            for row in csv.DictReader(handle):
                if row.get("event") != "FINAL_GOAL":
                    continue
                match = _FINAL_GOAL_RE.search(row.get("detail", ""))
                if match:
                    return Pose2D(
                        float(match.group(1)),
                        float(match.group(2)),
                        float(match.group(3)),
                    )
    return None


def derive_world_to_map(
    case: dict,
    actual: list[Pose2D],
    *,
    requested_goal: Pose2D | None = None,
) -> WorldToMapTransform | None:
    robot = case.get("robot") or {}
    goal = case.get("goal") or {}
    if "x" not in robot or "y" not in robot:
        return None

    world_start = Pose2D(
        float(robot["x"]),
        float(robot["y"]),
        float(robot.get("yaw", 0.0)),
    )

    # New result files contain the exact requested map goal. Derive the map
    # anchor from that fixed transform so pre-goal robot motion can never shift
    # the obstacle drawing. Older runs fall back to the first trajectory pose.
    if (
        requested_goal is not None
        and "x" in goal
        and "y" in goal
    ):
        world_goal = Pose2D(
            float(goal["x"]),
            float(goal["y"]),
            float(goal.get("yaw", 0.0)),
        )
        yaw_offset = normalize_angle(requested_goal.yaw - world_goal.yaw)
        dx = world_goal.x - world_start.x
        dy = world_goal.y - world_start.y
        c = math.cos(yaw_offset)
        s = math.sin(yaw_offset)
        map_start = Pose2D(
            requested_goal.x - (c * dx - s * dy),
            requested_goal.y - (s * dx + c * dy),
            normalize_angle(world_start.yaw + yaw_offset),
        )
        return WorldToMapTransform(world_start, map_start, yaw_offset)

    if not actual:
        return None
    map_start = actual[0]
    yaw_offset = normalize_angle(map_start.yaw - world_start.yaw)
    return WorldToMapTransform(world_start, map_start, yaw_offset)


def case_goal_in_map(
    case: dict,
    transform: WorldToMapTransform | None,
) -> Pose2D | None:
    goal = case.get("goal") or {}
    if transform is None or "x" not in goal or "y" not in goal:
        return None
    x, y = transform.point(float(goal["x"]), float(goal["y"]))
    return Pose2D(x, y, transform.yaw(float(goal.get("yaw", 0.0))))


def _rotate_translate(
    points: Iterable[tuple[float, float]],
    center_x: float,
    center_y: float,
    yaw: float,
) -> list[tuple[float, float]]:
    c = math.cos(yaw)
    s = math.sin(yaw)
    return [
        (
            center_x + c * x - s * y,
            center_y + s * x + c * y,
        )
        for x, y in points
    ]


def obstacle_outlines_in_map(
    case: dict,
    transform: WorldToMapTransform | None,
) -> list[dict]:
    """Return simple top-down outlines for benchmark obstacles.

    The benchmark world stores obstacle geometry in world coordinates while
    planned/actual paths are in map coordinates.  This helper performs the
    same rigid world->map alignment used for Start/Goal visualization.
    """
    if transform is None:
        return []

    outlines: list[dict] = []
    for index, obstacle in enumerate(case.get("obstacles", []), start=1):
        shape = obstacle.get("shape")
        pose = obstacle.get("pose") or {}
        if "x" not in pose or "y" not in pose:
            continue

        cx = float(pose["x"])
        cy = float(pose["y"])
        yaw = float(pose.get("yaw", 0.0))
        name = str(obstacle.get("name", f"obstacle_{index:02d}"))

        if shape == "cylinder":
            mx, my = transform.point(cx, cy)
            outlines.append(
                {
                    "kind": "circle",
                    "name": name,
                    "center": (mx, my),
                    "radius": float(obstacle["radius"]),
                }
            )
            continue

        if shape == "box":
            sx, sy, _ = map(float, obstacle["size"])
            local = [
                (-sx / 2.0, -sy / 2.0),
                (sx / 2.0, -sy / 2.0),
                (sx / 2.0, sy / 2.0),
                (-sx / 2.0, sy / 2.0),
            ]
        elif shape == "polygon":
            local = [(float(x), float(y)) for x, y in obstacle["points"]]
        else:
            continue

        world_points = _rotate_translate(local, cx, cy, yaw)
        map_points = [transform.point(x, y) for x, y in world_points]
        outlines.append(
            {
                "kind": "polygon",
                "name": name,
                "points": map_points,
            }
        )

    return outlines