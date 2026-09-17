#!/usr/bin/env python3
"""Generate 20 deterministic, behaviorally diverse S5 dynamic-obstacle cases.

S5 remains a dynamic-only benchmark: no static obstacles are added here because
S1-S4 already cover static planning/control.  Diversity comes from dynamic
motion, timing, size/shape, speed, stop/resume, reversal, lane merge, head-on,
slow lead obstacles, multiple actors, and seeded random-like waypoint motion.
"""

from __future__ import annotations

import argparse
import math
import random
from pathlib import Path

import yaml


def world_point(center, heading, longitudinal, lateral):
    c = math.cos(heading)
    s = math.sin(heading)
    return {
        "x": round(center[0] + longitudinal * c - lateral * s, 6),
        "y": round(center[1] + longitudinal * s + lateral * c, 6),
    }


def _path_length(points):
    return sum(
        math.hypot(b[0] - a[0], b[1] - a[1])
        for a, b in zip(points, points[1:])
    )


def _first_route_interaction_distance(points):
    """Distance travelled until the local polyline first reaches/crosses d=0."""
    travelled = 0.0
    for a, b in zip(points, points[1:]):
        s0, d0 = a
        s1, d1 = b
        leg = math.hypot(s1 - s0, d1 - d0)
        if abs(d0) <= 1e-9:
            return travelled
        if d0 * d1 <= 0.0 and abs(d1 - d0) > 1e-9:
            fraction = min(1.0, max(0.0, -d0 / (d1 - d0)))
            return travelled + leg * fraction
        travelled += leg
    return 0.5 * travelled


def auto_trigger_distance(points, speed):
    # Robot timing estimate only; runtime validity still checks actual spatial
    # and temporal interaction. Keep bounded to avoid absurd early triggers.
    motion_distance = _first_route_interaction_distance(points)
    seconds = motion_distance / max(0.05, float(speed))
    return round(min(8.0, max(2.5, 0.75 * seconds)), 3)


def seeded_random_path(seed: int, *, start_s=-4.0, steps=5):
    """Reproducible random-like zigzag that crosses the route several times."""
    rng = random.Random(seed)
    result = []
    s = float(start_s)
    sign = 1.0
    for _ in range(steps):
        s += rng.uniform(1.1, 1.8)
        lateral = sign * rng.uniform(0.6, 1.7)
        speed = rng.uniform(0.75, 1.00)
        hold = 0.0 if rng.random() < 0.80 else rng.uniform(0.2, 0.5)
        result.append({
            "point": (round(s, 3), round(lateral, 3)),
            "speed": round(speed, 3),
            "hold": round(hold, 3),
        })
        sign *= -1.0
    return result


def _normalize_path_item(item):
    if "path" in item:
        return list(item["path"])
    return [{"point": item["end"], "speed": item["speed"], "hold": item.get("hold", 0.0)}]


def dynamic_item(case_number, obstacle_number, center, heading, item):
    start_local = tuple(item["start"])
    path_items = _normalize_path_item(item)
    first_point = tuple(path_items[0]["point"])
    first_yaw = math.atan2(first_point[1] - start_local[1], first_point[0] - start_local[0]) + heading
    start = world_point(center, heading, *start_local)

    local_points = [start_local] + [tuple(p["point"]) for p in path_items]
    default_speed = float(item.get("speed", path_items[0].get("speed", 0.5)))
    trigger_distance = float(item.get(
        "trigger_distance",
        auto_trigger_distance(local_points, default_speed),
    ))

    waypoints = []
    previous = start_local
    for path_item in path_items:
        point = tuple(path_item["point"])
        target = world_point(center, heading, *point)
        segment_yaw = math.atan2(point[1] - previous[1], point[0] - previous[0]) + heading
        waypoint = {
            **target,
            "yaw": round(segment_yaw, 6),
            "speed_mps": float(path_item.get("speed", default_speed)),
        }
        hold = float(path_item.get("hold", 0.0))
        if hold > 0.0:
            waypoint["hold_sec"] = hold
        waypoints.append(waypoint)
        previous = point

    path_length = _path_length(local_points)
    result = {
        "name": f"s5_{case_number:02d}_dyn_{obstacle_number:02d}",
        "shape": item.get("shape", "cylinder"),
        "mass": float(item.get("mass", 40.0)),
        "pose": {
            **start,
            "yaw": round(first_yaw, 6),
        },
        "motion": {
            "speed_mps": default_speed,
            "trigger": {
                "type": item.get("trigger_type", "robot_distance"),
            },
        },
        "required": bool(item.get("required", True)),
        "validation": {
            "path_intersection_tolerance_m": float(item.get("path_tolerance", 0.30)),
            "max_robot_interaction_distance_m": float(item.get("interaction_distance", 3.50)),
            "min_travel_m": round(float(item.get("min_travel_m", max(0.50, 0.60 * path_length))), 3),
            "min_motion_before_collision_valid_m": float(item.get("min_motion_before_collision_valid_m", 0.50)),
            "end_tolerance_m": float(item.get("end_tolerance_m", 0.15)),
            "initial_pose_tolerance_m": float(item.get("initial_pose_tolerance_m", 0.35)),
            "allow_interaction_after_motion": bool(item.get("allow_interaction_after_motion", False)),
        },
    }

    if result["motion"]["trigger"]["type"] == "elapsed_after_goal":
        result["motion"]["trigger"]["delay_sec"] = float(item.get("delay_sec", 2.0))
    else:
        result["motion"]["trigger"]["distance_m"] = trigger_distance

    # Preserve the compact v1 representation for a truly simple one-leg case.
    # Complex motion uses schema-v2 waypoints.
    if len(waypoints) == 1 and float(waypoints[0].get("hold_sec", 0.0)) == 0.0:
        wp = waypoints[0]
        result["motion"]["end"] = {"x": wp["x"], "y": wp["y"], "yaw": wp["yaw"]}
        result["motion"]["speed_mps"] = wp["speed_mps"]
    else:
        result["motion"]["waypoints"] = waypoints

    if result["shape"] == "cylinder":
        result["radius"] = float(item.get("radius", 0.35))
        result["height"] = float(item.get("height", 1.40))
    elif result["shape"] == "box":
        result["size"] = [float(value) for value in item.get("size", (0.8, 0.6, 1.0))]
    else:
        raise ValueError(f"Unsupported dynamic shape: {result['shape']}")

    return result


# Local coordinates: longitudinal s follows the robot's nominal route and
# lateral d is left/right of it.  This makes scenario intent readable.
PROFILES = [
    # 01: validated baseline - intentionally kept equivalent to the current case.
    {
        "title": "basic perpendicular crossing",
        "center": (-35.0, -35.0), "heading_deg": 0.0, "length": 30.0,
        "dynamic": [{"start": (0.0, -2.5), "end": (0.0, 2.5), "speed": 0.70, "trigger_distance": 3.75, "min_travel_m": 4.0, "min_motion_before_collision_valid_m": 0.70}],
    },
    # 02-05: crossing geometry / speed / size diversity.
    {
        "title": "reverse slow small pedestrian crossing",
        "center": (0.0, -35.0), "heading_deg": 180.0, "length": 30.0,
        "dynamic": [{"start": (0.0, 2.6), "end": (0.0, -2.6), "speed": 0.35, "radius": 0.24, "height": 1.65, "mass": 20.0, "trigger_distance": 7.0}],
    },
    {
        "title": "fast large perpendicular crossing",
        "center": (35.0, -35.0), "heading_deg": 90.0, "length": 34.0,
        "dynamic": [{"start": (0.0, -4.0), "end": (0.0, 4.0), "speed": 1.10, "radius": 0.48, "height": 1.25, "mass": 70.0, "trigger_distance": 3.0}],
    },
    {
        "title": "very slow crossing with early visibility",
        "center": (-35.0, 0.0), "heading_deg": -90.0, "length": 32.0,
        "dynamic": [{"start": (0.0, -1.8), "end": (0.0, 1.8), "speed": 0.25, "radius": 0.22, "height": 1.75, "mass": 18.0, "trigger_distance": 7.0}],
    },
    {
        "title": "oblique crossing",
        "center": (0.0, 0.0), "heading_deg": 30.0, "length": 34.0,
        "dynamic": [{"start": (-2.0, -3.0), "end": (2.0, 3.0), "speed": 0.65, "radius": 0.32, "trigger_distance": 4.0}],
    },
    # 06-08: merge / head-on / same-direction blocker.
    {
        "title": "merge onto route then continue ahead",
        "center": (35.0, 0.0), "heading_deg": 150.0, "length": 38.0,
        "dynamic": [{
            "start": (-2.5, -2.5), "speed": 0.60, "trigger_distance": 5.5,
            "path": [
                {"point": (-1.0, 0.0), "speed": 0.80},
                {"point": (2.5, 0.20), "speed": 0.70},
                {"point": (4.5, 0.20), "speed": 0.65},
            ],
        }],
    },
    {
        "title": "head-on approach along nominal route",
        "center": (-35.0, 35.0), "heading_deg": -30.0, "length": 38.0,
        "dynamic": [{
            "start": (6.5, 2.5), "speed": 0.80, "radius": 0.30,
            "trigger_distance": 6.5,
            "path": [
                {"point": (5.0, 0.0), "speed": 0.85},
                {"point": (0.0, 0.0), "speed": 0.75},
            ],
        }],
    },
    {
        "title": "slow same-direction blocker ahead",
        "center": (0.0, 35.0), "heading_deg": -150.0, "length": 38.0,
        "dynamic": [{
            "start": (-2.5, -2.5), "speed": 0.60,
            "shape": "box", "size": (0.90, 0.65, 0.85), "mass": 65.0,
            "trigger_distance": 6.0,
            "path": [
                {"point": (-1.0, 0.0), "speed": 0.70},
                {"point": (2.8, 0.0), "speed": 0.48},
            ],
        }],
    },
    # 09-12: stop, stop/resume, return, hesitation.
    {
        "title": "enter and stop on route",
        "center": (35.0, 35.0), "heading_deg": 15.0, "length": 34.0,
        "dynamic": [{
            "start": (0.0, -3.2), "end": (0.0, 0.0), "speed": 0.45,
            "trigger_distance": 5.5, "allow_interaction_after_motion": True,
        }],
    },
    {
        "title": "crossing stop then resume",
        "center": (-20.0, -15.0), "heading_deg": 75.0, "length": 36.0,
        "dynamic": [{
            "start": (0.0, -3.0), "speed": 0.50, "trigger_distance": 5.0,
            "path": [
                {"point": (0.0, 0.0), "speed": 0.60, "hold": 2.0},
                {"point": (0.0, 3.0), "speed": 0.70},
            ],
        }],
    },
    {
        "title": "cross route then reverse back",
        "center": (20.0, -15.0), "heading_deg": 105.0, "length": 38.0,
        "dynamic": [{
            "start": (0.0, -3.0), "speed": 0.80, "trigger_distance": 5.0,
            "path": [
                {"point": (0.0, 2.8), "speed": 0.80, "hold": 0.5},
                {"point": (0.0, -2.4), "speed": 0.90},
            ],
        }],
    },
    {
        "title": "hesitate on route then retreat",
        "center": (-20.0, 15.0), "heading_deg": -75.0, "length": 34.0,
        "dynamic": [{
            "start": (-1.0, -2.7), "speed": 0.55, "trigger_distance": 5.0,
            "path": [
                {"point": (-1.0, 0.0), "speed": 0.55, "hold": 1.2},
                {"point": (-1.0, -2.9), "speed": 0.70},
            ],
        }],
    },
    # 13-14: non-straight / random-like motion.
    {
        "title": "zigzag repeatedly across route",
        "center": (20.0, 15.0), "heading_deg": 45.0, "length": 40.0,
        "dynamic": [{
            "start": (-3.5, -2.2), "speed": 0.75, "trigger_distance": 5.5,
            "path": [
                {"point": (-2.0, 1.4), "speed": 0.80},
                {"point": (0.0, -1.2), "speed": 0.90},
                {"point": (2.2, 1.7), "speed": 0.80},
            ],
        }],
    },
    {
        "title": "seeded random-like wandering",
        "center": (0.0, -10.0), "heading_deg": 135.0, "length": 40.0,
        "dynamic": [{
            "start": (-3.8, -2.2), "speed": 0.75, "trigger_distance": 5.5,
            "path": seeded_random_path(514, start_s=-3.8, steps=4),
            "radius": 0.27, "mass": 28.0,
        }],
    },
    # 15-16: large cart and merge/block behavior.
    {
        "title": "large cart full crossing",
        "center": (0.0, 10.0), "heading_deg": -45.0, "length": 38.0,
        "dynamic": [{
            "start": (0.0, 3.2), "end": (0.0, -3.2), "speed": 0.45,
            "shape": "box", "size": (1.25, 0.85, 1.10), "mass": 95.0,
            "trigger_distance": 6.0,
        }],
    },
    {
        "title": "wide cart enters route then blocks ahead",
        "center": (-10.0, 0.0), "heading_deg": -135.0, "length": 40.0,
        "dynamic": [{
            "start": (-2.5, -2.8), "speed": 0.65, "trigger_distance": 6.0,
            "shape": "box", "size": (1.45, 0.80, 1.05), "mass": 100.0,
            "path": [
                {"point": (-2.5, 0.0), "speed": 0.70},
                {"point": (1.8, 0.0), "speed": 0.55},
            ],
        }],
    },
    # 17-18: multiple actors with different conflict directions.
    {
        "title": "two sequential crossings different speeds",
        "center": (10.0, 0.0), "heading_deg": 20.0, "length": 40.0,
        "dynamic": [
            {"start": (-5.0, -2.7), "end": (-5.0, 2.7), "speed": 0.45, "radius": 0.25, "trigger_distance": 5.5},
            {"start": (5.0, 3.2), "end": (5.0, -3.2), "speed": 0.85, "radius": 0.38, "trigger_distance": 3.0},
        ],
    },
    {
        "title": "head-on actor plus lateral crosser",
        "center": (-5.0, 25.0), "heading_deg": 70.0, "length": 42.0,
        "dynamic": [
            {
                "start": (6.5, 2.3), "speed": 0.80, "radius": 0.30, "trigger_distance": 6.5,
                "path": [
                    {"point": (5.0, 0.0), "speed": 0.85},
                    {"point": (0.5, 0.0), "speed": 0.75},
                ],
            },
            {"start": (3.0, -3.0), "end": (3.0, 3.0), "speed": 0.70, "radius": 0.24, "trigger_distance": 4.0},
        ],
    },
    # 19: stop on route, then go back to where it came from.
    {
        "title": "cart stops on route then returns",
        "center": (25.0, -5.0), "heading_deg": 110.0, "length": 40.0,
        "dynamic": [{
            "start": (0.0, 3.2), "speed": 0.55, "trigger_distance": 6.5,
            "shape": "box", "size": (0.85, 0.60, 0.90), "mass": 65.0,
            "path": [
                {"point": (0.0, 0.0), "speed": 0.55, "hold": 2.0},
                {"point": (0.0, 3.2), "speed": 0.70},
            ],
        }],
    },
    # 20: mixed high-complexity scene.
    {
        "title": "mixed three-actor dynamic scene",
        "center": (-25.0, 5.0), "heading_deg": 160.0, "length": 46.0,
        "dynamic": [
            {
                "start": (-7.0, -2.5), "speed": 0.75, "trigger_distance": 5.5,
                "path": [
                    {"point": (-5.5, 1.5), "speed": 0.85},
                    {"point": (-3.5, -1.2), "speed": 0.95},
                    {"point": (-1.5, 2.0), "speed": 0.85},
                ],
                "radius": 0.25, "mass": 25.0,
            },
            {
                "start": (0.0, -2.3), "speed": 0.65,
                "shape": "box", "size": (1.00, 0.70, 0.90), "mass": 80.0,
                "trigger_distance": 6.0,
                "path": [
                    {"point": (1.0, 0.0), "speed": 0.70},
                    {"point": (5.0, 0.0), "speed": 0.55},
                ],
            },
            {
                "start": (8.0, 3.2), "end": (8.0, -3.2), "speed": 0.95,
                "radius": 0.35, "mass": 45.0, "trigger_distance": 3.0,
            },
        ],
    },
]


def build_case(index, profile):
    heading = math.radians(float(profile["heading_deg"]))
    center = profile["center"]
    half_length = 0.5 * float(profile["length"])
    start = world_point(center, heading, -half_length, 0.0)
    goal = world_point(center, heading, half_length, 0.0)
    case_id = f"S5_{index:02d}"
    return {
        "scenario": "S5",
        "case_id": case_id,
        "description": profile["title"],
        "robot": {**start, "yaw": round(heading, 6)},
        "goal": {**goal, "yaw": round(heading, 6)},
        # Keep S5 focused on dynamic behavior. Static geometry remains S1-S4.
        "obstacles": [],
        "dynamic_obstacles": [
            dynamic_item(index, obstacle_index, center, heading, item)
            for obstacle_index, item in enumerate(profile["dynamic"], start=1)
        ],
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--output",
        type=Path,
        default=Path.home() / "nav_benchmark" / "cases" / "S5",
    )
    parser.add_argument("--force", action="store_true")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)

    for index, profile in enumerate(PROFILES, start=1):
        case = build_case(index, profile)
        path = args.output / f"{case['case_id']}.yaml"
        if path.exists() and not args.force:
            raise SystemExit(f"Refusing to overwrite {path}; pass --force")
        path.write_text(
            yaml.safe_dump(case, sort_keys=False, allow_unicode=True),
            encoding="utf-8",
        )
        print(path)


if __name__ == "__main__":
    main()
