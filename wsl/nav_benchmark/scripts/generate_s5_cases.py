#!/usr/bin/env python3
"""Generate the 20 deterministic S5 dynamic-obstacle benchmark cases."""

from __future__ import annotations

import argparse
import math
from pathlib import Path

import yaml


def world_point(center, heading, longitudinal, lateral):
    cosine = math.cos(heading)
    sine = math.sin(heading)
    return {
        "x": round(center[0] + longitudinal * cosine - lateral * sine, 6),
        "y": round(center[1] + longitudinal * sine + lateral * cosine, 6),
    }


def crossing_trigger_distance(start, end, speed):
    s0, d0 = start
    s1, d1 = end
    if abs(d1 - d0) < 1e-9:
        fraction = 0.5
    else:
        fraction = min(1.0, max(0.0, -d0 / (d1 - d0)))
    distance_to_route = math.hypot(s1 - s0, d1 - d0) * fraction
    # The A200 does not instantly reach max_vel_x.  0.75 m/s is a useful
    # trigger-time estimate; actual temporal overlap is still measured and a
    # mistimed trial becomes INVALID_STIMULUS instead of silently passing.
    return round(min(8.0, max(2.5, 0.75 * distance_to_route / speed)), 3)


def dynamic_item(case_number, obstacle_number, center, heading, item):
    start_local = item["start"]
    end_local = item["end"]
    speed = float(item["speed"])
    start = world_point(center, heading, *start_local)
    end = world_point(center, heading, *end_local)
    motion_yaw = math.atan2(end["y"] - start["y"], end["x"] - start["x"])
    segment_length = math.hypot(
        end["x"] - start["x"], end["y"] - start["y"]
    )
    trigger_distance = item.get(
        "trigger_distance",
        crossing_trigger_distance(start_local, end_local, speed),
    )

    result = {
        "name": f"s5_{case_number:02d}_dyn_{obstacle_number:02d}",
        "shape": item.get("shape", "cylinder"),
        "mass": float(item.get("mass", 40.0)),
        "pose": {
            **start,
            "yaw": round(motion_yaw, 6),
        },
        "motion": {
            "end": {
                **end,
                "yaw": round(motion_yaw, 6),
            },
            "speed_mps": speed,
            "trigger": {
                "type": "robot_distance",
                "distance_m": float(trigger_distance),
            },
        },
        "required": True,
        "validation": {
            "path_intersection_tolerance_m": 0.30,
            "max_robot_interaction_distance_m": 3.50,
            "min_travel_m": round(0.80 * segment_length, 3),
            "min_motion_before_collision_valid_m": 0.50,
            "end_tolerance_m": 0.15,
            "initial_pose_tolerance_m": 0.35,
            "allow_interaction_after_motion": bool(item.get("stop_on_route", False)),
        },
    }
    if result["shape"] == "cylinder":
        result["radius"] = float(item.get("radius", 0.35))
        result["height"] = float(item.get("height", 1.40))
    else:
        result["size"] = [float(value) for value in item["size"]]
    return result


PROFILES = [
    {
        "title": "basic perpendicular crossing",
        "center": (-35.0, -35.0), "heading_deg": 0.0, "length": 30.0,
        "dynamic": [{"start": (0.0, -2.5), "end": (0.0, 2.5), "speed": 0.50}],
    },
    {
        "title": "reverse-side perpendicular crossing",
        "center": (0.0, -35.0), "heading_deg": 180.0, "length": 30.0,
        "dynamic": [{"start": (0.0, 2.5), "end": (0.0, -2.5), "speed": 0.55}],
    },
    {
        "title": "fast long crossing",
        "center": (35.0, -35.0), "heading_deg": 90.0, "length": 32.0,
        "dynamic": [{"start": (0.0, -3.0), "end": (0.0, 3.0), "speed": 0.90}],
    },
    {
        "title": "slow pedestrian crossing",
        "center": (-35.0, 0.0), "heading_deg": -90.0, "length": 30.0,
        "dynamic": [{"start": (0.0, -2.2), "end": (0.0, 2.2), "speed": 0.30}],
    },
    {
        "title": "forward oblique crossing",
        "center": (0.0, 0.0), "heading_deg": 30.0, "length": 34.0,
        "dynamic": [{"start": (-1.5, -2.8), "end": (1.5, 2.8), "speed": 0.60}],
    },
    {
        "title": "backward oblique crossing",
        "center": (35.0, 0.0), "heading_deg": 150.0, "length": 34.0,
        "dynamic": [{"start": (1.8, -2.8), "end": (-1.2, 2.8), "speed": 0.55}],
    },
    {
        "title": "small agile obstacle",
        "center": (-35.0, 35.0), "heading_deg": -30.0, "length": 30.0,
        "dynamic": [{
            "start": (0.0, -2.0), "end": (0.0, 2.4), "speed": 0.75,
            "radius": 0.25, "mass": 25.0,
        }],
    },
    {
        "title": "large slow obstacle",
        "center": (0.0, 35.0), "heading_deg": -150.0, "length": 32.0,
        "dynamic": [{
            "start": (0.0, 2.8), "end": (0.0, -2.8), "speed": 0.40,
            "radius": 0.50, "mass": 65.0,
        }],
    },
    {
        "title": "moving cart crossing",
        "center": (35.0, 35.0), "heading_deg": 15.0, "length": 34.0,
        "dynamic": [{
            "start": (0.0, -2.8), "end": (0.0, 2.8), "speed": 0.50,
            "shape": "box", "size": (0.80, 0.60, 1.00), "mass": 70.0,
        }],
    },
    {
        "title": "obstacle enters and stops on route",
        "center": (-20.0, -15.0), "heading_deg": 75.0, "length": 32.0,
        "dynamic": [{
            "start": (0.0, -3.0), "end": (0.0, 0.0), "speed": 0.40,
            "stop_on_route": True,
        }],
    },
    {
        "title": "late-route crossing",
        "center": (20.0, -15.0), "heading_deg": 105.0, "length": 36.0,
        "dynamic": [{"start": (6.0, 3.0), "end": (6.0, -3.0), "speed": 0.60}],
    },
    {
        "title": "early-route crossing",
        "center": (-20.0, 15.0), "heading_deg": -75.0, "length": 36.0,
        "dynamic": [{"start": (-6.0, -3.0), "end": (-6.0, 3.0), "speed": 0.65}],
    },
    {
        "title": "two sequential opposite crossings",
        "center": (20.0, 15.0), "heading_deg": 45.0, "length": 38.0,
        "dynamic": [
            {"start": (-5.0, -2.6), "end": (-5.0, 2.6), "speed": 0.55},
            {"start": (5.0, 2.8), "end": (5.0, -2.8), "speed": 0.70},
        ],
    },
    {
        "title": "two sequential same-side crossings",
        "center": (0.0, -10.0), "heading_deg": 135.0, "length": 38.0,
        "dynamic": [
            {"start": (-4.0, -2.4), "end": (-4.0, 2.4), "speed": 0.45},
            {"start": (4.0, -3.0), "end": (4.0, 3.0), "speed": 0.80},
        ],
    },
    {
        "title": "opposing close double crossing",
        "center": (0.0, 10.0), "heading_deg": -45.0, "length": 36.0,
        "dynamic": [
            {"start": (-1.2, -2.6), "end": (-1.2, 2.6), "speed": 0.55},
            {"start": (1.2, 2.6), "end": (1.2, -2.6), "speed": 0.55},
        ],
    },
    {
        "title": "fast diagonal emergence",
        "center": (-10.0, 0.0), "heading_deg": -135.0, "length": 34.0,
        "dynamic": [{"start": (-2.0, -3.0), "end": (2.0, 3.0), "speed": 0.95}],
    },
    {
        "title": "short-notice crossing",
        "center": (10.0, 0.0), "heading_deg": 20.0, "length": 30.0,
        "dynamic": [{
            "start": (0.0, -1.6), "end": (0.0, 2.4), "speed": 0.65,
            "trigger_distance": 2.5,
        }],
    },
    {
        "title": "far-start long traversal",
        "center": (-5.0, 25.0), "heading_deg": 70.0, "length": 36.0,
        "dynamic": [{"start": (0.0, -4.0), "end": (0.0, 4.0), "speed": 0.80}],
    },
    {
        "title": "wide cart enters and stops just beyond route",
        "center": (25.0, -5.0), "heading_deg": 110.0, "length": 36.0,
        "dynamic": [{
            "start": (0.0, -3.2), "end": (0.0, 0.2), "speed": 0.35,
            "shape": "box", "size": (1.00, 0.75, 1.10), "mass": 85.0,
            "stop_on_route": True,
        }],
    },
    {
        "title": "three staged crossings",
        "center": (-25.0, 5.0), "heading_deg": 160.0, "length": 42.0,
        "dynamic": [
            {"start": (-8.0, -2.5), "end": (-8.0, 2.5), "speed": 0.60},
            {"start": (0.0, 2.7), "end": (0.0, -2.7), "speed": 0.45},
            {
                "start": (8.0, -3.0), "end": (8.0, 3.0), "speed": 0.75,
                "shape": "box", "size": (0.70, 0.55, 0.90), "mass": 55.0,
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
        # S5 intentionally isolates dynamic response. Static-obstacle quality
        # is already covered by S1-S4.
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
