#!/usr/bin/env python3
"""ROS-independent regression tests for benchmark suite 2.0."""

from __future__ import annotations

import csv
import importlib.util
import math
import sys
import tempfile
import unittest
from pathlib import Path

from benchmark_common import (
    Pose2D,
    load_case_spec,
    make_run_dir,
    navigation_terminal_policy,
    pose_error,
    transform_world_goal_to_map,
)
from benchmark_path_tools import (
    derive_world_to_map,
    is_canonical_global_plan,
    load_plans,
)


class CommonTests(unittest.TestCase):
    def test_run_numbers_never_reuse_a_deleted_hole(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            (root / "run_001").mkdir()
            (root / "run_003").mkdir()
            self.assertEqual(make_run_dir(root).name, "run_004")

    def test_goal_transform_uses_fixed_map_start(self):
        world_start = Pose2D(0.0, 0.0, 0.0)
        world_goal = Pose2D(10.0, 0.0, math.pi / 2.0)
        map_start = Pose2D(2.0, 3.0, math.pi / 2.0)
        converted = transform_world_goal_to_map(
            world_start, world_goal, map_start
        )
        self.assertAlmostEqual(converted.x, 2.0, places=6)
        self.assertAlmostEqual(converted.y, 13.0, places=6)
        self.assertAlmostEqual(abs(converted.yaw), math.pi, places=6)

    def test_pose_error_detects_pre_goal_motion(self):
        xy, yaw = pose_error(
            Pose2D(0.06, 0.0, math.radians(4.0)),
            Pose2D(0.0, 0.0, 0.0),
        )
        self.assertGreater(xy, 0.05)
        self.assertGreater(math.degrees(yaw), 3.0)

    def test_case_yaml_is_validated(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            case_dir = root / "S1"
            case_dir.mkdir()
            (case_dir / "S1_01.yaml").write_text(
                "robot: {x: 1.0, y: 2.0, yaw: 0.0}\n"
                "goal: {x: 4.0, y: 6.0, yaw: 1.0}\n",
                encoding="utf-8",
            )
            case = load_case_spec("s1_01", root)
            self.assertEqual(case.case_id, "S1_01")
            self.assertAlmostEqual(case.direct_distance_m, 5.0)


class MissionPolicyTests(unittest.TestCase):
    def test_intermediate_abort_waits_for_replacement(self):
        policy = navigation_terminal_policy("ABORTED", 12.0)
        self.assertTrue(policy.is_intermediate)
        self.assertEqual(policy.wait_sec, 12.0)
        self.assertEqual(policy.timeout_result, "ABORTED")

    def test_intermediate_success_without_next_goal_is_stalled(self):
        policy = navigation_terminal_policy("SUCCEEDED", 12.0)
        self.assertTrue(policy.is_intermediate)
        self.assertEqual(policy.timeout_result, "MISSION_STALLED")

    def test_terminal_near_final_goal_uses_short_grace(self):
        policy = navigation_terminal_policy("SUCCEEDED", 0.1)
        self.assertFalse(policy.is_intermediate)
        self.assertEqual(policy.wait_sec, 3.0)
        self.assertEqual(policy.timeout_result, "SUCCEEDED")


class PathTests(unittest.TestCase):
    def test_legacy_map_plan_is_accepted_but_odom_plan_is_not(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            path = root / "planned_paths.csv"
            with path.open("w", newline="", encoding="utf-8") as handle:
                writer = csv.writer(handle)
                writer.writerow([
                    "time_sec", "plan_id", "nav_goal_index", "frame_id",
                    "pose_index", "x", "y", "yaw_rad",
                ])
                writer.writerow([1.0, 1, 1, "map", 0, 0.0, 0.0, 0.0])
                writer.writerow([1.0, 1, 1, "map", 1, 1.0, 0.0, 0.0])
                writer.writerow([2.0, 2, 1, "odom", 0, 0.0, 0.0, 0.0])
            plans = load_plans(root)
            accepted = [plan for plan in plans if is_canonical_global_plan(plan)]
            self.assertEqual(len(plans), 2)
            self.assertEqual(len(accepted), 1)
            self.assertEqual(accepted[0].frame_id, "map")

    def test_requested_goal_recovers_map_anchor(self):
        case = {
            "robot": {"x": 0.0, "y": 0.0, "yaw": 0.0},
            "goal": {"x": 10.0, "y": 0.0, "yaw": math.pi / 2.0},
        }
        requested = Pose2D(2.0, 13.0, math.pi)
        transform = derive_world_to_map(
            case, [], requested_goal=requested
        )
        self.assertIsNotNone(transform)
        self.assertAlmostEqual(transform.map_start.x, 2.0, places=6)
        self.assertAlmostEqual(transform.map_start.y, 3.0, places=6)
        self.assertAlmostEqual(transform.map_start.yaw, math.pi / 2.0, places=6)


def _load_uploaded_module(module_name: str, candidates: tuple[str, ...]):
    root = Path(__file__).resolve().parent
    for candidate in candidates:
        path = root / candidate
        if path.exists():
            spec = importlib.util.spec_from_file_location(module_name, path)
            module = importlib.util.module_from_spec(spec)
            sys.modules[module_name] = module
            spec.loader.exec_module(module)
            return module
    raise FileNotFoundError(candidates)


class RunnerAndGuiTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.support = _load_uploaded_module(
            "benchmark_runner_support",
            ("benchmark_runner_support.py",),
        )
        cls.runner = _load_uploaded_module(
            "benchmark_runner_tested",
            ("benchmark_runner.py", "benchmark_runner(1).py"),
        )
        _load_uploaded_module(
            "ui_theme",
            ("ui_theme.py", "ui_theme(1).py"),
        )
        cls.gui_module = _load_uploaded_module(
            "benchmark_qa_gui_tested",
            ("benchmark_qa_gui.py", "benchmark_qa_gui(1).py"),
        )

    def test_tf2_echo_pose_parser(self):
        output = (
            "- Translation: [1.250, -2.500, 0.000]\n"
            "- Rotation: in RPY (radian) [0.000, 0.000, 1.570]\n"
        )
        pose = self.support._parse_tf2_echo_pose(output)
        self.assertIsNotNone(pose)
        self.assertAlmostEqual(pose.x, 1.25)
        self.assertAlmostEqual(pose.y, -2.5)
        self.assertAlmostEqual(pose.yaw, 1.57)

    def test_sender_command_carries_anchor_and_sim_time(self):
        command = self.support.sender_command("S1_01", Pose2D(1.0, 2.0, 0.5))
        joined = " ".join(command)
        self.assertIn("--map-start-x 1.000000000", joined)
        self.assertIn("--map-start-y 2.000000000", joined)
        self.assertIn("--map-start-yaw 0.500000000", joined)
        self.assertIn("use_sim_time:=true", joined)

    def test_latest_diagnosed_run_is_visible_without_summary(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            case_dir = Path(temp_dir) / "S1_01"
            old_run = case_dir / "run_001"
            new_run = case_dir / "run_002"
            old_run.mkdir(parents=True)
            new_run.mkdir()
            (old_run / "summary.yaml").write_text(
                "benchmark_result: PASS\n", encoding="utf-8"
            )
            (new_run / "runner_status.yaml").write_text(
                "runner_result: INFRA_ERROR\n"
                "failure_reason: costmap timeout\n",
                encoding="utf-8",
            )
            gui = object.__new__(self.gui_module.BenchmarkQAGui)
            run_dir, payload = gui._latest_result_payload(case_dir)
            self.assertEqual(run_dir.name, "run_002")
            self.assertEqual(payload["benchmark_result"], "INFRA_ERROR")
            self.assertEqual(
                payload["benchmark_result_reason"], "costmap timeout"
            )


if __name__ == "__main__":
    unittest.main()
