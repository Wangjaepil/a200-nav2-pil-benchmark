#!/usr/bin/env python3
"""Publish one validated benchmark mission goal.

The world->map transform is anchored to the map pose captured by the runner
before Nav2 activation. The goal is rejected if the robot moved away from that
anchor before publication.
"""

from __future__ import annotations

import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

from benchmark_common import (
    Pose2D,
    SUITE_VERSION,
    load_case_spec,
    pose_error,
    quaternion_to_yaw,
    transform_world_goal_to_map,
    yaw_to_quaternion_zw,
)


MAX_PRE_GOAL_DRIFT_M = 0.05
MAX_PRE_GOAL_YAW_DRIFT_RAD = math.radians(3.0)
PUBLISH_SETTLE_SEC = 1.0


def _split_application_and_ros_args(argv: list[str]) -> tuple[list[str], list[str]]:
    if "--ros-args" not in argv:
        return argv, []
    index = argv.index("--ros-args")
    return argv[:index], argv[index:]


def parse_args(argv: list[str]) -> tuple[argparse.Namespace, list[str]]:
    app_args, ros_args = _split_application_and_ros_args(argv)
    parser = argparse.ArgumentParser(
        description="Validate and publish one /far_goal_pose benchmark goal."
    )
    parser.add_argument("case_id")
    parser.add_argument("--map-start-x", type=float, required=True)
    parser.add_argument("--map-start-y", type=float, required=True)
    parser.add_argument("--map-start-yaw", type=float, required=True)
    parser.add_argument("--min-subscribers", type=int, default=2)
    parser.add_argument("--timeout-sec", type=float, default=30.0)
    return parser.parse_args(app_args), ros_args


class CaseGoalPublisher(Node):
    def __init__(self, args: argparse.Namespace):
        super().__init__("case_goal_publisher")
        self.case = load_case_spec(args.case_id)
        self.map_start = Pose2D(
            float(args.map_start_x),
            float(args.map_start_y),
            float(args.map_start_yaw),
        )
        self.min_subscribers = max(1, int(args.min_subscribers))
        self.timeout_sec = max(1.0, float(args.timeout_sec))
        self.started_wall = time.monotonic()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pub = self.create_publisher(PoseStamped, "/far_goal_pose", 10)

        self.sent = False
        self.failed = False
        self.failure_reason = ""
        self.timer = self.create_timer(0.2, self.try_send)

        self.get_logger().info(
            f"Goal sender ready: suite={SUITE_VERSION}, case={self.case.case_id}, "
            f"required_subscribers={self.min_subscribers}"
        )

    def _current_map_pose(self) -> Pose2D | None:
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

    def _fail(self, message: str) -> None:
        self.failed = True
        self.failure_reason = message
        self.timer.cancel()
        self.get_logger().error(message)

    def try_send(self) -> None:
        if self.sent or self.failed:
            return

        elapsed = time.monotonic() - self.started_wall
        if elapsed >= self.timeout_sec:
            self._fail(
                "Goal sender timed out waiting for TF/subscribers within "
                f"{self.timeout_sec:.1f}s"
            )
            return

        if self.pub.get_subscription_count() < self.min_subscribers:
            return

        current = self._current_map_pose()
        if current is None:
            return

        drift_m, yaw_drift_rad = pose_error(current, self.map_start)
        if (
            drift_m > MAX_PRE_GOAL_DRIFT_M
            or yaw_drift_rad > MAX_PRE_GOAL_YAW_DRIFT_RAD
        ):
            self._fail(
                "Robot moved before goal publication: "
                f"xy_drift={drift_m:.3f}m "
                f"yaw_drift={math.degrees(yaw_drift_rad):.2f}deg; "
                f"limits={MAX_PRE_GOAL_DRIFT_M:.3f}m/"
                f"{math.degrees(MAX_PRE_GOAL_YAW_DRIFT_RAD):.1f}deg"
            )
            return

        goal_map = transform_world_goal_to_map(
            self.case.robot,
            self.case.goal,
            self.map_start,
        )
        z, w = yaw_to_quaternion_zw(goal_map.yaw)

        message = PoseStamped()
        message.header.frame_id = "map"
        message.header.stamp = self.get_clock().now().to_msg()
        message.pose.position.x = goal_map.x
        message.pose.position.y = goal_map.y
        message.pose.orientation.z = z
        message.pose.orientation.w = w

        print()
        print("======================================")
        print(f"CASE        : {self.case.case_id}")
        print(
            f"World Start : ({self.case.robot.x:.3f}, "
            f"{self.case.robot.y:.3f}, {self.case.robot.yaw:.4f})"
        )
        print(
            f"World Goal  : ({self.case.goal.x:.3f}, "
            f"{self.case.goal.y:.3f}, {self.case.goal.yaw:.4f})"
        )
        print(
            f"Map Anchor  : ({self.map_start.x:.3f}, "
            f"{self.map_start.y:.3f}, {self.map_start.yaw:.4f})"
        )
        print(
            f"Map Current : ({current.x:.3f}, "
            f"{current.y:.3f}, {current.yaw:.4f})"
        )
        print(
            f"Map Goal    : ({goal_map.x:.3f}, "
            f"{goal_map.y:.3f}, {goal_map.yaw:.4f})"
        )
        print(f"Direct dist : {self.case.direct_distance_m:.3f} m")
        print("======================================")
        print()

        self.pub.publish(message)
        self.sent = True
        self.timer.cancel()
        self.get_logger().info("Goal published to /far_goal_pose")


def main() -> int:
    args, ros_args = parse_args(sys.argv[1:])
    rclpy.init(args=ros_args)
    node = CaseGoalPublisher(args)

    try:
        while rclpy.ok() and not node.sent and not node.failed:
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.sent:
            # Give the reliable publisher a short delivery window before the
            # one-shot node destroys its DDS entities. The runner separately
            # confirms that the benchmark logger received this exact goal.
            deadline = time.monotonic() + PUBLISH_SETTLE_SEC
            while rclpy.ok() and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        return 130
    finally:
        failed = node.failed
        failure_reason = node.failure_reason
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    if failed:
        print(f"GOAL_SEND_FAILED: {failure_reason}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
