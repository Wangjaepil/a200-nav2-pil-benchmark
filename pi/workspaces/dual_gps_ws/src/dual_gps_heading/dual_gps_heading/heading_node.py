import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import NavSatFix, Imu
from std_msgs.msg import Float64


EARTH_RADIUS = 6378137.0


class DualGpsHeading(Node):

    def __init__(self):
        super().__init__('dual_gps_heading')

        self.front = None
        self.rear = None

        # Dual GPS
        self.front_sub = self.create_subscription(
            NavSatFix,
            '/gps/front',
            self.front_callback,
            qos_profile_sensor_data
        )

        self.rear_sub = self.create_subscription(
            NavSatFix,
            '/gps/rear',
            self.rear_callback,
            qos_profile_sensor_data
        )

        # 사람이 확인하기 위한 yaw [rad]
        self.yaw_pub = self.create_publisher(
            Float64,
            '/dual_gps/yaw',
            10
        )

        # Compass heading [deg]
        # North = 0, East = 90
        self.heading_pub = self.create_publisher(
            Float64,
            '/dual_gps/heading_deg',
            10
        )

        # navsat_transform_node에서 사용할 heading
        self.imu_pub = self.create_publisher(
            Imu,
            '/gps/heading',
            10
        )

        self.get_logger().info(
            'Dual GPS heading node started'
        )

    def front_callback(self, msg):

        self.front = msg

        if self.rear is not None:
            self.calculate_heading()

    def rear_callback(self, msg):

        self.rear = msg

    def calculate_heading(self):

        if not all([
            math.isfinite(self.front.latitude),
            math.isfinite(self.front.longitude),
            math.isfinite(self.rear.latitude),
            math.isfinite(self.rear.longitude)
        ]):
            return

        front_lat = math.radians(self.front.latitude)
        front_lon = math.radians(self.front.longitude)

        rear_lat = math.radians(self.rear.latitude)
        rear_lon = math.radians(self.rear.longitude)

        mean_lat = (front_lat + rear_lat) / 2.0

        # Rear -> Front 벡터
        baseline_east = (
            EARTH_RADIUS
            * (front_lon - rear_lon)
            * math.cos(mean_lat)
        )

        baseline_north = (
            EARTH_RADIUS
            * (front_lat - rear_lat)
        )

        baseline = math.hypot(
            baseline_east,
            baseline_north
        )

        if baseline < 0.01:
            self.get_logger().warning(
                'GPS baseline too small'
            )
            return

        # ROS ENU yaw
        # East = 0 rad
        # North = +pi/2
        yaw = math.atan2(
            baseline_north,
            baseline_east
        )

        # Compass heading
        # North = 0 deg
        # East = 90 deg
        heading_deg = (
            math.degrees(
                math.atan2(
                    baseline_east,
                    baseline_north
                )
            )
            + 360.0
        ) % 360.0

        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)

        # ---------------------------------
        # 확인용 yaw
        # ---------------------------------

        yaw_msg = Float64()
        yaw_msg.data = yaw
        self.yaw_pub.publish(yaw_msg)

        heading_msg = Float64()
        heading_msg.data = heading_deg
        self.heading_pub.publish(heading_msg)

        # ---------------------------------
        # navsat_transform용 IMU
        # ---------------------------------

        imu = Imu()

        imu.header.stamp = self.get_clock().now().to_msg()
        imu.header.frame_id = 'base_link'

        imu.orientation.x = 0.0
        imu.orientation.y = 0.0
        imu.orientation.z = qz
        imu.orientation.w = qw

        # 시뮬레이션 heading
        # roll/pitch는 사용하지 않고 yaw를 신뢰
        imu.orientation_covariance = [
            999999.0, 0.0,      0.0,
            0.0,      999999.0, 0.0,
            0.0,      0.0,      0.01
        ]

        self.imu_pub.publish(imu)

        self.get_logger().info(
            f'heading={heading_deg:.1f} deg, '
            f'yaw={math.degrees(yaw):.1f} deg, '
            f'baseline={baseline:.3f} m',
            throttle_duration_sec=1.0
        )


def main(args=None):

    rclpy.init(args=args)

    node = DualGpsHeading()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()