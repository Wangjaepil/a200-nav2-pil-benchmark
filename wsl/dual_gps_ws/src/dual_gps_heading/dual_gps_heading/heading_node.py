import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


# WGS84 지구 장반경 [m]
# 지금처럼 수십~수백 m 규모의 HIL에서는
# 위도/경도 -> Local ENU 근사 변환에 충분하다.
EARTH_RADIUS = 6378137.0


def wrap_angle(angle):
    """
    각도를 -pi ~ +pi 범위로 정규화한다.

    예)
      190 deg  -> -170 deg
      370 deg  ->   10 deg

    angular velocity 계산할 때
    +179도 -> -179도 경계에서 값이 튀는 것을 막는다.
    """
    return math.atan2(math.sin(angle), math.cos(angle))


class DualGpsHeading(Node):

    def __init__(self):
        super().__init__('dual_gps_heading')

        # -------------------------------------------------
        # 가장 최근에 받은 Front / Rear GPS 데이터 저장
        # -------------------------------------------------
        self.front = None
        self.rear = None

        # -------------------------------------------------
        # Local coordinate 원점
        #
        # 노드가 시작된 순간의 차량 중심 GPS 위치를
        # x=0, y=0, z=0 으로 사용한다.
        # -------------------------------------------------
        self.origin_lat = None
        self.origin_lon = None
        self.origin_alt = None

        # -------------------------------------------------
        # 속도 계산을 위한 이전 값
        # -------------------------------------------------
        self.prev_time = None
        self.prev_x = None
        self.prev_y = None
        self.prev_yaw = None

        # -------------------------------------------------
        # GPS Subscriber
        #
        # Gazebo:
        #   /gps/front
        #   /gps/rear
        #
        # ros_gz_bridge를 통해 NavSatFix로 들어온다.
        # -------------------------------------------------
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

        # -------------------------------------------------
        # 사람이 확인하기 위한 heading publisher
        # -------------------------------------------------

        # ROS ENU yaw [rad]
        self.yaw_pub = self.create_publisher(
            Float64,
            '/dual_gps/yaw',
            10
        )

        # Compass heading [deg]
        # North = 0
        # East  = 90
        self.heading_pub = self.create_publisher(
            Float64,
            '/dual_gps/heading_deg',
            10
        )

        # -------------------------------------------------
        # Nav2에서 사용할 Odometry
        # -------------------------------------------------
        self.odom_pub = self.create_publisher(
            Odometry,
            '/gps/odom',
            10
        )

        # -------------------------------------------------
        # TF broadcaster
        #
        # odom
        #   ↓
        # base_link
        #
        # 를 계속 publish한다.
        # -------------------------------------------------
        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info(
            'Dual GPS odometry node started'
        )

    # =====================================================
    # Front GPS callback
    # =====================================================

    def front_callback(self, msg):

        self.front = msg

        # Rear GPS 데이터까지 한 번 이상 들어왔으면
        # 위치 / heading 계산 시작
        if self.rear is not None:
            self.calculate()

    # =====================================================
    # Rear GPS callback
    # =====================================================

    def rear_callback(self, msg):

        self.rear = msg

    # =====================================================
    # Dual GPS 계산
    # =====================================================

    def calculate(self):

        # GPS 값이 정상적인 숫자인지 확인
        if not all([
            math.isfinite(self.front.latitude),
            math.isfinite(self.front.longitude),
            math.isfinite(self.rear.latitude),
            math.isfinite(self.rear.longitude)
        ]):
            return

        # -------------------------------------------------
        # 1. GPS 위도/경도
        #
        # NavSatFix:
        # latitude / longitude는 degree이므로
        # 계산을 위해 radian으로 변환한다.
        # -------------------------------------------------

        front_lat = math.radians(
            self.front.latitude
        )

        front_lon = math.radians(
            self.front.longitude
        )

        rear_lat = math.radians(
            self.rear.latitude
        )

        rear_lon = math.radians(
            self.rear.longitude
        )

        # -------------------------------------------------
        # 2. 차량 중심 GPS 위치
        #
        # GPS Front와 Rear의 중점을
        # 차량 base_link의 위치라고 가정한다.
        #
        #
        # GPS Front ●
        #           |
        #           | 0.6 m
        #           |
        # GPS Rear  ●
        #
        #      중간점 = 차량 중심
        # -------------------------------------------------

        center_lat = (
            front_lat + rear_lat
        ) / 2.0

        center_lon = (
            front_lon + rear_lon
        ) / 2.0

        center_alt = (
            self.front.altitude
            + self.rear.altitude
        ) / 2.0

        # -------------------------------------------------
        # 3. 최초 GPS 위치를 odom 원점으로 설정
        #
        # 프로그램 시작 위치:
        #
        # x = 0
        # y = 0
        #
        # -------------------------------------------------

        if self.origin_lat is None:

            self.origin_lat = center_lat
            self.origin_lon = center_lon
            self.origin_alt = center_alt

            self.get_logger().info(
                'GPS origin initialized: x=0, y=0'
            )

        # -------------------------------------------------
        # 4. 위도/경도 → Local ENU 좌표
        #
        # x = East
        # y = North
        #
        #
        #              North +Y
        #                  ↑
        #                  |
        #                  |
        #                  +──────→ East +X
        #
        # 수십~수백 m 정도 이동하는 테스트에서는
        # 이 local tangent-plane 근사면 충분하다.
        # -------------------------------------------------

        x = (
            EARTH_RADIUS
            * (center_lon - self.origin_lon)
            * math.cos(self.origin_lat)
        )

        y = (
            EARTH_RADIUS
            * (center_lat - self.origin_lat)
        )

        z = center_alt - self.origin_alt

        # -------------------------------------------------
        # 5. Rear GPS → Front GPS 벡터 계산
        #
        # 바로 이것으로 차량 heading을 구한다.
        #
        #              Front ●
        #                   ↑
        #                   | 차량 방향
        #                   |
        #               Rear ●
        #
        # -------------------------------------------------

        mean_lat = (
            front_lat + rear_lat
        ) / 2.0

        baseline_east = (
            EARTH_RADIUS
            * (front_lon - rear_lon)
            * math.cos(mean_lat)
        )

        baseline_north = (
            EARTH_RADIUS
            * (front_lat - rear_lat)
        )

        # GPS 두 개 사이 거리
        baseline = math.hypot(
            baseline_east,
            baseline_north
        )

        # 두 GPS 좌표가 거의 같으면
        # heading 계산 불가능
        if baseline < 0.01:
            self.get_logger().warning(
                'GPS baseline too small'
            )
            return

        # -------------------------------------------------
        # 6. ROS ENU yaw 계산
        #
        # ROS:
        #
        # East  = 0 rad
        # North = +pi/2
        # West  = +-pi
        # South = -pi/2
        #
        # -------------------------------------------------

        yaw = math.atan2(
            baseline_north,
            baseline_east
        )

        # -------------------------------------------------
        # 사람이 보기 편한 Compass heading
        #
        # North =   0 deg
        # East  =  90 deg
        # South = 180 deg
        # West  = 270 deg
        # -------------------------------------------------

        heading_deg = (
            math.degrees(
                math.atan2(
                    baseline_east,
                    baseline_north
                )
            )
            + 360.0
        ) % 360.0

        # -------------------------------------------------
        # 7. Yaw → Quaternion
        #
        # 차량은 평면 주행한다고 가정:
        #
        # roll  = 0
        # pitch = 0
        # yaw   = GPS heading
        #
        # 따라서 quaternion에서
        # z, w만 필요하다.
        # -------------------------------------------------

        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)

        # -------------------------------------------------
        # 8. 속도 추정
        #
        # GPS 위치 변화량 / 시간
        #
        # GPS 자체 위치만으로 속도를 근사한다.
        # -------------------------------------------------

        now = self.get_clock().now()

        current_time = (
            now.nanoseconds / 1e9
        )

        linear_x = 0.0
        linear_y = 0.0
        angular_z = 0.0

        if (
            self.prev_time is not None
            and self.prev_x is not None
            and self.prev_y is not None
            and self.prev_yaw is not None
        ):

            dt = (
                current_time
                - self.prev_time
            )

            if dt > 0.001:

                # -----------------------------
                # odom/world 좌표계 속도
                # -----------------------------

                vx_world = (
                    x - self.prev_x
                ) / dt

                vy_world = (
                    y - self.prev_y
                ) / dt

                # -----------------------------
                # world velocity를
                # base_link 좌표계로 회전
                #
                # Odometry.twist는
                # child_frame_id 기준으로 표현
                # -----------------------------

                linear_x = (
                    math.cos(yaw) * vx_world
                    + math.sin(yaw) * vy_world
                )

                linear_y = (
                    -math.sin(yaw) * vx_world
                    + math.cos(yaw) * vy_world
                )

                # -----------------------------
                # yaw 변화량으로 angular velocity
                # -----------------------------

                yaw_delta = wrap_angle(
                    yaw - self.prev_yaw
                )

                angular_z = (
                    yaw_delta / dt
                )

        # 현재값 저장
        self.prev_time = current_time
        self.prev_x = x
        self.prev_y = y
        self.prev_yaw = yaw

        # -------------------------------------------------
        # 9. /dual_gps/yaw publish
        # -------------------------------------------------

        yaw_msg = Float64()

        yaw_msg.data = yaw

        self.yaw_pub.publish(
            yaw_msg
        )

        # -------------------------------------------------
        # 10. /dual_gps/heading_deg publish
        # -------------------------------------------------

        heading_msg = Float64()

        heading_msg.data = heading_deg

        self.heading_pub.publish(
            heading_msg
        )

        # -------------------------------------------------
        # 11. /gps/odom publish
        #
        # odom
        #   ↓
        # base_link
        #
        # pose:
        #   GPS 위치 + Dual GPS yaw
        #
        # twist:
        #   GPS 위치 변화로 계산
        # -------------------------------------------------

        odom = Odometry()

        odom.header.stamp = now.to_msg()

        odom.header.frame_id = 'odom'

        odom.child_frame_id = 'base_link'

        # Position
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = z

        # Orientation
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        # Velocity
        odom.twist.twist.linear.x = linear_x
        odom.twist.twist.linear.y = linear_y
        odom.twist.twist.linear.z = 0.0

        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = angular_z

        self.odom_pub.publish(
            odom
        )

        # -------------------------------------------------
        # 12. TF publish
        #
        #        odom
        #          |
        #          ↓
        #      base_link
        #
        # Nav2가 로봇의 현재 위치를 알 수 있게 한다.
        # -------------------------------------------------

        transform = TransformStamped()

        transform.header.stamp = now.to_msg()

        transform.header.frame_id = 'odom'

        transform.child_frame_id = 'base_link'

        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.translation.z = z

        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(
            transform
        )

        # -------------------------------------------------
        # 13. 상태 로그
        # -------------------------------------------------

        self.get_logger().info(
            f'x={x:.2f} m, '
            f'y={y:.2f} m, '
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