import rclpy
from rclpy.node import Node

from tf2_msgs.msg import TFMessage

from rclpy.qos import (
    QoSProfile,
    QoSDurabilityPolicy,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
)


class TfRelay(Node):

    def __init__(self):
        super().__init__('clearpath_tf_relay')

        # ============================================================
        # Dynamic TF QoS
        #
        # Clearpath:
        #   /a200_0000/tf
        #
        #          ↓ relay
        #
        # Global:
        #   /tf
        # ============================================================

        dynamic_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=100,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        # ============================================================
        # Static TF QoS
        #
        # /tf_static은 나중에 들어온 subscriber도
        # 기존 static transform을 받을 수 있어야 하므로
        # TRANSIENT_LOCAL 사용
        # ============================================================

        static_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )

        # ----------------------------
        # Global TF publishers
        # ----------------------------

        self.tf_pub = self.create_publisher(
            TFMessage,
            '/tf',
            dynamic_qos
        )

        self.tf_static_pub = self.create_publisher(
            TFMessage,
            '/tf_static',
            static_qos
        )

        # ----------------------------
        # Clearpath TF subscribers
        # ----------------------------

        self.tf_sub = self.create_subscription(
            TFMessage,
            '/a200_0000/tf',
            self.tf_callback,
            dynamic_qos
        )

        self.tf_static_sub = self.create_subscription(
            TFMessage,
            '/a200_0000/tf_static',
            self.tf_static_callback,
            static_qos
        )

        self.get_logger().info(
            'Clearpath TF relay started'
        )

        self.get_logger().info(
            '/a200_0000/tf -> /tf'
        )

        self.get_logger().info(
            '/a200_0000/tf_static -> /tf_static'
        )

    # ================================================================
    # Dynamic TF relay
    # ================================================================

    def tf_callback(self, msg):

        self.tf_pub.publish(msg)

    # ================================================================
    # Static TF relay
    # ================================================================

    def tf_static_callback(self, msg):

        self.tf_static_pub.publish(msg)


def main(args=None):

    rclpy.init(args=args)

    node = TfRelay()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()