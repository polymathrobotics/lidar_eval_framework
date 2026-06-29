# Copyright (c) 2025-present Polymath Robotics, Inc. All rights reserved
# Proprietary. Any unauthorized copying, distribution, or modification of this software is strictly prohibited.

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32


class LidarBenchTFBroadcaster(Node):
    def __init__(self):
        super().__init__('lidar_bench_tf_broadcaster')

        self._current_angle_rad = 0.0
        self._joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.create_subscription(Int32, '/servo_angle', self._on_servo_angle, 10)
        self.create_timer(0.1, self._publish_joint_state_periodic)

    def _publish_joint_state_periodic(self) -> None:
        self._publish_joint_state(self._current_angle_rad)

    def _publish_joint_state(self, angle_rad: float) -> None:
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = ['motor_joint']
        js.position = [angle_rad]
        self._joint_state_pub.publish(js)

    def _on_servo_angle(self, msg: Int32) -> None:
        self._current_angle_rad = math.radians(-msg.data)
        self._publish_joint_state(self._current_angle_rad)
        self.get_logger().info(f'motor_joint updated: {msg.data} deg')


def main(args=None):
    rclpy.init(args=args)
    node = LidarBenchTFBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
