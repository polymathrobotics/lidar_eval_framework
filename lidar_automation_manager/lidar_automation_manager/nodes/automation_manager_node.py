# Copyright (c) 2025-present Polymath Robotics, Inc.
# SPDX-License-Identifier: Apache-2.0

import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Int32
from std_srvs.srv import SetBool, Trigger

from lidar_automation_manager.tools.driver_manager import DriverManager
from lidar_automation_manager.tools.test_case_handler import TestCaseHandler


class AutomationManagerNode(Node):

    def __init__(self):
        super().__init__('lidar_automation_manager')

        self.test_case_handler = TestCaseHandler(self)
        self.driver_manager = DriverManager(self)
        self.declare_parameter('lidar', '')
        self.declare_parameter('ros2_driver', '')
        self.declare_parameter('driver_command', '')
        self.declare_parameter('driver_config_file', '')
        # '' disables, 'prompt' enables manual entry, any other value is a GUI to launch.
        self.declare_parameter('lidar_gui', '')
        self.declare_parameter('bag_recorder_directory', '')
        self.declare_parameter('bench_initiate_service', '~/lidar_test_bench_initiate')
        self.declare_parameter('start_evaluation_service', '/start_evaluation')
        self.declare_parameter('servo_angle_topic', '/servo_angle')
        self.declare_parameter('bag_recording_duration', 120)
        self.declare_parameter('pointcloud_topic', '')
        self.declare_parameter('angles', Parameter.Type.DOUBLE_ARRAY)
        # Set by polysetup-angle-detection. Off by default: an angle sweep is a bag per
        # angle, so it is opted into. `angles` stays populated either way, so flipping
        # this back on needs no reconfigure.
        self.declare_parameter('angle_detection_enabled', False)
        self.declare_parameter('parameter_names', Parameter.Type.STRING_ARRAY)

        self.lidar = self.get_parameter('lidar').get_parameter_value().string_value
        self.ros2_driver = self.get_parameter('ros2_driver').get_parameter_value().string_value
        self.driver_command = self.get_parameter('driver_command').get_parameter_value().string_value
        self.driver_config_file = self.get_parameter('driver_config_file').get_parameter_value().string_value
        self.lidar_gui = self.get_parameter('lidar_gui').get_parameter_value().string_value
        self.bag_recorder_directory = self.get_parameter('bag_recorder_directory').get_parameter_value().string_value
        bench_initiate_service = self.get_parameter('bench_initiate_service').get_parameter_value().string_value
        start_evaluation_service = self.get_parameter('start_evaluation_service').get_parameter_value().string_value
        servo_angle_topic = self.get_parameter('servo_angle_topic').get_parameter_value().string_value
        self.bag_recording_duration = self.get_parameter('bag_recording_duration').get_parameter_value().integer_value
        self.pointcloud_topic = self.get_parameter('pointcloud_topic').get_parameter_value().string_value
        self.angles = list(self.get_parameter('angles').get_parameter_value().double_array_value)
        self.angle_detection_enabled = self.get_parameter(
            'angle_detection_enabled').get_parameter_value().bool_value
        self.parameter_names = list(self.get_parameter('parameter_names').get_parameter_value().string_array_value)

        self.defaults = {}
        self._first_angle_seen = False

        self._lidar_test_bench_srv = self.create_service(Trigger, bench_initiate_service, self._on_bench_trigger)
        self._start_evaluation_client = self.create_client(SetBool, start_evaluation_service)
        self._servo_angle_pub = self.create_publisher(Int32, servo_angle_topic, 10)

        self.load_test_cases(self.angles, self.parameter_names)
        self._log_config()


    def _on_bench_trigger(self, request, response):

        self.get_logger().info('Received initiation trigger to start the lidar test bench')

        self.initiate_testing()

        response.success = True
        response.message = 'automation started'
        return response


    def initiate_testing(self):

        self.driver_manager.kill_driver_processes()
        time.sleep(5)
        self.driver_manager.set_driver_config_to_default()
        self.driver_manager.record_bag()

        while (case := self.test_case_handler.next_test_case()) is not None:

            self.get_logger().info(f'Executing test case: {case}')

            self.execute_test_case(case)

        self.get_logger().info('All test cases have been executed')
        self.driver_manager.kill_driver_processes()
        self._start_evaluation_client.call_async(SetBool.Request(data=True))
        return


    def set_angle(self, case):

        if 'angle' == case.test_type:
            self._servo_angle_pub.publish(Int32(data=int(case.angle)))
        else:
            self._servo_angle_pub.publish(Int32(data=0))
        time.sleep(10)


    def execute_test_case(self, case):

        self.driver_manager.kill_driver_processes()
        time.sleep(5)
        if 'angle' == case.test_type and not self._first_angle_seen:
            self.driver_manager.prompt_and_record_gui_params(self.lidar_gui)
            self._first_angle_seen = True

        self.set_angle(case)
        self.driver_manager.set_driver_config_to_default()
        if 'angle' != case.test_type:
            self.driver_manager.set_driver_param(case)
        self.driver_manager.record_bag(case)


    def load_test_cases(self, angles, parameter_names):

        self.test_case_handler.load_test_cases(
            angles if self.angle_detection_enabled else [], parameter_names)

        for name in parameter_names:
            self.defaults[name] = (
                self.get_parameter(f'{name}.path').value,
                self.get_parameter(f'{name}.default').value,
            )


    def _log_config(self):
        log = self.get_logger()
        log.info(f'lidar: {self.lidar}')
        log.info(f'ros2_driver: {self.ros2_driver}')
        log.info(f'driver_command: {self.driver_command}')
        log.info(f'angles: {self.angles}')
        log.info(f'test_cases: {self.test_case_handler.test_cases}')


def main(args=None):
    rclpy.init(args=args)
    node = AutomationManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
