import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

from arduino_eth_bridge.tcp_tools import TcpClient


class ArduinoEthBridge(Node):

    def __init__(self):
        super().__init__("arduino_eth_bridge")

        self.declare_parameter("arduino_ip", "192.168.1.177")
        self.declare_parameter("arduino_port", 5000)
        self.declare_parameter("command_topic", "/servo_angle")

        ip = self.get_parameter("arduino_ip").value
        port = self.get_parameter("arduino_port").value
        command_topic = self.get_parameter("command_topic").value

        self.client = TcpClient(ip, port)

        self.subscription = self.create_subscription(Int32, command_topic, self.callback, 10)

    def callback(self, msg):
        try:
            self.client.send_command(msg.data)
        except Exception as exc:
            self.get_logger().error(f"Failed to send command to Arduino: {exc}")

    def destroy_node(self):
        self.client.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArduinoEthBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
