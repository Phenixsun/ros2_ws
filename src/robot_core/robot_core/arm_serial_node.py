import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import time

class ArmSerialNode(Node):
    def __init__(self):
        super().__init__('arm_serial_node')
        self.get_logger().info("✅ ArmSerialNode started")

        try:
            self.ser = serial.Serial('/dev/arduino_arm', 9600, timeout=1)
            time.sleep(2)  # รอให้ Arduino Reset
            self.get_logger().info("✅ Connected to /dev/arduino_arm")
        except serial.SerialException as e:
            self.get_logger().error(f"❌ Serial error: {e}")
            self.ser = None

        self.subscription = self.create_subscription(
            String,
            '/arm_command',
            self.command_callback,
            10
        )

    def command_callback(self, msg):
        if self.ser:
            command = msg.data.strip() + '\n'
            self.ser.write(command.encode())
            self.get_logger().info(f"📤 Sent to Arduino: {command.strip()}")
        else:
            self.get_logger().warn("⚠️ No serial connection available.")

def main(args=None):
    rclpy.init(args=args)
    node = ArmSerialNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
