import rclpy
from rclpy.node import Node
import serial
import time

class ArmSerialNode(Node):
    def __init__(self):
        super().__init__('arm_serial_node')
        self.get_logger().info("✅ ArmSerialNode started")

        try:
            self.ser = serial.Serial('/dev/arduino_arm', 9600, timeout=1)
            time.sleep(2)  # รอให้ Arduino reset
            self.get_logger().info("✅ Connected to /dev/ttyUSB1")

            # ตัวอย่าง: ส่งคำสั่ง PICK ทุก 10 วินาที
            self.create_timer(10.0, self.send_pick)

        except serial.SerialException as e:
            self.get_logger().error(f"❌ Serial error: {e}")
            self.ser = None

    def send_pick(self):
        if self.ser:
            self.ser.write(b'PICK\n')
            self.get_logger().info("📤 Sent: PICK")

def main(args=None):
    rclpy.init(args=args)
    node = ArmSerialNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
