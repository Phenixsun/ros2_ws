import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import re
import time

class HuskyLensUARTNode(Node):
    def __init__(self):
        super().__init__('huskylens_uart_node')
        self.publisher_ = self.create_publisher(String, '/huskylens/objects', 10)

        self.id_to_name = {
            1: 'brake_fluid_can',
            2: 'multimeter',
            3: 'inner_tube',
            4: 'led_bulb'
        }

        self.last_sent_time = {}
        self.send_interval = 1.0  # วินาที

        try:
            self.serial_port = serial.Serial('/dev/huskylens', 9600, timeout=1)
            self.get_logger().info('✅ HuskyLens connected on /dev/huskylens')
        except serial.SerialException as e:
            self.get_logger().error(f'❌ Cannot open serial port: {e}')
            exit(1)

        self.timer = self.create_timer(0.1, self.read_from_uart)

    def read_from_uart(self):
        if self.serial_port.in_waiting > 0:
            line = self.serial_port.readline().decode('utf-8', errors='ignore').strip()
            if not line:
                return

            # ค้นหา ID และ Confidence ในข้อความ
            match = re.search(r'ID:(\d+).*?Confidence:(\d+)', line)
            if match:
                obj_id = int(match.group(1))
                confidence = int(match.group(2))
                obj_name = self.id_to_name.get(obj_id, f'unknown_id_{obj_id}')
                current_time = time.time()

                last_time = self.last_sent_time.get(obj_name, 0)
                if current_time - last_time >= self.send_interval:
                    self.last_sent_time[obj_name] = current_time

                    # ส่งชื่อวัตถุ + confidence เช่น brake_fluid_can:82
                    msg = String()
                    msg.data = f'{obj_name}:{confidence}'
                    self.publisher_.publish(msg)
                    self.get_logger().info(f'📤 Detected: {obj_name} ({confidence}%)')
            else:
                self.get_logger().warn(f'⚠️ Invalid data from HuskyLens: {line}')

def main(args=None):
    rclpy.init(args=args)
    node = HuskyLensUARTNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
