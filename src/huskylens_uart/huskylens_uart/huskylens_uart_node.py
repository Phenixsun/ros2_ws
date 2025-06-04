import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

class HuskyLensUARTNode(Node):
    def __init__(self):
        super().__init__('huskylens_uart_node')
        self.publisher_ = self.create_publisher(String, 'huskylens_data', 10)
        
        try:
            self.serial_port = serial.Serial('/dev/huskylens', 9600, timeout=1)
            self.get_logger().info('HuskyLens connected on /dev/huskylens')
        except serial.SerialException as e:
            self.get_logger().error(f'Cannot open serial port: {e}')
            exit(1)

        self.timer = self.create_timer(0.1, self.read_from_uart)

    def read_from_uart(self):
        if self.serial_port.in_waiting > 0:
            line = self.serial_port.readline().decode('utf-8', errors='ignore').strip()
            if line:
                msg = String()
                msg.data = line
                self.publisher_.publish(msg)
                self.get_logger().info(f'HuskyLens: {line}')

def main(args=None):
    rclpy.init(args=args)
    node = HuskyLensUARTNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
