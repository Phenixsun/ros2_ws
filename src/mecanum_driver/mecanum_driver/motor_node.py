#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import gpiod
from geometry_msgs.msg import Twist
import math

class MecanumMotorNode(Node):
    def __init__(self):
        super().__init__('mecanum_motor_node')

        self.chip = gpiod.Chip('gpiochip4')  # ตรวจสอบแล้วถูกต้อง
        self.motor_pins = {
            'FL': (17, 18),
            'FR': (22, 23),
            'RL': (24, 25),
            'RR': (5, 6),
        }

        self.lines = {}
        self.setup_gpio()

        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.get_logger().info("✅ Mecanum Motor Node started using gpiod (Ubuntu 24.04)!")

    def setup_gpio(self):
        for name, (in1, in2) in self.motor_pins.items():
            try:
                line1 = self.chip.get_line(in1)
                line2 = self.chip.get_line(in2)

                line1.request(consumer='motor_node', type=gpiod.LINE_REQ_DIR_OUT)
                line2.request(consumer='motor_node', type=gpiod.LINE_REQ_DIR_OUT)

                line1.set_value(0)
                line2.set_value(0)

                self.lines[name] = (line1, line2)
                self.get_logger().info(f"✅ GPIO{in1}, GPIO{in2} ready for motor {name}")
            except OSError as e:
                self.get_logger().error(f"❌ Failed to request GPIO {in1}, {in2} for motor {name}: {e}")
                self.lines[name] = (None, None)

    def set_motor(self, name, direction):
        line1, line2 = self.lines.get(name, (None, None))
        if line1 is None or line2 is None:
            self.get_logger().warn(f"⚠️ Motor {name} is not initialized, skipping.")
            return

        if direction == 1:
            line1.set_value(1)
            line2.set_value(0)
        elif direction == -1:
            line1.set_value(0)
            line2.set_value(1)
        else:
            line1.set_value(0)
            line2.set_value(0)

    def cmd_vel_callback(self, msg):
        vx = msg.linear.x
        vy = msg.linear.y
        omega = msg.angular.z

        directions = {
            'FL': math.copysign(1, vx - vy - omega) if vx or vy or omega else 0,
            'FR': math.copysign(1, vx + vy + omega) if vx or vy or omega else 0,
            'RL': math.copysign(1, vx + vy - omega) if vx or vy or omega else 0,
            'RR': math.copysign(1, vx - vy + omega) if vx or vy or omega else 0,
        }

        for wheel, direction in directions.items():
            self.set_motor(wheel, int(direction))

    def destroy_node(self):
        for line1, line2 in self.lines.values():
            try:
                if line1:
                    line1.set_value(0)
                    line1.release()
                if line2:
                    line2.set_value(0)
                    line2.release()
            except Exception as e:
                self.get_logger().warn(f"⚠️ Error releasing GPIO: {e}")
        self.chip.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MecanumMotorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
