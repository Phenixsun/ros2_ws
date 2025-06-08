#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import lgpio
from geometry_msgs.msg import Twist
import math

class MecanumMotorNode(Node):
    def __init__(self):
        super().__init__('mecanum_motor_node')

        self.chip = 0  # gpiochip0
        self.handle = lgpio.gpiochip_open(self.chip)

        self.motor_pins = {
            'FL': (17, 18),  # DIR, PWM
            'FR': (22, 23),
            'RL': (24, 25),
            'RR': (5, 6),
        }

        self.setup_gpio()
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.get_logger().info("✅ Mecanum Motor Node started using lgpio!")

    def setup_gpio(self):
        for name, (dir_pin, pwm_pin) in self.motor_pins.items():
            try:
                lgpio.gpio_claim_output(self.handle, dir_pin)
                lgpio.gpio_claim_output(self.handle, pwm_pin)
                lgpio.gpio_write(self.handle, pwm_pin, 0)
                self.get_logger().info(f"✅ GPIO{dir_pin}, GPIO{pwm_pin} ready for motor {name}")
            except Exception as e:
                self.get_logger().error(f"❌ Failed to initialize motor {name}: {e}")

    def set_motor(self, name, direction, speed_percent):
        dir_pin, pwm_pin = self.motor_pins.get(name, (None, None))
        if dir_pin is None or pwm_pin is None:
            self.get_logger().warn(f"⚠️ Motor {name} is not configured.")
            return

        try:
            lgpio.gpio_write(self.handle, dir_pin, 1 if direction >= 0 else 0)
            speed = max(0, min(100, abs(speed_percent)))
            lgpio.tx_pwm(self.handle, pwm_pin, 20000, speed)  # 20 kHz PWM
        except Exception as e:
            self.get_logger().error(f"❌ Error setting motor {name}: {e}")

    def cmd_vel_callback(self, msg):
        vx = msg.linear.x
        vy = msg.linear.y
        omega = msg.angular.z

        base_speed = 80  # ความเร็วพื้นฐาน 0-100
        directions = {
            'FL': vx - vy - omega,
            'FR': vx + vy + omega,
            'RL': vx + vy - omega,
            'RR': vx - vy + omega,
        }

        for wheel, value in directions.items():
            self.set_motor(wheel, math.copysign(1, value) if value else 0, base_speed if value else 0)

    def destroy_node(self):
        for dir_pin, pwm_pin in self.motor_pins.values():
            try:
                lgpio.tx_pwm(self.handle, pwm_pin, 0, 0)
                lgpio.gpio_write(self.handle, dir_pin, 0)
            except Exception as e:
                self.get_logger().warn(f"⚠️ Error releasing motor pins: {e}")
        lgpio.gpiochip_close(self.handle)
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
