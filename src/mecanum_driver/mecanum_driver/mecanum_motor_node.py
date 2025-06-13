#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import lgpio
from geometry_msgs.msg import Twist

class MecanumMotorNode(Node):
    def __init__(self):
        super().__init__('mecanum_motor_node')

        self.chip = 4  # RPi 5
        self.gpio_handle = lgpio.gpiochip_open(self.chip)

        # DIR = ทิศทาง / PWM = ความเร็ว
        self.motor_pins = {
            'FL': {'DIR': 27, 'PWM': 12},
            'FR': {'DIR': 23, 'PWM': 13},
            'RL': {'DIR': 25, 'PWM': 18},
            'RR': {'DIR': 16, 'PWM': 19},
        }

        self.FREQ = 500
        self.DUTY_LINEAR = 50
        self.DUTY_TURN_LEFT = 60
        self.DUTY_TURN_RIGHT = 80

        self.setup_gpio()
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.get_logger().info("✅ Mecanum Motor Node started (lgpio)")

    def setup_gpio(self):
        for name, pins in self.motor_pins.items():
            lgpio.gpio_claim_output(self.gpio_handle, pins['DIR'])
            lgpio.gpio_claim_output(self.gpio_handle, pins['PWM'])
            lgpio.tx_pwm(self.gpio_handle, pins['PWM'], self.FREQ, 0)
            lgpio.gpio_write(self.gpio_handle, pins['DIR'], 0)
            self.get_logger().info(f"⚙️ Motor {name}: DIR={pins['DIR']} PWM={pins['PWM']} ready")

    def set_motor(self, name, direction, duty):
        pins = self.motor_pins[name]
        lgpio.gpio_write(self.gpio_handle, pins['DIR'], 1 if direction >= 0 else 0)
        lgpio.tx_pwm(self.gpio_handle, pins['PWM'], self.FREQ, duty)

    def stop_all(self):
        for pins in self.motor_pins.values():
            lgpio.tx_pwm(self.gpio_handle, pins['PWM'], self.FREQ, 0)

    def cmd_vel_callback(self, msg):
        vx = msg.linear.x
        vy = msg.linear.y
        omega = msg.angular.z

        if vx == 0.0 and vy == 0.0 and omega == 0.0:
            self.stop_all()
            return

        directions = {
            'FL': vx - vy - omega,
            'FR': vx + vy + omega,
            'RL': vx + vy - omega,
            'RR': vx - vy + omega,
        }

        for wheel, value in directions.items():
            if vx != 0 or vy != 0:
                duty = self.DUTY_LINEAR
            elif omega < 0:
                duty = self.DUTY_TURN_LEFT
            else:
                duty = self.DUTY_TURN_RIGHT

            direction = 1 if value >= 0 else -1
            self.set_motor(wheel, direction, duty if value != 0 else 0)

    def destroy_node(self):
        self.stop_all()
        lgpio.gpiochip_close(self.gpio_handle)
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
