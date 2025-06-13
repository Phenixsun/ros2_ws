import rclpy
import math
from rclpy.node import Node
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler
from enum import Enum
import time
import serial
import yaml
import os

class RobotState(Enum):
    IDLE = 0
    MOVE_TO_PICKUP = 1
    PICKUP_OBJECTS = 2
    MOVE_TO_ROOM1 = 3
    PLACE_IN_ROOM1 = 4
    MOVE_TO_ROOM2 = 5
    PLACE_IN_ROOM2 = 6
    MOVE_TO_CONVEYOR = 7
    PLACE_ON_CONVEYOR = 8
    RETURN_TO_START = 9
    DONE = 10

class FSMController(Node):
    def __init__(self):
        super().__init__('fsm_controller')
        self.state = RobotState.IDLE
        self.timer = self.create_timer(1.0, self.state_machine)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.current_goal = None
        self.detected_objects = []
        self.picked_objects = []
        self.waiting_for_arm = False
        self.arm_ack_timeout = 10.0
        self.state_enter_time = time.time()

        self.subscription = self.create_subscription(
            String,
            '/huskylens/objects',
            self.huskylens_callback,
            10
        )

        try:
            self.arduino = serial.Serial('/dev/ttyUSB1', 9600, timeout=1)
            time.sleep(2)
            self.get_logger().info("✅ Arduino connected on /dev/ttyUSB1")
        except Exception as e:
            self.get_logger().error(f"❌ Arduino connection failed: {e}")
            self.arduino = None

        self.object_to_angle = {
            'brake_fluid_can': 90,
            'multimeter': -90,
            'inner_tube': 45,
            'led_bulb': -45
        }

        self.object_to_room = {
            'brake_fluid_can': 'room1',
            'multimeter': 'room1',
            'inner_tube': 'room2',
            'led_bulb': 'conveyor'
        }

        self.object_to_place_cmd = {
            'brake_fluid_can': 'place_1',
            'multimeter': 'place_1',
            'inner_tube': 'place_3',
            'led_bulb': 'place_4'
        }

        self.nav_goals = self.load_nav_goals()
        self.get_logger().info("🧠 FSM Controller Node Started")

    def load_nav_goals(self):
        file_path = os.path.join(os.path.expanduser('~'), 'ros2_ws', 'src', 'robot_core', 'config', 'nav_goals.yaml')
        try:
            with open(file_path, 'r') as f:
                data = yaml.safe_load(f)
                self.get_logger().info(f"📍 Loaded nav goals from YAML: {file_path}")
                return data
        except Exception as e:
            self.get_logger().error(f"⚠️ Failed to load nav_goals.yaml: {e}")
            return {}

    def send_arduino_command(self, command):
        if self.arduino:
            self.arduino.write((command + '\n').encode())
            self.get_logger().info(f"📤 Sent to Arduino: {command}")
            self.waiting_for_arm = True
            self.state_enter_time = time.time()

    def wait_for_arm_ack(self):
        if not self.waiting_for_arm:
            return True
        if self.arduino.in_waiting:
            line = self.arduino.readline().decode().strip()
            if "✅ DONE" in line:
                self.waiting_for_arm = False
                return True
        elif time.time() - self.state_enter_time > self.arm_ack_timeout:
            self.get_logger().warn("⚠️ Timeout waiting for Arduino ack")
            self.waiting_for_arm = False
            return True
        return False

    def wait_delay(self, seconds):
        return time.time() - self.state_enter_time >= seconds

    def transition_state(self, new_state):
        self.state = new_state
        self.state_enter_time = time.time()

    def huskylens_callback(self, msg):
        try:
            object_name, confidence = msg.data.split(':')
            confidence = int(confidence)
            if confidence < 50:
                return
            if object_name not in self.detected_objects:
                self.detected_objects.append(object_name)
                self.get_logger().info(f"📦 Detected object: {object_name}")
        except Exception as e:
            self.get_logger().error(f"⚠️ Invalid message format: {e}")

    def state_machine(self):
        if self.state == RobotState.IDLE:
            self.send_navigation_goal('pickup_zone')
            self.transition_state(RobotState.MOVE_TO_PICKUP)

        elif self.state == RobotState.MOVE_TO_PICKUP:
            if self.goal_reached():
                self.transition_state(RobotState.PICKUP_OBJECTS)

        elif self.state == RobotState.PICKUP_OBJECTS:
            if len(self.picked_objects) < 4 and self.detected_objects:
                obj = self.detected_objects.pop(0)
                angle = self.object_to_angle.get(obj, 0)
                self.send_arduino_command(f"CMD:PICK:{angle}")
                self.picked_objects.append(obj)
            elif self.waiting_for_arm is False and len(self.picked_objects) == 4:
                self.transition_state(RobotState.MOVE_TO_ROOM1)

        elif self.state == RobotState.MOVE_TO_ROOM1:
            self.send_navigation_goal('room1')
            self.transition_state(RobotState.PLACE_IN_ROOM1)

        elif self.state == RobotState.PLACE_IN_ROOM1:
            if self.goal_reached() and self.wait_for_arm_ack():
                for obj in ['brake_fluid_can', 'multimeter']:
                    if obj in self.picked_objects:
                        self.send_arduino_command(self.object_to_place_cmd[obj])
                        self.picked_objects.remove(obj)
                        return
                self.transition_state(RobotState.MOVE_TO_ROOM2)

        elif self.state == RobotState.MOVE_TO_ROOM2:
            self.send_navigation_goal('room2')
            self.transition_state(RobotState.PLACE_IN_ROOM2)

        elif self.state == RobotState.PLACE_IN_ROOM2:
            if self.goal_reached() and self.wait_for_arm_ack():
                if 'inner_tube' in self.picked_objects:
                    self.send_arduino_command(self.object_to_place_cmd['inner_tube'])
                    self.picked_objects.remove('inner_tube')
                    self.transition_state(RobotState.MOVE_TO_CONVEYOR)

        elif self.state == RobotState.MOVE_TO_CONVEYOR:
            self.send_navigation_goal('conveyor')
            self.transition_state(RobotState.PLACE_ON_CONVEYOR)

        elif self.state == RobotState.PLACE_ON_CONVEYOR:
            if self.goal_reached() and self.wait_for_arm_ack():
                if 'led_bulb' in self.picked_objects:
                    self.send_arduino_command(self.object_to_place_cmd['led_bulb'])
                    self.picked_objects.remove('led_bulb')
                    self.transition_state(RobotState.RETURN_TO_START)

        elif self.state == RobotState.RETURN_TO_START:
            self.send_navigation_goal('start')
            self.transition_state(RobotState.DONE)

        elif self.state == RobotState.DONE:
            self.get_logger().info("✅ Mission complete.")
            self.timer.cancel()

    def send_navigation_goal(self, location_name):
        pose = self.nav_goals.get(location_name, None)
        if pose is None:
            self.get_logger().error(f"❌ NAV_GOAL '{location_name}' not found")
            return

        yaw_rad = pose.get('yaw', 0.0)
        quat = quaternion_from_euler(0, 0, yaw_rad)

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = pose['x']
        goal_msg.pose.pose.position.y = pose['y']
        goal_msg.pose.pose.orientation.x = quat[0]
        goal_msg.pose.pose.orientation.y = quat[1]
        goal_msg.pose.pose.orientation.z = quat[2]
        goal_msg.pose.pose.orientation.w = quat[3]

        self.current_goal = self.nav_client.send_goal_async(goal_msg)
        self.get_logger().info(f"🚗 Navigating to {location_name} → x:{pose['x']} y:{pose['y']} yaw:{pose['yaw']}")

    def goal_reached(self):
        if self.current_goal is None:
            return False
        if self.current_goal.done():
            result = self.current_goal.result()
            return result.result.success
        return False

def main(args=None):
    rclpy.init(args=args)
    node = FSMController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()
