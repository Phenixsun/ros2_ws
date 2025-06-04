import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from enum import Enum
import time
import serial
import yaml
import os

class RobotState(Enum):
    IDLE = 0
    MOVE_TO_PICKUP = 1
    PICKUP_OBJECT = 2
    MOVE_TO_ROOM1 = 3
    PLACE_IN_ROOM1 = 4
    MOVE_TO_ROOM2 = 5
    PLACE_IN_ROOM2 = 6
    MOVE_TO_ROOM3 = 7
    PLACE_IN_ROOM3 = 8
    RETURN_TO_START = 9
    DONE = 10

class FSMController(Node):
    def __init__(self):
        super().__init__('fsm_controller')
        self.state = RobotState.IDLE
        self.timer = self.create_timer(1.0, self.state_machine)
        self.goal_pub = self.create_publisher(String, 'navigation/goal', 10)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.current_goal = None
        self.object_count = 0
        self.state_enter_time = time.time()

        try:
            self.arduino = serial.Serial('/dev/ttyUSB1', 9600, timeout=1)
            time.sleep(2)
            self.get_logger().info("✅ Arduino connected on /dev/ttyUSB1")
        except Exception as e:
            self.get_logger().error(f"❌ Arduino connection failed: {e}")
            self.arduino = None

        self.nav_goals = self.load_nav_goals()
        self.get_logger().info("FSM Controller Node Started")

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

    def wait_delay(self, seconds):
        return time.time() - self.state_enter_time >= seconds

    def transition_state(self, new_state):
        self.state = new_state
        self.state_enter_time = time.time()

    def state_machine(self):
        if self.state == RobotState.IDLE:
            self.send_navigation_goal('pickup_zone')
            self.transition_state(RobotState.MOVE_TO_PICKUP)

        elif self.state == RobotState.MOVE_TO_PICKUP:
            if self.goal_reached():
                self.get_logger().info("📍 Reached pickup zone")
                self.transition_state(RobotState.PICKUP_OBJECT)

        elif self.state == RobotState.PICKUP_OBJECT:
            if self.wait_delay(1.5):
                self.object_count += 1
                self.get_logger().info(f"🤖 Picking object {self.object_count}")
                self.send_arduino_command(f"pick_{self.object_count}")
                self.transition_state(RobotState.PICKUP_OBJECT if self.object_count < 4 else RobotState.MOVE_TO_ROOM1)

        elif self.state == RobotState.MOVE_TO_ROOM1:
            self.send_navigation_goal('room1')
            self.transition_state(RobotState.PLACE_IN_ROOM1)

        elif self.state == RobotState.PLACE_IN_ROOM1:
            if self.goal_reached() and self.wait_delay(1.0):
                self.send_arduino_command("place_1")
                time.sleep(1)
                self.send_arduino_command("place_2")
                self.send_navigation_goal('room2')
                self.transition_state(RobotState.MOVE_TO_ROOM2)

        elif self.state == RobotState.MOVE_TO_ROOM2:
            if self.goal_reached():
                self.transition_state(RobotState.PLACE_IN_ROOM2)

        elif self.state == RobotState.PLACE_IN_ROOM2:
            if self.wait_delay(1.0):
                self.send_arduino_command("place_3")
                self.send_navigation_goal('room3')
                self.transition_state(RobotState.MOVE_TO_ROOM3)

        elif self.state == RobotState.MOVE_TO_ROOM3:
            if self.goal_reached():
                self.transition_state(RobotState.PLACE_IN_ROOM3)

        elif self.state == RobotState.PLACE_IN_ROOM3:
            if self.wait_delay(1.0):
                self.send_arduino_command("place_4")
                self.send_navigation_goal('start')
                self.transition_state(RobotState.RETURN_TO_START)

        elif self.state == RobotState.RETURN_TO_START:
            if self.goal_reached():
                self.transition_state(RobotState.DONE)

        elif self.state == RobotState.DONE:
            self.get_logger().info("✅ Mission completed.")
            self.timer.cancel()

    def send_navigation_goal(self, location_name):
        pose = self.nav_goals.get(location_name, None)
        if pose is None:
            self.get_logger().error(f"❌ NAV_GOAL '{location_name}' not found in YAML")
            return
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = pose['x']
        goal_msg.pose.pose.position.y = pose['y']
        goal_msg.pose.pose.orientation.w = 1.0
        self.current_goal = self.nav_client.send_goal_async(goal_msg)
        self.get_logger().info(f"🚗 Sending goal to {location_name}: ({pose['x']}, {pose['y']})")

    def goal_reached(self):
        if self.current_goal is None:
            return False
        if self.current_goal.done():
            result = self.current_goal.result()
            if result.result.success:
                return True
            else:
                self.get_logger().warn("⚠️ Navigation failed. Retrying...")
                return False
        return False

def main(args=None):
    rclpy.init(args=args)
    node = FSMController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()
