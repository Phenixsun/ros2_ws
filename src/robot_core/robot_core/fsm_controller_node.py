import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from enum import Enum
import time

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

NAV_GOALS = {
    'pickup_zone': {"x": 1.0, "y": 1.0, "yaw": 0.0},
    'room1': {"x": 3.0, "y": 1.0, "yaw": 0.0},
    'room2': {"x": 3.0, "y": 3.0, "yaw": 0.0},
    'room3': {"x": 1.0, "y": 3.0, "yaw": 0.0},
    'start': {"x": 0.0, "y": 0.0, "yaw": 0.0}
}

class FSMController(Node):
    def __init__(self):
        super().__init__('fsm_controller')
        self.state = RobotState.IDLE
        self.timer = self.create_timer(1.0, self.state_machine)
        self.goal_pub = self.create_publisher(String, 'navigation/goal', 10)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.current_goal = None
        self.object_count = 0
        self.get_logger().info("FSM Controller Node Started")

    def state_machine(self):
        if self.state == RobotState.IDLE:
            self.send_navigation_goal('pickup_zone')
            self.state = RobotState.MOVE_TO_PICKUP

        elif self.state == RobotState.MOVE_TO_PICKUP:
            if self.goal_reached():
                self.get_logger().info("Reached pickup zone")
                self.state = RobotState.PICKUP_OBJECT

        elif self.state == RobotState.PICKUP_OBJECT:
            self.object_count += 1
            self.get_logger().info(f"Picked object {self.object_count}")
            if self.object_count >= 4:
                self.send_navigation_goal('room1')
                self.state = RobotState.MOVE_TO_ROOM1

        elif self.state == RobotState.MOVE_TO_ROOM1:
            if self.goal_reached():
                self.get_logger().info("Reached room 1")
                self.state = RobotState.PLACE_IN_ROOM1

        elif self.state == RobotState.PLACE_IN_ROOM1:
            self.get_logger().info("Placed 2 objects in room 1")
            self.send_navigation_goal('room2')
            self.state = RobotState.MOVE_TO_ROOM2

        elif self.state == RobotState.MOVE_TO_ROOM2:
            if self.goal_reached():
                self.get_logger().info("Reached room 2")
                self.state = RobotState.PLACE_IN_ROOM2

        elif self.state == RobotState.PLACE_IN_ROOM2:
            self.get_logger().info("Placed 1 object in room 2")
            self.send_navigation_goal('room3')
            self.state = RobotState.MOVE_TO_ROOM3

        elif self.state == RobotState.MOVE_TO_ROOM3:
            if self.goal_reached():
                self.get_logger().info("Reached room 3")
                self.state = RobotState.PLACE_IN_ROOM3

        elif self.state == RobotState.PLACE_IN_ROOM3:
            self.get_logger().info("Placed 1 object in room 3")
            self.send_navigation_goal('start')
            self.state = RobotState.RETURN_TO_START

        elif self.state == RobotState.RETURN_TO_START:
            if self.goal_reached():
                self.get_logger().info("Returned to start")
                self.state = RobotState.DONE

        elif self.state == RobotState.DONE:
            self.get_logger().info("✅ Mission completed.")
            self.timer.cancel()

    def send_navigation_goal(self, location_name):
        pose = NAV_GOALS[location_name]
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = pose['x']
        goal_msg.pose.pose.position.y = pose['y']
        goal_msg.pose.pose.orientation.w = 1.0  # Simple forward-facing
        self.current_goal = self.nav_client.send_goal_async(goal_msg)
        self.get_logger().info(f"Sending goal to {location_name}: ({pose['x']}, {pose['y']})")

    def goal_reached(self):
        if self.current_goal is None:
            return False
        if self.current_goal.done():
            result = self.current_goal.result()
            if result.result.success:
                return True
            else:
                self.get_logger().warn("Navigation failed, retrying")
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
