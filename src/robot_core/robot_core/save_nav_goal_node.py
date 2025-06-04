import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import yaml
import os
from datetime import datetime

class SaveNavGoalNode(Node):
    def __init__(self):
        super().__init__('save_nav_goal_node')
        self.subscription = self.create_subscription(
            PoseStamped,
            '/goal_pose',  # from RViz2 Nav Goal Tool
            self.listener_callback,
            10
        )
        self.subscription  # prevent unused variable warning
        self.saved_goals = {}
        self.yaml_file = os.path.join(
            os.path.expanduser('~'),
            'ros2_ws',
            'src',
            'robot_core',
            'config',
            'nav_goals.yaml'
        )
        self.load_existing_goals()
        self.get_logger().info("📍 Ready to save nav goals from RViz (/goal_pose)")

    def listener_callback(self, msg):
        while True:
            print("\nAvailable commands:")
            print("  🔤 [name]     - Save with custom name (e.g., pickup_zone)")
            print("  📜 'list'     - Show saved goals")
            print("  🧹 'reset'    - Clear all saved goals")
            print("  ⏸️  'pause'    - Pause until Enter")
            print("  ❌ 'cancel'   - Skip saving this goal")
            name = input("Enter goal name or command: ").strip()

            if name == 'pause':
                input("⏸️ Paused. Press Enter to continue...")
                return
            elif name == 'reset':
                self.saved_goals = {}
                self.get_logger().info("🔄 All saved goals have been cleared.")
                self.save_to_yaml()
                return
            elif name == 'list':
                print("\n📜 Saved Goals:")
                for key, val in self.saved_goals.items():
                    print(f"  - {key}: x={val['x']}, y={val['y']}, yaw={val['yaw']}")
                continue
            elif name == 'cancel':
                print("❌ Skipping current goal.")
                return
            elif name:
                break
            else:
                self.get_logger().warn("⚠️ No name entered. Try again.")

        yaw = self.get_yaw_from_quaternion(msg.pose.orientation)
        self.saved_goals[name] = {
            'x': float(msg.pose.position.x),
            'y': float(msg.pose.position.y),
            'yaw': float(yaw)
        }
        self.get_logger().info(f"✅ Saved goal '{name}': x={msg.pose.position.x:.2f}, y={msg.pose.position.y:.2f}, yaw={yaw:.2f}")
        self.save_to_yaml()

    def load_existing_goals(self):
        if os.path.exists(self.yaml_file):
            try:
                with open(self.yaml_file, 'r') as f:
                    self.saved_goals = yaml.safe_load(f) or {}
                self.get_logger().info(f"📂 Loaded existing nav goals from {self.yaml_file}")
            except Exception as e:
                self.get_logger().error(f"⚠️ Failed to load existing YAML: {e}")

    def get_yaw_from_quaternion(self, q):
        import math
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def save_to_yaml(self):
        try:
            with open(self.yaml_file, 'w') as f:
                yaml.dump(self.saved_goals, f)
            self.get_logger().info(f"📁 nav_goals.yaml updated at: {self.yaml_file}")
        except Exception as e:
            self.get_logger().error(f"❌ Failed to save YAML: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = SaveNavGoalNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()
