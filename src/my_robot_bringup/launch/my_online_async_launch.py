from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    return LaunchDescription([
        # ทำให้สามารถกำหนดไฟล์ params จากคำสั่ง ros2 launch ได้
        DeclareLaunchArgument(
            'slam_params_file',
            default_value='/home/robokae/ros2_ws/src/my_robot_bringup/config/mapper_params_online_async.yaml',
            description='Full path to the slam_toolbox parameters file'
        ),

        LifecycleNode(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',  # ✅ ชื่อควรเป็น 'slam_toolbox' สำหรับระบบ ROS2
            namespace='', #TypeError: LifecycleNode.__init__() missing 1 required keyword-only argument: 'namespace
            output='screen',
            parameters=[LaunchConfiguration('slam_params_file')],
        )
    ])
