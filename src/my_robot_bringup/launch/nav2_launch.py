from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nav2_bringup',
            executable='navigation_launch.py',
            output='screen',
            parameters=[
                {'use_sim_time': False},
                {'autostart': True},
                {'params_file': '/home/robokae/ros2_ws/src/my_robot_bringup/config/nav2_params.yaml'},
                {'map': '/home/robokae/ros2_ws/src/my_robot_bringup/maps/my_room.yaml'}
            ]
        )
    ])
