from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='huskylens_interface',
            executable='huskylens_uart_node',
            name='huskylens_uart_node',
            output='screen'
        ),
        Node(
            package='arm_controller',
            executable='arm_serial_node',
            name='arm_serial_node',
            output='screen'
        ),
        Node(
            package='mecanum_driver',
            executable='motor_node',
            name='motor_node',
            output='screen'
        ),
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[{'use_sim_time': False}]
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen'
        ),
        Node(
            package='robot_core',
            executable='fsm_controller_node',
            name='fsm_controller_node',
            output='screen'
        )
    ])
