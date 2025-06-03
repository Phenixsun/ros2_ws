from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # RPLiDAR
    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('rplidar_ros'), 'launch', 'rplidar_c1_launch.py')
        ),
        launch_arguments={
            'serial_port': '/dev/ttyUSB0',  # หรือใช้ '/dev/rplidar' ถ้าสร้าง symlink ไว้
            'serial_baudrate': '460800'
        }.items()
    )

    # Robot Description (URDF)
    urdf_file = os.path.join(get_package_share_directory('my_robot_description'), 'urdf', 'my_robot.urdf')
    with open(urdf_file, 'r') as infp:
        robot_description = infp.read()

    state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    # Static Transform: base_link -> laser
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='laser_tf_pub',
        arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'laser']
    )

    # SLAM Toolbox
    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
        )
    )

    # Huskylens UART Node
    huskylens_node = Node(
        package='huskylens_uart',
        executable='huskylens_node',
        name='huskylens_node',
        output='screen'
    )

    # Mecanum Motor Node
    mecanum_motor_node = Node(
        package='mecanum_driver',
        executable='motor_node',
        name='mecanum_motor_node',
        output='screen'
    )

    # Arm Controller (ผ่าน UART to Arduino)
    arm_serial_node = Node(
        package='robot_core',
        executable='arm_serial_node',
        name='arm_serial_node',
        output='screen'
    )

    # (Optional) Navigation2 - ถ้าคุณตั้งค่าเสร็จแล้ว
    # nav2_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(get_package_share_directory('navigation2'), 'bringup', 'launch', 'bringup_launch.py')
    #     ),
    #     launch_arguments={
    #         'map': '/home/robokae/maps/office.yaml',
    #         'use_sim_time': 'false',
    #         'params_file': '/home/robokae/ros2_ws/src/navigation2/params/nav2_params.yaml'
    #     }.items()
    # )

    return LaunchDescription([
        rplidar_launch,
        state_publisher,
        static_tf,
        slam_toolbox,
        huskylens_node,
        mecanum_motor_node,
        arm_serial_node,
        # nav2_launch  # uncomment ถ้าต้องการ
    ])
