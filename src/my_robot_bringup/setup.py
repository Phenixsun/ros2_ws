from setuptools import setup

package_name = 'my_robot_bringup'

setup(
    name=package_name,
    version='0.0.1',
    packages=[],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/bringup.launch.py',
            'launch/online_slam_launch.py',
            'launch/main.launch.py'  # ✅ เพิ่มไฟล์ launch หลัก
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robokae',
    maintainer_email='beeyond.ma@gmail.com',
    description='Launch files for bringing up the full robot system',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
