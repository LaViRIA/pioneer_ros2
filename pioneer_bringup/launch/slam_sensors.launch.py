"""
Launches the onboard drivers a Pioneer 3AT needs for off-board SLAM (slam_toolbox).

- Robot description (TF for base_laser_link and imu_link)
- Aria base driver (/odom, /cmd_vel)
- SICK LMS 200 lidar (/scan in base_laser_link)
- BNO055 USB stick IMU (/imu/data in imu_link)
- EKF (robot_localization) fusing wheel speed and gyro yaw rate into the
  odom -> base_link TF. With use_ekf:=false, Aria publishes that TF from the
  wheel odometry instead.

The lidar and IMU drivers live outside this repository (sick_lms200_node and
bno055_usb_stick packages in the same workspace).
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    aria_dir = get_package_share_directory('pioneer_aria')
    bringup_dir = get_package_share_directory('pioneer_bringup')
    description_dir = get_package_share_directory('pioneer_description')

    laser_port = LaunchConfiguration('laser_port')
    laser_baud = LaunchConfiguration('laser_baud')
    use_imu = LaunchConfiguration('use_imu')
    use_ekf = LaunchConfiguration('use_ekf')

    declare_laser_port_arg = DeclareLaunchArgument(
        'laser_port', default_value='/dev/ttyS1',
        description='Serial port of the SICK LMS 200')

    declare_laser_baud_arg = DeclareLaunchArgument(
        'laser_baud', default_value='38400',
        description='Baud rate of the SICK LMS 200')

    declare_use_imu_arg = DeclareLaunchArgument(
        'use_imu', default_value='true',
        description='Launch the BNO055 USB stick IMU driver')

    declare_use_ekf_arg = DeclareLaunchArgument(
        'use_ekf', default_value='true',
        description='Fuse wheel odometry and IMU in an EKF for odom -> base_link')

    state_publisher_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(description_dir, 'launch', 'state_publisher.launch.py')),
    )

    aria_launch_file = os.path.join(aria_dir, 'launch', 'aria.launch.py')
    aria_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(aria_launch_file),
        condition=UnlessCondition(use_ekf),
    )

    # Same Aria setup, but the EKF publishes odom -> base_link instead of Aria
    aria_ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(aria_launch_file),
        launch_arguments={
            'params_file': os.path.join(bringup_dir, 'params', 'slam_aria.yaml'),
        }.items(),
        condition=IfCondition(use_ekf),
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(bringup_dir, 'params', 'ekf.yaml')],
        condition=IfCondition(use_ekf),
    )

    laser_node = Node(
        package='sick_lms200_node',
        executable='sick_lms200_node',
        name='sick_lms200_node',
        output='screen',
        parameters=[{
            'serial_port': laser_port,
            'baud_rate': laser_baud,
            'frame_id': 'base_laser_link',
        }],
    )

    # The URDF already defines imu_link, so the IMU launch must not publish its own TF.
    # params_file must be passed explicitly: the Aria launch above sets a launch
    # configuration of the same name, which would otherwise leak into this include.
    imu_dir = get_package_share_directory('bno055_usb_stick')
    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(imu_dir, 'launch', 'bno055_usb_stick.launch.py')),
        launch_arguments={
            'publish_tf': 'false',
            'params_file': os.path.join(imu_dir, 'config', 'params.yaml'),
        }.items(),
        condition=IfCondition(use_imu),
    )

    return LaunchDescription([
        declare_laser_port_arg,
        declare_laser_baud_arg,
        declare_use_imu_arg,
        declare_use_ekf_arg,
        state_publisher_launch,
        aria_launch,
        aria_ekf_launch,
        laser_node,
        imu_launch,
        ekf_node,
    ])
