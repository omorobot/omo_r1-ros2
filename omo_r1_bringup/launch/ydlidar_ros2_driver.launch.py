#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    bringup_dir = get_package_share_directory('omo_r1_bringup')

    lidar_yaml = LaunchConfiguration('lidar_yaml', default=os.path.join(bringup_dir, 'param', 'g2.yaml'))

    lidar_yaml_arg = DeclareLaunchArgument('lidar_yaml', default_value=lidar_yaml)

    lidar_node = LifecycleNode(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        name='ydlidar_ros2_driver_node',
        output='screen',
        emulate_tty=True,
        parameters=[lidar_yaml],
        namespace='/'
    )

    ld = LaunchDescription()
    ld.add_action(lidar_yaml_arg)
    ld.add_action(lidar_node)

    return ld
