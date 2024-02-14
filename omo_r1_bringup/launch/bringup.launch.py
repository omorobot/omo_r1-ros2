#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    bringup_dir = get_package_share_directory('omo_r1_bringup')
    description_dir = get_package_share_directory('omo_r1_description')
    robot_dir = get_package_share_directory('omo_r1_robot')

    lidar_yaml = LaunchConfiguration('lidar_yaml', default=os.path.join(bringup_dir, 'param', 'g2.yaml'))
    robot_yaml = LaunchConfiguration('robot_yaml', default=os.path.join(robot_dir, 'param', 'omo_r1.yaml'))
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    lidar_yaml_arg = DeclareLaunchArgument('lidar_yaml', default_value=lidar_yaml)
    robot_yaml_arg = DeclareLaunchArgument('robot_yaml', default_value=robot_yaml)
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value=use_sim_time)

    robot_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([robot_dir, '/launch/robot_control.launch.py']),
        launch_arguments={'robot_yaml': robot_yaml}.items()
    )

    lidar_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([bringup_dir, '/launch/ydlidar_ros2_driver.launch.py']),
        launch_arguments={'lidar_yaml': lidar_yaml}.items()
    )

    robot_state_publisher_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([description_dir, '/launch/robot_state_publisher.launch.py']),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    ld = LaunchDescription()
    ld.add_action(lidar_yaml_arg)
    ld.add_action(robot_yaml_arg)
    ld.add_action(use_sim_time_arg)
    ld.add_action(robot_node)
    ld.add_action(lidar_node)
    ld.add_action(robot_state_publisher_node)

    return ld
