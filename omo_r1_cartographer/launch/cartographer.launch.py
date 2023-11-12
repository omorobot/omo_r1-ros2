#!/usr/bin/env python3
import os
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    cartographer_dir = get_package_share_directory('omo_r1_cartographer')

    configuration_basename = LaunchConfiguration('configuration_basename', default='omo_r1.lua')
    configuration_directory = LaunchConfiguration('configuration_directory', default=os.path.join(cartographer_dir, 'config'))
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')
    resolution = LaunchConfiguration('resolution', default='0.05')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    configuration_basename_arg = DeclareLaunchArgument('configuration_basename', default_value=configuration_basename)
    configuration_directory_arg = DeclareLaunchArgument('configuration_directory', default_value=configuration_directory)
    publish_period_sec_arg = DeclareLaunchArgument('publish_period_sec', default_value=publish_period_sec)
    resolution_arg = DeclareLaunchArgument('resolution', default_value=resolution)
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value=use_sim_time)

    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        arguments=['-configuration_basename', configuration_basename, '-configuration_directory', configuration_directory],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    occupancy_grid_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/occupancy_grid.launch.py']),
        launch_arguments={'publish_period_sec': publish_period_sec, 'resolution': resolution, 'use_sim_time': use_sim_time}.items()
    )

    ld = LaunchDescription()
    ld.add_action(configuration_basename_arg)
    ld.add_action(configuration_directory_arg)
    ld.add_action(publish_period_sec_arg)
    ld.add_action(resolution_arg)
    ld.add_action(use_sim_time_arg)
    ld.add_action(cartographer_node)
    ld.add_action(occupancy_grid_node)

    return ld
