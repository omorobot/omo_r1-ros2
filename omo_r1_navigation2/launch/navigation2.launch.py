#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    navigation2_dir = get_package_share_directory('omo_r1_navigation2')
    nav2_launch_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')

    map = LaunchConfiguration('map', default=os.path.join(navigation2_dir, 'map', 'turtle_world.yaml'))
    params_file = LaunchConfiguration('params_file', default=os.path.join(navigation2_dir, 'param', 'omo_r1.yaml'))
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    map_arg = DeclareLaunchArgument('map', default_value=map)
    params_file_arg = DeclareLaunchArgument('params_file', default_value=params_file)
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value=use_sim_time)

    navigation2_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nav2_launch_dir, '/bringup_launch.py']),
        launch_arguments={'map': map, 'params_file': params_file, 'use_sim_time': use_sim_time}.items()
    )

    ld = LaunchDescription()
    ld.add_action(map_arg)
    ld.add_action(params_file_arg)
    ld.add_action(use_sim_time_arg)
    ld.add_action(navigation2_node)

    return ld
