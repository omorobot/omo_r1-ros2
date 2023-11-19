#!/usr/bin/env python3
import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    cartographer_dir = get_package_share_directory('omo_r1_cartographer')

    rviz_file = LaunchConfiguration('rviz_file', default=os.path.join(cartographer_dir, 'rviz', 'cartographer.rviz'))
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    rviz_file_arg = DeclareLaunchArgument('rviz_file', default_value=rviz_file)
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value=use_sim_time)

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_file],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    ld = LaunchDescription()
    ld.add_action(rviz_file_arg)
    ld.add_action(use_sim_time_arg)
    ld.add_action(rviz2_node)

    return ld
