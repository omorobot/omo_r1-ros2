#!/usr/bin/env python3
import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription

def generate_launch_description():
    description_dir = get_package_share_directory('omo_r1_description')

    rviz_file = LaunchConfiguration('rviz_file', default=os.path.join(description_dir, 'rviz', 'description.rviz'))
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    rviz_file_arg = DeclareLaunchArgument('rviz_file', default_value=rviz_file)
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value=use_sim_time)

    with open(os.path.join(description_dir, 'urdf', 'omo_r1.urdf'), 'r') as infp:
        robot_description = infp.read()

    robot_state_publisher_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/robot_state_publisher.launch.py']),
        launch_arguments={'robot_description': robot_description, 'use_sim_time': use_sim_time}.items()
    )

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_file],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    ld = LaunchDescription()
    ld.add_action(rviz_file_arg)
    ld.add_action(use_sim_time_arg)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(rviz2_node)

    return ld