#!/usr/bin/env python3
import os
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    robot_dir = get_package_share_directory('omo_r1_robot')
    
    robot_yaml = LaunchConfiguration('robot_yaml', default=os.path.join(robot_dir, 'param', 'omo_r1.yaml'))

    robot_yaml_arg = DeclareLaunchArgument('robot_yaml', default_value=robot_yaml)
    
    robot_control_node = Node(
        package='omo_r1_robot',
        executable='robot_control',
        name='robot_control',
        output='screen',
        emulate_tty=True,
        parameters=[robot_yaml],
        namespace=''
    )
    
    ld = LaunchDescription()
    ld.add_action(robot_yaml_arg)
    ld.add_action(robot_control_node)
    
    return ld
