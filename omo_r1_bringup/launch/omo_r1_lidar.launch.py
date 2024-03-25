#!/usr/bin/python3

import os
import lifecycle_msgs.msg
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import LogInfo



def generate_launch_description():
  omo_r1_lidar_parameter = LaunchConfiguration(
    'omo_r1_lidar_parameter',
    default=os.path.join(
      get_package_share_directory('omo_r1_bringup'),
      'param',
      'omo_r1_lidar.yaml'))

  return LaunchDescription([
    DeclareLaunchArgument(
      'omo_r1_lidar_parameter',
      default_value=omo_r1_lidar_parameter
    ),
        
    LifecycleNode(
      package='ydlidar_ros2_driver',
      executable='ydlidar_ros2_driver_node',
      name='ydlidar_ros2_driver_node',
      output='screen',
      emulate_tty=True,
      parameters=[omo_r1_lidar_parameter],
      namespace='/',
    )
  ])
