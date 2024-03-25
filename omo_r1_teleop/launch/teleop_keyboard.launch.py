#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    teleop_keyboard = ExecuteProcess(
        cmd=['ros2', 'run', 'omo_r1_teleop', 'teleop_keyboard'],
        output='screen',
        prefix='gnome-terminal --'
    )

    ld = LaunchDescription()
    ld.add_action(teleop_keyboard)

    return ld