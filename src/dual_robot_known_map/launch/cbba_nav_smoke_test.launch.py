#!/usr/bin/env python3
"""Run CBBA Nav2 smoke test (requires four_robots_nav2 already up)."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    mode = LaunchConfiguration('mode')
    tol = LaunchConfiguration('goal_tolerance_m')

    return LaunchDescription([
        DeclareLaunchArgument(
            'mode',
            default_value='parallel_pairs',
            description='sequential | parallel_all | parallel_pairs',
        ),
        DeclareLaunchArgument('goal_tolerance_m', default_value='0.40'),
        Node(
            package='dual_robot_known_map',
            executable='cbba_nav_smoke_test.py',
            name='cbba_nav_smoke_test',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'mode': mode,
                'goal_tolerance_m': tol,
            }],
        ),
    ])
