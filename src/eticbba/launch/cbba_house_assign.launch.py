#!/usr/bin/env python3
"""Run CBBA assignment (requires Nav2 planner active on robot1)."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    tasks_file = LaunchConfiguration('tasks_file')
    planner_action = LaunchConfiguration('planner_action')
    output_file = LaunchConfiguration('output_file')

    return LaunchDescription([
        DeclareLaunchArgument('tasks_file', default_value=''),
        DeclareLaunchArgument(
            'planner_action',
            default_value='/robot1/compute_path_to_pose',
        ),
        DeclareLaunchArgument(
            'output_file',
            default_value='/tmp/house_10_cbba_assignment.yaml',
        ),
        Node(
            package='eticbba',
            executable='cbba_house_assign',
            name='cbba_house_assign',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'tasks_file': tasks_file,
                'planner_action': planner_action,
                'publish_goals': True,
                'output_file': output_file,
            }],
        ),
    ])
