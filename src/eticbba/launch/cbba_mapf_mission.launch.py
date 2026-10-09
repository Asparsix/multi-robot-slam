#!/usr/bin/env python3
"""Full CBBA → MAPF-per-leg → verify mission on 4-robot house Nav2 stack."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    dual_pkg = get_package_share_directory('dual_robot_known_map')
    headless = LaunchConfiguration('headless')
    use_rviz = LaunchConfiguration('use_rviz')
    mission_delay = LaunchConfiguration('mission_delay')

    nav_swarm = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(dual_pkg, 'launch', 'four_robots_swarm_nav.launch.py')
        ),
        launch_arguments={
            'use_demo_goals': 'False',
            'use_rviz': use_rviz,
            'headless': headless,
        }.items(),
    )

    mission = Node(
        package='eticbba',
        executable='cbba_mapf_mission',
        name='cbba_mapf_mission',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'output_file': '/tmp/house_10_cbba_assignment.yaml',
            'result_file': '/tmp/cbba_mapf_mission_result.yaml',
            'goal_tolerance_xy': 0.45,
            'paths_ready_timeout_sec': 180.0,
            'leg_timeout_sec': 1200.0,
        }],
    )

    # Wait for Nav2 + MAPF/executor (swarm layer starts at t=15s)
    delayed = TimerAction(period=mission_delay, actions=[mission])

    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='True'),
        DeclareLaunchArgument('use_rviz', default_value='False'),
        DeclareLaunchArgument('mission_delay', default_value='90.0'),
        nav_swarm,
        delayed,
    ])
