#!/usr/bin/env python3
"""Fast visual demo: map + CBBA + MAPF + cmd_vel (no heavy Nav2 stacks).

Pipeline still present:
  CBBA assigns tasks → prioritized MAPF plans conflict-aware paths
  → lightweight cmd_vel follower drives robots at decent speed.
"""

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

    lite = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(dual_pkg, 'launch', 'four_robots_lite.launch.py')
        ),
        launch_arguments={
            'headless': headless,
            'use_rviz': use_rviz,
        }.items(),
    )

    mapf = Node(
        package='dual_robot_known_map',
        executable='prioritized_mapf.py',
        name='prioritized_mapf',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'map_topic': '/robot1/map',
            'grid_stride': 2,
            'robot_radius': 0.18,
        }],
    )

    follower = Node(
        package='dual_robot_known_map',
        executable='cmd_vel_path_follower.py',
        name='cmd_vel_path_follower',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'linear_speed': 0.30,
            'angular_speed': 1.3,
            'waypoint_tol': 0.25,
            'final_tol': 0.22,
            'pose_stride': 2,
            'tick_hz': 20.0,
        }],
    )

    mission = Node(
        package='eticbba',
        executable='cbba_lite_mission',
        name='cbba_lite_mission',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'output_file': '/tmp/house_10_cbba_assignment.yaml',
            'result_file': '/tmp/cbba_mapf_mission_result.yaml',
            'goal_tolerance_xy': 0.50,
            'paths_ready_timeout_sec': 90.0,
            'leg_timeout_sec': 240.0,
            'task_timeout_sec': 240.0,
            'replan_retries': 4,
            'tf_wait_sec': 40.0,
        }],
    )

    # Short warm-up: no Nav2 lifecycle to wait for
    delayed_swarm = TimerAction(period=8.0, actions=[mapf, follower])
    delayed_mission = TimerAction(period=mission_delay, actions=[mission])

    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='False'),
        DeclareLaunchArgument('use_rviz', default_value='True'),
        DeclareLaunchArgument('mission_delay', default_value='20.0'),
        lite,
        delayed_swarm,
        delayed_mission,
    ])
