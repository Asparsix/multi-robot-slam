#!/usr/bin/env python3
"""4 robots: lite Gazebo bringup + PIBT brain + SMART ADG executor (no Nav2).

Pipeline:
  swarm_demo_goals → /swarm/robotN/goal
  pibt_planner     → collision-free /swarm/robotN/path + paths_ready
  adg_executor     → SMART ADG Type-2 release → /robotN/cmd_vel
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('dual_robot_known_map')

    use_rviz = LaunchConfiguration('use_rviz')
    headless = LaunchConfiguration('headless')
    use_demo_goals = LaunchConfiguration('use_demo_goals')
    demo_delay = LaunchConfiguration('demo_delay')

    lite = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'four_robots_lite.launch.py')
        ),
        launch_arguments={
            'use_rviz': use_rviz,
            'headless': headless,
        }.items(),
    )

    # Grid size / min_sep / ADG quant are derived from robot_radius + map
    # resolution (see mapf_grid_params.py). Only body geometry is configured.
    body = {
        'robot_radius': 0.18,       # TB3 waffle ≈ 0.18–0.21 m
        'planning_margin': 0.08,    # tracking / ADG slack
        'extra_sep_cells': 1,       # Chebyshev pad beyond geometric minimum
    }

    pibt = Node(
        package='dual_robot_known_map',
        executable='pibt_planner.py',
        name='pibt_planner',
        parameters=[{
            'use_sim_time': True,
            'map_topic': '/robot1/map',
            **body,
            'grid_stride': 0,   # 0 = auto
            'min_sep': -1,      # -1 = auto
            'max_steps': 2000,
        }],
        output='screen',
    )

    follower = Node(
        package='dual_robot_known_map',
        executable='adg_executor.py',
        name='adg_executor',
        parameters=[{
            'use_sim_time': True,
            'linear_speed': 0.22,
            'angular_speed': 1.2,
            'goal_tol': 0.12,
            'pose_stride': 1,
            'tick_hz': 20.0,
            **body,
            'cell_quant': 0.0,       # 0 = follow /swarm/mapf_grid
            'neighbor_buffer': -1,   # -1 = follow planner
            'lookahead_actions': 1,
            'hold_sec': 0.40,
            'clear_start_frac': 0.60,
        }],
        output='screen',
    )

    demo = Node(
        package='dual_robot_known_map',
        executable='swarm_demo_goals.py',
        name='swarm_demo_goals',
        parameters=[{
            'use_sim_time': True,
            'delay_sec': demo_delay,
            'repeat': True,
            'replan_pause_sec': 2.0,
        }],
        condition=IfCondition(use_demo_goals),
        output='screen',
    )

    # Warm-up: Gazebo + map lifecycle
    delayed = TimerAction(period=10.0, actions=[pibt, follower, demo])

    ld = LaunchDescription()
    # headless:=True = Gazebo server only (much more stable sim clock).
    # Watch robots in RViz; set headless:=False only if you need the Gazebo GUI.
    ld.add_action(DeclareLaunchArgument('use_rviz', default_value='True'))
    ld.add_action(DeclareLaunchArgument('headless', default_value='True'))
    ld.add_action(DeclareLaunchArgument('use_demo_goals', default_value='True'))
    ld.add_action(DeclareLaunchArgument('demo_delay', default_value='20.0'))
    ld.add_action(lite)
    ld.add_action(delayed)
    return ld
