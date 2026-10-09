#!/usr/bin/env python3
"""4 robots: PIBT + SMART ADG + CBBA multi-task mission (no Nav2, no swap demo).

Pipeline:
  cbba_lite_mission  → assigns up to 20 tasks (K from tasks YAML)
                     → each robot advances its own queue; replan MAPF on any completion
  pibt_planner       → /swarm/robotN/path + paths_ready
  adg_executor       → SMART ADG → /robotN/cmd_vel
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
    etic_pkg = get_package_share_directory('eticbba')

    use_rviz = LaunchConfiguration('use_rviz')
    headless = LaunchConfiguration('headless')
    mission_delay = LaunchConfiguration('mission_delay')
    tasks_file = LaunchConfiguration('tasks_file')

    lite = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(dual_pkg, 'launch', 'four_robots_lite.launch.py')
        ),
        launch_arguments={
            'use_rviz': use_rviz,
            'headless': headless,
        }.items(),
    )

    body = {
        'robot_radius': 0.18,
        'planning_margin': 0.08,
        'extra_sep_cells': 1,
    }

    pibt = Node(
        package='dual_robot_known_map',
        executable='pibt_planner.py',
        name='pibt_planner',
        parameters=[{
            'use_sim_time': True,
            'map_topic': '/robot1/map',
            **body,
            'grid_stride': 0,
            'min_sep': -1,
            'max_steps': 2000,
        }],
        output='screen',
    )

    adg = Node(
        package='dual_robot_known_map',
        executable='adg_executor.py',
        name='adg_executor',
        parameters=[{
            'use_sim_time': True,
            'linear_speed': 0.22,
            'angular_speed': 1.2,
            'goal_tol': 0.18,
            'pose_stride': 1,
            'tick_hz': 20.0,
            **body,
            'cell_quant': 0.0,
            'neighbor_buffer': -1,
            'lookahead_actions': 1,
            'hold_sec': 0.30,
            'clear_start_frac': 0.50,
        }],
        output='screen',
    )

    default_tasks = os.path.join(etic_pkg, 'tasks', 'house_20_tasks.yaml')
    mission = Node(
        package='eticbba',
        executable='cbba_lite_mission',
        name='cbba_lite_mission',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'tasks_file': tasks_file,
            'output_file': '/tmp/house_20_cbba_assignment.yaml',
            'result_file': '/tmp/cbba_pibt_mission_result.yaml',
            'goal_tolerance_xy': 0.45,
            'paths_ready_timeout_sec': 120.0,
            'leg_timeout_sec': 400.0,
            'task_timeout_sec': 400.0,
            'mission_timeout_sec': 2400.0,
            'replan_retries': 4,
            'tf_wait_sec': 50.0,
        }],
    )

    delayed_stack = TimerAction(period=10.0, actions=[pibt, adg])
    delayed_mission = TimerAction(period=mission_delay, actions=[mission])

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='True'),
        DeclareLaunchArgument('headless', default_value='True'),
        DeclareLaunchArgument('mission_delay', default_value='22.0'),
        DeclareLaunchArgument('tasks_file', default_value=default_tasks),
        lite,
        delayed_stack,
        delayed_mission,
    ])
