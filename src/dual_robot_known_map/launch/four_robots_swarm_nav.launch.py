#!/usr/bin/env python3
"""4× Nav2 house stack + prioritized MAPF + timestep NavigateToPose executor + demo goals."""

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

    use_demo_goals = LaunchConfiguration('use_demo_goals')
    demo_delay = LaunchConfiguration('demo_delay')
    use_rviz = LaunchConfiguration('use_rviz')
    headless = LaunchConfiguration('headless')

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'four_robots_nav2.launch.py')
        ),
        launch_arguments={
            'use_rviz': use_rviz,
            'headless': headless,
        }.items(),
    )

    mapf = Node(
        package='dual_robot_known_map',
        executable='prioritized_mapf.py',
        name='prioritized_mapf',
        parameters=[{
            'use_sim_time': True,
            'map_topic': '/robot1/map',
            'robot_radius': 0.22,
        }],
        output='screen',
    )

    mrpa = Node(
        package='dual_robot_known_map',
        executable='mrpa_executor.py',
        name='mrpa_executor',
        parameters=[{
            'use_sim_time': True,
            # mapf_ros plan_executor style: one NavigateToPose per MAPF step,
            # wait until all robots are near, then advance.
            'mid_xy_tolerance': 0.40,
            'xy_goal_tolerance': 0.25,
            'yaw_goal_tolerance': 0.35,
            'pose_stride': 2,       # every 2nd MAPF cell (~0.2 m with stride=2 map)
            'step_timeout_sec': 45.0,
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
            'repeat': False,
        }],
        condition=IfCondition(use_demo_goals),
        output='screen',
    )

    # Start swarm layer after Nav2 has time to come up
    swarm_nodes = TimerAction(
        period=15.0,
        actions=[mapf, mrpa, demo],
    )

    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument('use_demo_goals', default_value='True'))
    ld.add_action(DeclareLaunchArgument('demo_delay', default_value='30.0'))
    ld.add_action(DeclareLaunchArgument('use_rviz', default_value='True'))
    ld.add_action(DeclareLaunchArgument('headless', default_value='False'))
    ld.add_action(nav2)
    ld.add_action(swarm_nodes)
    return ld
