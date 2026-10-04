#!/usr/bin/env python3
"""Two TB3s in one Gazebo world + known map + one RViz. No SLAM.

Pattern used by multi-robot demos that want a single RViz:
  - one /clock bridge for the world
  - per-robot sensor bridges (no clock)
  - prefixed frames on the global /tf tree (robot1/..., robot2/...)
  - map_server with a standard known map (tb3_sandbox)
  - static map -> robotN/odom at each spawn pose
"""

import os
import tempfile
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.substitutions import Command, LaunchConfiguration
from launch.substitutions.find_executable import FindExecutable
from launch.conditions import IfCondition
from launch_ros.actions import Node


ROBOTS = [
    {'name': 'robot1', 'x': 0.5, 'y': 0.5, 'yaw': 0.0},
    {'name': 'robot2', 'x': -0.5, 'y': -0.5, 'yaw': 1.5707},
]


def generate_launch_description():
    pkg = get_package_share_directory('dual_robot_known_map')
    sim_dir = get_package_share_directory('nav2_minimal_tb3_sim')
    bringup_dir = get_package_share_directory('nav2_bringup')

    use_rviz = LaunchConfiguration('use_rviz')
    headless = LaunchConfiguration('headless')
    world = LaunchConfiguration('world')
    map_yaml = LaunchConfiguration('map')

    # One Gazebo process only (avoid parallel `-g` client creating extra worlds).
    world_sdf = tempfile.mktemp(prefix='dual_map_', suffix='.sdf')
    world_xacro = ExecuteProcess(
        cmd=['xacro', '-o', world_sdf, ['headless:=', headless], world],
        output='screen',
    )

    def _start_gz(context, *args, **kwargs):
        hl = LaunchConfiguration('headless').perform(context).lower() in (
            'true', '1', 'yes'
        )
        cmd = ['gz', 'sim', '-r', '-s', world_sdf] if hl else ['gz', 'sim', '-r', world_sdf]
        return [ExecuteProcess(cmd=cmd, output='screen')]

    start_gz_after_xacro = RegisterEventHandler(
        OnProcessExit(
            target_action=world_xacro,
            on_exit=[OpaqueFunction(function=_start_gz)],
        )
    )
    cleanup = RegisterEventHandler(
        OnShutdown(
            on_shutdown=[
                OpaqueFunction(
                    function=lambda _: os.remove(world_sdf)
                    if os.path.exists(world_sdf)
                    else None
                )
            ]
        )
    )

    set_env = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', os.path.join(sim_dir, 'models')
    )
    set_env2 = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', str(Path(sim_dir).parent.resolve())
    )

    # One shared clock for the whole sim
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # Known map (standard Nav2 tb3_sandbox)
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'yaml_filename': map_yaml,
        }],
    )
    lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': ['map_server'],
        }],
    )

    # Upstream URDF points at models/*.dae, but the Jazzy package installs
    # meshes under models/turtlebot3_model/meshes/ — rewrite so RViz can load them.
    urdf_path = os.path.join(sim_dir, 'urdf', 'turtlebot3_waffle.urdf')
    with open(urdf_path, 'r', encoding='utf-8') as f:
        robot_description = f.read().replace(
            'package://nav2_minimal_tb3_sim/models/',
            'package://nav2_minimal_tb3_sim/models/turtlebot3_model/meshes/',
        )

    robot_sdf = os.path.join(sim_dir, 'urdf', 'gz_waffle.sdf.xacro')
    bridge_yaml = os.path.join(pkg, 'config', 'bridge_no_clock.yaml')

    robot_nodes = []
    for r in ROBOTS:
        name = r['name']
        prefix = f'{name}/'

        robot_nodes.append(
            Node(
                package='ros_gz_sim',
                executable='create',
                namespace=name,
                output='screen',
                arguments=[
                    '-name', name,
                    '-string', Command([
                        FindExecutable(name='xacro'), ' ',
                        'namespace:=', name, ' ', robot_sdf,
                    ]),
                    '-x', str(r['x']),
                    '-y', str(r['y']),
                    '-z', '0.01',
                    '-Y', str(r['yaw']),
                ],
            )
        )
        robot_nodes.append(
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                namespace=name,
                name='bridge',
                parameters=[{
                    'config_file': bridge_yaml,
                    'expand_gz_topic_names': True,
                    'use_sim_time': True,
                }],
                output='screen',
            )
        )
        # Global /tf with prefixed frames (needed for one RViz)
        robot_nodes.append(
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                namespace=name,
                name='robot_state_publisher',
                parameters=[{
                    'use_sim_time': True,
                    'robot_description': robot_description,
                    'frame_prefix': prefix,
                }],
                output='screen',
            )
        )
        robot_nodes.append(
            Node(
                package='dual_robot_known_map',
                executable='prefix_odom_tf.py',
                namespace=name,
                name='prefix_odom_tf',
                parameters=[{
                    'use_sim_time': True,
                    'odom_topic': 'odom',
                    'prefix': prefix,
                    'odom_frame': 'odom',
                    'base_frame': 'base_footprint',
                }],
                output='screen',
            )
        )
        robot_nodes.append(
            Node(
                package='dual_robot_known_map',
                executable='prefix_scan.py',
                namespace=name,
                name='prefix_scan',
                parameters=[{
                    'use_sim_time': True,
                    'input_topic': 'scan',
                    'output_topic': 'scan_viz',
                    'frame_id': f'{prefix}base_scan',
                }],
                output='screen',
            )
        )
        # Place each robot's odom on the known map at its spawn pose
        robot_nodes.append(
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name=f'{name}_map_odom',
                arguments=[
                    '--x', str(r['x']),
                    '--y', str(r['y']),
                    '--z', '0',
                    '--yaw', str(r['yaw']),
                    '--frame-id', 'map',
                    '--child-frame-id', f'{prefix}odom',
                ],
                parameters=[{'use_sim_time': True}],
                output='screen',
            )
        )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg, 'rviz', 'two_robots.rviz')],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(use_rviz),
        output='screen',
    )

    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument('use_rviz', default_value='True'))
    ld.add_action(DeclareLaunchArgument('headless', default_value='False'))
    ld.add_action(DeclareLaunchArgument(
        'world',
        default_value=os.path.join(sim_dir, 'worlds', 'tb3_sandbox.sdf.xacro'),
    ))
    ld.add_action(DeclareLaunchArgument(
        'map',
        default_value=os.path.join(bringup_dir, 'maps', 'tb3_sandbox.yaml'),
    ))

    ld.add_action(set_env)
    ld.add_action(set_env2)
    ld.add_action(world_xacro)
    ld.add_action(start_gz_after_xacro)
    ld.add_action(cleanup)
    ld.add_action(clock_bridge)
    ld.add_action(map_server)
    ld.add_action(lifecycle)
    for n in robot_nodes:
        ld.add_action(n)
    ld.add_action(rviz)
    return ld
