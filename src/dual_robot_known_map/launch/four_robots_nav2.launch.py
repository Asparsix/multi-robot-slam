#!/usr/bin/env python3
"""Four TB3s in house world + known map + Nav2 per robot + one RViz.

Uses saved collaborative map (house_collab). No SLAM — AMCL localizes
each robot; each namespace runs a full Nav2 stack.
"""

import math
import os
import shutil
import tempfile
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch.substitutions.find_executable import FindExecutable
from launch_ros.actions import Node


ROBOTS = [
    {'name': 'robot1', 'x': -3.0, 'y': -3.0, 'yaw': 0.0},
    {'name': 'robot2', 'x': 3.0, 'y': -3.0, 'yaw': math.pi},
    {'name': 'robot3', 'x': -3.0, 'y': 3.0, 'yaw': 0.0},
    {'name': 'robot4', 'x': 3.0, 'y': 3.0, 'yaw': math.pi},
]

TF_REMAPS = [('/tf', 'tf'), ('/tf_static', 'tf_static')]


def _peer_nodes(peer_name: str):
    return [
        Node(
            package='dual_robot_known_map',
            executable='peer_tf_relay.py',
            name=f'peer_tf_relay_{peer_name}',
            parameters=[{
                'use_sim_time': True,
                'peer_tf_topic': f'/{peer_name}/tf',
                'peer_tf_static_topic': f'/{peer_name}/tf_static',
                'host_tf_topic': '/robot1/tf',
                'host_tf_static_topic': '/robot1/tf_static',
                'viz_prefix': peer_name,
                'shared_frames': ['map'],
                'relay_map_frames': True,
            }],
            output='screen',
        ),
        Node(
            package='dual_robot_known_map',
            executable='peer_scan_relay.py',
            name=f'peer_scan_relay_{peer_name}',
            parameters=[{
                'use_sim_time': True,
                'input_topic': f'/{peer_name}/scan',
                'output_topic': f'/{peer_name}/scan_viz',
                'frame_id': f'{peer_name}/base_scan',
            }],
            output='screen',
        ),
    ]


def generate_launch_description():
    pkg = get_package_share_directory('dual_robot_known_map')
    sim_dir = get_package_share_directory('nav2_minimal_tb3_sim')
    bringup_dir = get_package_share_directory('nav2_bringup')
    bringup_launch_dir = os.path.join(bringup_dir, 'launch')

    use_rviz = LaunchConfiguration('use_rviz')
    headless = LaunchConfiguration('headless')
    world = LaunchConfiguration('world')
    map_yaml = LaunchConfiguration('map')
    autostart = LaunchConfiguration('autostart')

    world_sdf = tempfile.mktemp(prefix='four_nav2_', suffix='.sdf')

    def _prepare_and_start_gz(context, *args, **kwargs):
        src = LaunchConfiguration('world').perform(context)
        shutil.copyfile(src, world_sdf)
        hl = LaunchConfiguration('headless').perform(context).lower() in (
            'true', '1', 'yes'
        )
        cmd = ['gz', 'sim', '-r', '-s', world_sdf] if hl else ['gz', 'sim', '-r', world_sdf]
        return [ExecuteProcess(cmd=cmd, output='screen')]

    start_gz = OpaqueFunction(function=_prepare_and_start_gz)
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

    # Must NOT use_sim_time — bridge is the /clock source (chicken-and-egg).
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        parameters=[{'use_sim_time': False}],
        output='screen',
    )

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
        robot_nodes.extend([
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
            ),
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
            ),
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                namespace=name,
                name='robot_state_publisher',
                parameters=[{
                    'use_sim_time': True,
                    'robot_description': robot_description,
                }],
                remappings=TF_REMAPS,
                output='screen',
            ),
            Node(
                package='dual_robot_known_map',
                executable='prefix_odom_tf.py',
                namespace=name,
                name='odom_tf',
                parameters=[{
                    'use_sim_time': True,
                    'odom_topic': 'odom',
                    'prefix': '',
                    'odom_frame': 'odom',
                    'base_frame': 'base_footprint',
                    'republish_hz': 20.0,
                }],
                remappings=TF_REMAPS,
                output='screen',
            ),
            # map->odom comes from AMCL (tf_broadcast: true in nav2_robot*.yaml).
            # Do not also publish a static map->odom — that fights AMCL and
            # breaks the one-localization-source-per-robot pattern.
        ])

    # Nav2 bringup per robot (AMCL + controller/planner/bt)
    nav_groups = []
    for r in ROBOTS:
        name = r['name']
        params = os.path.join(pkg, 'config', f'nav2_{name}.yaml')
        nav_groups.append(
            GroupAction([
                LogInfo(msg=f'Launching Nav2 for {name}'),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(bringup_launch_dir, 'bringup_launch.py')
                    ),
                    launch_arguments={
                        'namespace': name,
                        'use_namespace': 'True',
                        'slam': 'False',
                        'map': map_yaml,
                        'use_sim_time': 'True',
                        'params_file': params,
                        'autostart': autostart,
                        'use_composition': 'True',
                        'use_respawn': 'False',
                        'use_localization': 'True',
                    }.items(),
                ),
            ])
        )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg, 'rviz', 'four_robots_nav2.rviz')],
        parameters=[{'use_sim_time': True}],
        remappings=[
            ('/tf', '/robot1/tf'),
            ('/tf_static', '/robot1/tf_static'),
        ],
        condition=IfCondition(use_rviz),
        output='screen',
    )

    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument('use_rviz', default_value='True'))
    ld.add_action(DeclareLaunchArgument('headless', default_value='False'))
    ld.add_action(DeclareLaunchArgument('autostart', default_value='True'))
    ld.add_action(DeclareLaunchArgument(
        'world',
        default_value=os.path.join(pkg, 'worlds', 'house_rooms.sdf'),
    ))
    ld.add_action(DeclareLaunchArgument(
        'map',
        default_value=os.path.join(pkg, 'maps', 'house_collab.yaml'),
    ))

    ld.add_action(set_env)
    ld.add_action(set_env2)
    ld.add_action(start_gz)
    ld.add_action(cleanup)
    ld.add_action(clock_bridge)
    for n in robot_nodes:
        ld.add_action(n)
    for g in nav_groups:
        ld.add_action(g)
    for peer in ('robot2', 'robot3', 'robot4'):
        for n in _peer_nodes(peer):
            ld.add_action(n)
    ld.add_action(rviz)
    return ld
