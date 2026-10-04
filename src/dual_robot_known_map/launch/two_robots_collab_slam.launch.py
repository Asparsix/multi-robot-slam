#!/usr/bin/env python3
"""Macenski-style collaborative multi-robot SLAM + one RViz.

- Each robot: decentralized_multirobot_slam_toolbox_node
- Share scans on /localized_scan
- Common global_odom (static global_odom -> odom at spawn)
- Namespaced TF trees (/robotN/tf)
- One RViz on robot1 TF; display /robot1/map (the shared collaborative map)
- Robot2 scan shown via peer pose TF that slam publishes into robot1 tree
"""

import os
import tempfile
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.events import matches_action
from launch.substitutions import Command, LaunchConfiguration
from launch.substitutions.find_executable import FindExecutable
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


ROBOTS = [
    {'name': 'robot1', 'x': 0.5, 'y': 0.5, 'yaw': 0.0},
    {'name': 'robot2', 'x': -0.5, 'y': -0.5, 'yaw': 1.5707},
]

TF_REMAPS = [('/tf', 'tf'), ('/tf_static', 'tf_static')]


def _add_collab_slam(ld, pkg, robot_name):
    params = os.path.join(pkg, 'config', f'collab_slam_{robot_name}.yaml')
    # Critical params also set inline — yaml-only load was falling back to
    # defaults (scan_topic=/scan), so slam never built map -> global_odom.
    node = LifecycleNode(
        package='slam_toolbox',
        executable='decentralized_multirobot_slam_toolbox_node',
        name='slam_toolbox',
        namespace=robot_name,
        output='screen',
        parameters=[
            params,
            {
                'use_sim_time': True,
                'use_lifecycle_manager': False,
                'odom_frame': 'global_odom',
                'map_frame': 'map',
                'base_frame': 'base_footprint',
                'scan_topic': 'scan',
                'mode': 'mapping',
                'restamp_tf': True,
                'transform_publish_period': 0.02,
                'map_update_interval': 2.0,
            },
        ],
        remappings=[
            ('/map', 'map'),
            ('/map_metadata', 'map_metadata'),
            ('/map_updates', 'map_updates'),
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
            # Force relative scan even if a default absolute /scan sneaks in
            ('/scan', 'scan'),
        ],
    )
    configure = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(node),
            transition_id=Transition.TRANSITION_CONFIGURE,
        )
    )
    activate = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=node,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                LogInfo(msg=f'[LifecycleLaunch] {robot_name} collab slam activating.'),
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                ),
            ],
        )
    )
    ld.add_action(node)
    ld.add_action(configure)
    ld.add_action(activate)


def generate_launch_description():
    pkg = get_package_share_directory('dual_robot_known_map')
    sim_dir = get_package_share_directory('nav2_minimal_tb3_sim')

    use_rviz = LaunchConfiguration('use_rviz')
    headless = LaunchConfiguration('headless')
    world = LaunchConfiguration('world')

    # One Gazebo process only. Starting a separate `-g` client in parallel with
    # `-s` server races and can open extra empty worlds on each relaunch.
    world_sdf = tempfile.mktemp(prefix='dual_collab_', suffix='.sdf')
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

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        parameters=[{'use_sim_time': True}],
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
            # Namespaced TF tree (official multi-robot pattern)
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
                }],
                remappings=TF_REMAPS,
                output='screen',
            ),
            # Align each robot odom into shared global_odom (known spawn)
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                namespace=name,
                name='global_odom_to_odom',
                arguments=[
                    '--x', str(r['x']),
                    '--y', str(r['y']),
                    '--z', '0',
                    '--yaw', str(r['yaw']),
                    '--frame-id', 'global_odom',
                    '--child-frame-id', 'odom',
                ],
                parameters=[{'use_sim_time': True}],
                remappings=TF_REMAPS,
                output='screen',
            ),
        ])

    # Bring robot2's body TF into robot1's tree so one RViz shows both robots
    # immediately (does not wait for localized_scan peer poses).
    peer_tf = Node(
        package='dual_robot_known_map',
        executable='peer_tf_relay.py',
        name='peer_tf_relay',
        parameters=[{
            'use_sim_time': True,
            'peer_tf_topic': '/robot2/tf',
            'peer_tf_static_topic': '/robot2/tf_static',
            'host_tf_topic': '/robot1/tf',
            'host_tf_static_topic': '/robot1/tf_static',
            'viz_prefix': 'robot2',
            'shared_frames': ['global_odom'],
        }],
        output='screen',
    )
    peer_scan = Node(
        package='dual_robot_known_map',
        executable='peer_scan_relay.py',
        name='peer_scan_relay',
        parameters=[{
            'use_sim_time': True,
            'input_topic': '/robot2/scan',
            'output_topic': '/robot2/scan_viz',
            'frame_id': 'robot2/base_scan',
        }],
        output='screen',
    )

    # One RViz: robot1 TF tree + collaborative map from robot1
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg, 'rviz', 'two_robots_collab.rviz')],
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
    ld.add_action(DeclareLaunchArgument(
        'world',
        default_value=os.path.join(sim_dir, 'worlds', 'tb3_sandbox.sdf.xacro'),
    ))

    ld.add_action(set_env)
    ld.add_action(set_env2)
    ld.add_action(world_xacro)
    ld.add_action(start_gz_after_xacro)
    ld.add_action(cleanup)
    ld.add_action(clock_bridge)
    for n in robot_nodes:
        ld.add_action(n)
    _add_collab_slam(ld, pkg, 'robot1')
    _add_collab_slam(ld, pkg, 'robot2')
    ld.add_action(peer_tf)
    ld.add_action(peer_scan)
    ld.add_action(rviz)
    return ld
