#!/usr/bin/env python3
"""Four-robot Macenski collaborative SLAM in a warehouse + one RViz.

- World: Fuel OpenRobotics warehouse (shelves/obstacles)
- Each robot: decentralized_multirobot_slam_toolbox_node
- Share scans on /localized_scan
- Common global_odom (static global_odom -> odom at spawn)
- Namespaced TF (/robotN/tf); peer relays bring robot2/3/4 into robot1 tree
- One RViz on robot1 TF + /robot1/map
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
    EmitEvent,
    ExecuteProcess,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch.events import matches_action
from launch.substitutions import Command, LaunchConfiguration
from launch.substitutions.find_executable import FindExecutable
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


# Spaced on open warehouse floor (away from shelf at ~0.4,-2 and person at 1,-1).
ROBOTS = [
    {'name': 'robot1', 'x': -3.0, 'y': 0.0, 'yaw': 0.0},
    {'name': 'robot2', 'x': 3.0, 'y': 0.0, 'yaw': math.pi},
    {'name': 'robot3', 'x': -3.0, 'y': 5.0, 'yaw': 0.0},
    {'name': 'robot4', 'x': 3.0, 'y': 5.0, 'yaw': math.pi},
]

TF_REMAPS = [('/tf', 'tf'), ('/tf_static', 'tf_static')]


def _add_collab_slam(ld, pkg, robot_name):
    params = os.path.join(pkg, 'config', f'collab_slam_{robot_name}.yaml')
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
                'max_laser_range': 20.0,
            },
        ],
        remappings=[
            ('/map', 'map'),
            ('/map_metadata', 'map_metadata'),
            ('/map_updates', 'map_updates'),
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
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


def _peer_nodes(peer_name: str):
    """Relay peer TF + scan into robot1 tree for one-RViz viewing."""
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
                'shared_frames': ['global_odom'],
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

    use_rviz = LaunchConfiguration('use_rviz')
    headless = LaunchConfiguration('headless')
    world = LaunchConfiguration('world')

    # Plain SDF (not xacro): copy to a temp path so we can clean up safely.
    world_sdf = tempfile.mktemp(prefix='four_collab_', suffix='.sdf')

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

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg, 'rviz', 'four_robots_collab.rviz')],
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
        default_value=os.path.join(pkg, 'worlds', 'warehouse_collab.sdf'),
    ))

    ld.add_action(set_env)
    ld.add_action(set_env2)
    ld.add_action(start_gz)
    ld.add_action(cleanup)
    ld.add_action(clock_bridge)
    for n in robot_nodes:
        ld.add_action(n)
    for r in ROBOTS:
        _add_collab_slam(ld, pkg, r['name'])
    for peer in ('robot2', 'robot3', 'robot4'):
        for n in _peer_nodes(peer):
            ld.add_action(n)
    ld.add_action(rviz)
    return ld
