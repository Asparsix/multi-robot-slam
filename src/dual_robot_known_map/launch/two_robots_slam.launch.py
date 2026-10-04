#!/usr/bin/env python3
"""Two TB3s + independent SLAM each + one RViz (both robots visible).

Reuses the working one-RViz TF pattern (prefixed frames on global /tf),
then runs slam_toolbox per robot. Maps are aligned in a shared `map`
frame using known spawn poses (not collaborative slam_toolbox).
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
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.events import matches_action
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch.substitutions.find_executable import FindExecutable
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


ROBOTS = [
    {'name': 'robot1', 'x': 0.5, 'y': 0.5, 'yaw': 0.0},
    {'name': 'robot2', 'x': -0.5, 'y': -0.5, 'yaw': 1.5707},
]


def _add_slam(ld, pkg, robot_name):
    params = os.path.join(pkg, 'config', f'slam_{robot_name}.yaml')
    node = LifecycleNode(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace=robot_name,
        output='screen',
        parameters=[
            params,
            {'use_sim_time': True, 'use_lifecycle_manager': False},
        ],
        remappings=[
            # Keep TF on the shared global tree for one RViz
            ('/tf', '/tf'),
            ('/tf_static', '/tf_static'),
            # slam_toolbox uses absolute /map by default — force per-robot topics
            ('/map', 'map'),
            ('/map_metadata', 'map_metadata'),
            ('/map_updates', 'map_updates'),
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
                LogInfo(msg=f'[LifecycleLaunch] {robot_name} slam activating.'),
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

    world_sdf = tempfile.mktemp(prefix='dual_slam_', suffix='.sdf')
    world_xacro = ExecuteProcess(
        cmd=['xacro', '-o', world_sdf, ['headless:=', headless], world],
        output='screen',
    )
    start_gz = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-s', world_sdf],
        output='screen',
    )
    start_gz_after_xacro = RegisterEventHandler(
        OnProcessExit(target_action=world_xacro, on_exit=[start_gz])
    )
    start_gz_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py',
            )
        ),
        condition=IfCondition(PythonExpression(['not ', headless])),
        launch_arguments={'gz_args': ['-v4 -g ']}.items(),
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

    # Anchor robot maps into one shared `map` frame using known spawn poses.
    # robot1/map is the reference; robot2/map is offset by relative spawn.
    r1, r2 = ROBOTS[0], ROBOTS[1]
    align_nodes = [
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_robot1_map',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0', '--yaw', '0',
                '--frame-id', 'map',
                '--child-frame-id', 'robot1/map',
            ],
            parameters=[{'use_sim_time': True}],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_robot2_map',
            arguments=[
                '--x', str(r2['x'] - r1['x']),
                '--y', str(r2['y'] - r1['y']),
                '--z', '0',
                '--yaw', str(r2['yaw'] - r1['yaw']),
                '--frame-id', 'map',
                '--child-frame-id', 'robot2/map',
            ],
            parameters=[{'use_sim_time': True}],
        ),
    ]

    robot_nodes = []
    for r in ROBOTS:
        name = r['name']
        prefix = f'{name}/'

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
                    'frame_prefix': prefix,
                }],
                output='screen',
            ),
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
            ),
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
            ),
        ])

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg, 'rviz', 'two_robots_slam.rviz')],
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

    ld.add_action(set_env)
    ld.add_action(set_env2)
    ld.add_action(world_xacro)
    ld.add_action(start_gz_after_xacro)
    ld.add_action(start_gz_client)
    ld.add_action(cleanup)
    ld.add_action(clock_bridge)
    for n in align_nodes:
        ld.add_action(n)
    for n in robot_nodes:
        ld.add_action(n)
    _add_slam(ld, pkg, 'robot1')
    _add_slam(ld, pkg, 'robot2')
    ld.add_action(rviz)
    return ld
