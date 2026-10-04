# Multi-Robot Collaborative SLAM (ROS 2)

Two (or more) TurtleBot3 robots in **one Gazebo world**, **Macenski-style decentralized slam_toolbox**, and **one shared RViz** showing both robots and a collaborative map.

This is **not** the official “one RViz per robot” Nav2 demo. It wires the industry pattern (namespaced TF + `/localized_scan` sharing) into a single operator view.

## Features

- One Gazebo world, **one `/clock` bridge** (no dual-clock TF fighting)
- Per-robot sensor bridges (scan, odom, cmd_vel)
- **Decentralized multi-robot slam_toolbox** (`decentralized_multirobot_slam_toolbox_node`)
- Shared scan topic: `/localized_scan`
- Shared odometry frame: `global_odom` (static `global_odom → odom` at each spawn)
- **One RViz**: robot1 TF tree + peer TF relay so **both robots** appear
- Collaborative map on `/robot1/map` (both robots contribute when driven)

## Package

| Path | Role |
|------|------|
| `src/dual_robot_known_map` | Launch, bridges, relays, RViz configs |

### Launches

| Launch | Description |
|--------|-------------|
| `two_robots_collab_slam.launch.py` | **Main:** collaborative Macenski SLAM + one RViz |
| `two_robots_known_map.launch.py` | Known map only (no SLAM), both robots in one RViz |
| `two_robots_slam.launch.py` | Independent SLAM per robot (two maps overlaid) |

## Dependencies

- ROS 2 **Jazzy**
- Gazebo Harmonic (`ros_gz_sim`, `ros_gz_bridge`)
- `nav2_minimal_tb3_sim`, `nav2_map_server`, `nav2_lifecycle_manager`, `rviz2`, `tf2_ros`
- **slam_toolbox** built with the **decentralized multi-robot** node  
  (stock apt `slam_toolbox` on Jazzy may not ship `decentralized_multirobot_slam_toolbox_node`)

Build / overlay a slam_toolbox that includes:

```text
decentralized_multirobot_slam_toolbox_node
```

Example (if you already have that workspace):

```bash
source ~/slam_multi_ws/install/setup.bash
```

## Build

```bash
mkdir -p ~/multi_robot_slam_ws/src
cp -r src/dual_robot_known_map ~/multi_robot_slam_ws/src/
cd ~/multi_robot_slam_ws
source /opt/ros/jazzy/setup.bash
# also source your slam_toolbox overlay if needed
colcon build --packages-select dual_robot_known_map --symlink-install
source install/setup.bash
```

## Run (collaborative SLAM)

```bash
source /opt/ros/jazzy/setup.bash
source ~/slam_multi_ws/install/setup.bash   # decentralized slam_toolbox
source ~/multi_robot_slam_ws/install/setup.bash

ros2 launch dual_robot_known_map two_robots_collab_slam.launch.py
```

### Drive robots

```bash
# Robot 1
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r __ns:=/robot1 -p stamped:=false

# Robot 2 (other terminal)
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r __ns:=/robot2 -p stamped:=false
```

Use `stamped:=false` — the Gazebo bridge expects `geometry_msgs/Twist`, not `TwistStamped`.

Keys: `i` forward, `,` back, `j`/`l` turn, `k` stop.

### RViz tips

- Fixed frame: start with `global_odom`; switch to `map` once SLAM has published `map → global_odom`
- Shared map topic: `/robot1/map`
- Both robots should appear (robot2 via TF relay)

## Design notes (why this works)

1. **Single `/clock`** — only one process bridges Gazebo clock; per-robot bridges omit clock.
2. **Namespaced TF for SLAM** — each robot has `/robotN/tf` so both can publish `map → global_odom` without colliding.
3. **One RViz** — remapped to `/robot1/tf`; `peer_tf_relay` copies robot2 body frames in under `robot2/…`.
4. **Collaborative map** — robots exchange `LocalizedLaserScan` on `/localized_scan`; each slam instance maintains a global map (view `/robot1/map`).

## License

Apache-2.0 (package); TurtleBot / Nav2 assets remain under their upstream licenses.
