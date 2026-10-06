# Four-robot swarm: MAPF + timestep Nav2 execution

This document describes the multi-robot planning and execution stack on the
saved house map (`house_collab`).

## Pipeline

```text
Goals (/swarm/robotN/goal)
        │
        ▼
prioritized_mapf          space-time A* (priority robot1 > … > robot4)
        │                 publishes /swarm/robotN/path
        │                 /swarm/paths_ready = true when all succeed
        ▼
mrpa_executor             timestep executor (mapf_ros plan_executor style)
        │                 for t in 0..makespan-1:
        │                   NavigateToPose(pose[t]) for each robot
        │                   wait until all near (or wall-clock timeout)
        ▼
Nav2 per robot            /robotN/navigate_to_pose → controller
```

Optional upstream: **eticbba** assigns house tasks and publishes the first
goal per robot on `/swarm/robotN/goal` (same topic the MAPF node listens to).

## TF architecture (Nav2 multi-robot pattern)

Each robot has an **isolated** TF tree on `/robotN/tf` + `/robotN/tf_static`:

| Transform | Publisher | Notes |
|-----------|-----------|--------|
| `map → odom` | **AMCL** (`tf_broadcast: true`) | Sole localization TF source; spawn `initial_pose` set in `nav2_robotN.yaml` |
| `odom → base_footprint` | `prefix_odom_tf` | From Gazebo odom; stamps use **odom header time** (not `now()`) |
| body frames | `robot_state_publisher` | URDF links on the namespaced TF topics |

Do **not** also publish a static `map → odom` — that fights AMCL and reconnects poorly under DDS load.

RViz (optional) remaps to `/robot1/tf`; `peer_tf_relay` copies peer body frames for a shared view.

## Nodes

### `prioritized_mapf.py`

- Subscribes: `/robot1/map`, `/swarm/{robot}/goal`, per-robot `/tf` + `/tf_static`
- Plans robots in fixed priority order with a reservation table (vertex + edge conflicts)
- Publishes: `/swarm/{robot}/path` (`nav_msgs/Path`, one pose per MAPF timestep including waits)
- Signals: `/swarm/paths_ready` (`std_msgs/Bool`)

### `mrpa_executor.py` (timestep executor)

Inspired by [mapf_ros `plan_executor`](https://github.com/speedzjy/mapf_ros):

- Waits for `/swarm/paths_ready`
- Subsamples poses with `pose_stride` (default 2)
- Sends `nav2_msgs/action/NavigateToPose` per step
- Arrival check via TF `map → base_footprint`
- Step timeout uses **wall clock** so a TF blip cannot hang forever

### `swarm_demo_goals.py`

Corner-swap demo goals (robot1 ↔ NE, robot2 ↔ NW, etc.) after `demo_delay` seconds.

## Topics / actions

| Name | Type | Role |
|------|------|------|
| `/swarm/robotN/goal` | `geometry_msgs/PoseStamped` | Goal input to MAPF |
| `/swarm/robotN/path` | `nav_msgs/Path` | Planned path |
| `/swarm/paths_ready` | `std_msgs/Bool` | Batch ready |
| `/robotN/navigate_to_pose` | action | Timestep goals |
| `/robotN/compute_path_to_pose` | action | Used by CBBA cost |

## Run

```bash
source /opt/ros/jazzy/setup.bash
source ~/multi_robot_slam/install/setup.bash   # or nav2_ws overlay

export FASTRTPS_DEFAULT_PROFILES_FILE=~/nav2_ws/fastdds_no_shm.xml   # recommended
export ROS_DOMAIN_ID=42   # optional isolation

ros2 launch dual_robot_known_map four_robots_swarm_nav.launch.py \
  headless:=False use_rviz:=True demo_delay:=30.0
```

Headless smoke:

```bash
ros2 launch dual_robot_known_map four_robots_swarm_nav.launch.py \
  headless:=True use_rviz:=False
```

Expect logs like:

```text
paths_ready=True (4/4)
New plan: makespan=59 …
Step 0/58 reached by all agents
…
Makespan complete — all step goals done
```

## CBBA then MAPF (full mission)

One-shot launch that **assigns → MAPF each bundle leg → verifies every assigned task**:

```bash
source /opt/ros/jazzy/setup.bash
source ~/nav2_ws/install/setup.bash          # dual_robot_known_map
source ~/multi_robot_slam/install/setup.bash # eticbba
export FASTRTPS_DEFAULT_PROFILES_FILE=~/nav2_ws/fastdds_no_shm.xml

ros2 launch eticbba cbba_mapf_mission.launch.py headless:=True use_rviz:=False
```

Success criteria: `/tmp/cbba_mapf_mission_result.yaml` has `success: true` and log line `ALL_ASSIGNED_TASKS_OK`.  
With K=2 and 4 robots, up to 8 of 10 tasks are assigned; the rest stay in `unassigned_task_ids`.

Manual two-step (assign only, or demo goals):

1. `ros2 launch dual_robot_known_map four_robots_swarm_nav.launch.py use_demo_goals:=False`
2. `ros2 launch eticbba cbba_house_assign.launch.py`

See `src/eticbba/README.md`.

## Known limitations

- Four full Nav2 stacks on one machine stress DDS; use FastDDS no-SHM profile and prefer headless for CI-like runs.
- Some MAPF steps may hit the wall-clock timeout if Nav2 is slow; the executor advances anyway so the demo finishes.
- Prioritized MAPF is not optimal (not ECBS/CBS); higher-priority robots win reservations.
