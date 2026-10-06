# eticbba ROS 2 package

**CBBA** task allocation for the house demo, with **Nav2** `compute_path_to_pose` path length as the travel cost.

## Prerequisites

- Nav2 running (e.g. `four_robots_nav2.launch.py` or `four_robots_swarm_nav.launch.py`)
- `/robot1/compute_path_to_pose` action available (same map for all robots)

## Build

```bash
source /opt/ros/jazzy/setup.bash
cd ~/multi_robot_slam  # or your workspace containing src/eticbba
colcon build --packages-select eticbba --symlink-install
source install/setup.bash
```

## Run assignment

Terminal A — sim + Nav2:

```bash
ros2 launch dual_robot_known_map four_robots_nav2.launch.py
```

Terminal B — after planners are active:

```bash
ros2 launch eticbba cbba_house_assign.launch.py
```

Output: `/tmp/house_10_cbba_assignment.yaml` and first task goal per robot on `/swarm/robotN/goal`.

To **drive** the assigned goals with MAPF, run the swarm layer (or full swarm launch with demo goals off):

```bash
ros2 launch dual_robot_known_map four_robots_swarm_nav.launch.py use_demo_goals:=False
# then run this CBBA launch so goals feed prioritized_mapf + mrpa_executor
```

See [docs/SWARM_MAPF.md](../../docs/SWARM_MAPF.md).

## Tasks

See `tasks/house_10_tasks.yaml` — 10 reach points, **K=2** bundle cap per robot.

## Cost model

Each agent bids using Nav2 `ComputePathToPose` (`planner_id:=GridBased`) path length.
Bundle score = `10000 × |bundle| − path_length` so longer feasible bundles beat empty bids;
CBBA maximizes score (path length is minimized within that).
