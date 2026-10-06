# Technical Document: Multi-Robot House Mission Stack

**Repo:** [Asparsix/multi-robot-slam](https://github.com/Asparsix/multi-robot-slam)  
**Stack:** ROS 2 Jazzy · Gazebo Harmonic · Nav2 · slam_toolbox · CBBA · prioritized MAPF  

This document explains **what problem we solve**, **what exists in the system**, and **exactly which files implement each piece**.

---

## 1. Problem statement

We want **four TurtleBot3 robots** in one simulated house to:

1. **Map** the environment collaboratively (or reuse a saved map).
2. **Localize** each robot on that map.
3. **Assign** a pool of reach-point tasks to robots (multi-robot task allocation).
4. **Plan** motion so robots do not collide in space–time (multi-agent path finding).
5. **Execute** those paths so robots actually arrive at every assigned task.

Classical single-robot Nav2 only solves (2)+(partial 5) for one agent. It does **not** assign tasks across a fleet or guarantee non-conflicting multi-robot plans. This stack adds **CBBA** (who does which tasks) and **MAPF** (how they move without fighting), then uses **Nav2** as the low-level driver.

---

## 2. What we are solving (layered)

| Layer | Question | Approach in this repo |
|-------|----------|------------------------|
| **L0 Simulation** | How do 4 robots exist in one world? | Gazebo house world + namespaced bridges |
| **L1 Mapping** | How do we get a shared map? | Collaborative slam_toolbox → `house_collab` map |
| **L2 Localization / TF** | Where is each robot in `map`? | AMCL `map→odom` + odom TF per namespace |
| **L3 Task allocation** | Who does which tasks, in what order? | **CBBA** (`eticbba`), bundle capacity K |
| **L4 Multi-robot path planning** | How to avoid collisions between robots? | **Prioritized MAPF** (space–time A*) |
| **L5 Execution** | How do wheels follow the plan? | Timestep **Nav2 `NavigateToPose`** executor |

**End-to-end mission we verified:** CBBA assigns tasks → MAPF each bundle leg → all robots finish every assigned task (including leftovers so all **10** house tasks complete).

---

## 3. System overview

```text
┌─────────────────────────────────────────────────────────────────┐
│  Gazebo house world (4 TB3s)                                     │
│  /clock · /robotN/odom · /robotN/scan · /robotN/cmd_vel          │
└───────────────────────────────┬─────────────────────────────────┘
                                │
┌───────────────────────────────▼─────────────────────────────────┐
│  Per-robot Nav2 stack (/robotN/…)                                │
│  map_server · AMCL · planner · controller · bt_navigator         │
│  TF: map → odom (AMCL) → base_footprint (prefix_odom_tf)         │
└───────────────────────────────┬─────────────────────────────────┘
                                │
┌───────────────────────────────▼─────────────────────────────────┐
│  Mission layer                                                   │
│  CBBA ──► /swarm/robotN/goal ──► MAPF ──► /swarm/robotN/path     │
│                              └──► timestep executor ──► Nav2     │
└─────────────────────────────────────────────────────────────────┘
```

### Packages

| Package | Path | Responsibility |
|---------|------|----------------|
| `dual_robot_known_map` | `src/dual_robot_known_map/` | Sim, SLAM launches, Nav2 configs, MAPF, executor, TF helpers |
| `eticbba` | `src/eticbba/` | CBBA solver, Nav2 path cost, full mission coordinator |

---

## 4. Codebase map (by concern)

### 4.1 Simulation & bringup

| File | Role |
|------|------|
| `src/dual_robot_known_map/worlds/house_rooms.sdf` | ~12×12 m house world |
| `src/dual_robot_known_map/launch/four_robots_nav2.launch.py` | Spawn 4 robots, bridges, RSP, odom TF, **4× Nav2** |
| `src/dual_robot_known_map/launch/four_robots_collab_slam.launch.py` | Collaborative SLAM bringup |
| `src/dual_robot_known_map/config/bridge_no_clock.yaml` | Per-robot GZ↔ROS bridge (no duplicate `/clock`) |
| `src/dual_robot_known_map/maps/house_collab.{yaml,pgm}` | Saved known map used for Nav2/MAPF |

### 4.2 Localization / TF (L2)

| File | Role |
|------|------|
| `src/dual_robot_known_map/config/nav2_robot{1..4}.yaml` | Nav2 params; **AMCL `tf_broadcast: true`**, spawn `initial_pose` |
| `src/dual_robot_known_map/scripts/prefix_odom_tf.py` | Publishes **`odom → base_footprint`** from Gazebo odom (odom timestamps) |
| `src/dual_robot_known_map/scripts/peer_tf_relay.py` | Relays peer TF into robot1 tree for single-RViz view |
| `src/dual_robot_known_map/scripts/peer_scan_relay.py` | Relays peer scans for visualization |

**Design rule:** one localization TF source per robot — **AMCL owns `map→odom`**. No static `map→odom` publisher (removed from `four_robots_nav2.launch.py` because it fought AMCL).

### 4.3 Task allocation — CBBA (L3)

| File | Role |
|------|------|
| `src/eticbba/eticbba/cbba_solver.py` | Core **CBBA** algorithm (bundles, consensus) |
| `src/eticbba/eticbba/nav2_path_cost.py` | Cost oracle: Nav2 `ComputePathToPose` length; Euclidean fallback if planner times out |
| `src/eticbba/eticbba/cbba_assign_node.py` | ROS node: load tasks, run CBBA, write YAML, optionally publish first goals |
| `src/eticbba/tasks/house_10_tasks.yaml` | **10** reach tasks + robot spawns + **K** (bundle capacity) |
| `src/eticbba/launch/cbba_house_assign.launch.py` | Assign-only launch |

**What CBBA solves:** given robots and tasks, produce ordered **bundles** per robot that maximize score (prefer more tasks / shorter travel).

**Important:** CBBA is *assignment*. It does **not** move robots. But **scoring** currently calls Nav2 path length many times, which can time out under 4× Nav2 load — that is why you saw “Nav2 timeout during CBBA.” Fallback Euclidean cost keeps assignment alive.

**K and 10 tasks:** With K=2 and 4 robots, at most 8 tasks fit in bundles. Leftovers (e.g. T03, T04) are attached in the mission coordinator so all 10 can still be executed.

### 4.4 Multi-agent path finding — MAPF (L4)

| File | Role |
|------|------|
| `src/dual_robot_known_map/scripts/prioritized_mapf.py` | Prioritized space–time A* + reservation table |

**Algorithm (short):**

1. Robots planned in fixed priority: `robot1 > robot2 > robot3 > robot4`.
2. Higher-priority path is reserved in **(x, y, t)** and on edges.
3. Lower-priority robot A*s around those reservations (can wait in place).
4. Output path keeps **wait cells** (one pose per timestep) for space–time sync.

**I/O:**

- In: `/swarm/robotN/goal`, `/robot1/map`, `/robotN/tf`
- Out: `/swarm/robotN/path`, `/swarm/paths_ready`

This is **prioritized MAPF**, not optimal CBS/ECBS.

### 4.5 Execution — timestep Nav2 (L5)

| File | Role |
|------|------|
| `src/dual_robot_known_map/scripts/mrpa_executor.py` | Timestep executor (mapf_ros `plan_executor` style) |

**Why not one `NavigateThroughPoses`?** Dumping the whole MAPF path into Nav2 lets robots free-run and break space–time sync. Instead:

```text
for t in 0 .. makespan-1:
    send NavigateToPose(pose[t]) to each robot
    wait until all are near (or wall-clock step timeout)
    advance
```

Publishes `/swarm/makespan_complete` when a plan finishes.

### 4.6 Full mission orchestration

| File | Role |
|------|------|
| `src/eticbba/eticbba/cbba_mapf_mission.py` | **CBBA → leftover attach → MAPF leg loop → verify arrivals** |
| `src/eticbba/launch/cbba_mapf_mission.launch.py` | Brings up swarm Nav2 (no demo goals) + delayed mission node |
| `src/dual_robot_known_map/launch/four_robots_swarm_nav.launch.py` | Nav2 + MAPF + executor + optional corner-swap demo goals |
| `src/dual_robot_known_map/scripts/swarm_demo_goals.py` | Demo goals only (not used in full CBBA mission) |

**Mission leg loop:**

```text
CBBA(bundles)
attach any unassigned leftovers to nearest/shortest robots
for leg = 0 .. max_bundle_length-1:
    publish /swarm/*/goal for this leg (idle robots hold pose)
    wait paths_ready
    wait makespan_complete or TF near goals
    mark COMPLETED task ids
assert all 10 tasks done  →  ALL_TEN_TASKS_OK
```

### 4.7 Supporting / test utilities

| File | Role |
|------|------|
| `src/dual_robot_known_map/scripts/cbba_nav_smoke_test.py` | Sequential/parallel Nav2 motion smoke tests |
| `src/dual_robot_known_map/scripts/drive_four_demo.py` | Open-loop drive for SLAM demos |
| `docs/SWARM_MAPF.md` | Shorter swarm/MAPF runbook |

---

## 5. End-to-end data flow

```text
house_10_tasks.yaml
        │
        ▼
cbba_mapf_mission / cbba_solver
        │  bundles per robot
        ▼
/swarm/robotN/goal          (PoseStamped, one goal per leg)
        │
        ▼
prioritized_mapf
        │
        ├── /swarm/robotN/path       (nav_msgs/Path)
        └── /swarm/paths_ready       (Bool)
                │
                ▼
        mrpa_executor
                │
                ├── /robotN/navigate_to_pose   (Nav2 action, per timestep)
                └── /swarm/makespan_complete
                        │
                        ▼
                TF check near task → COMPLETED
```

### Topics / actions cheat sheet

| Name | Type | Producer → Consumer |
|------|------|---------------------|
| `/swarm/robotN/goal` | `PoseStamped` | mission/CBBA/demo → MAPF |
| `/swarm/robotN/path` | `Path` | MAPF → executor / RViz |
| `/swarm/paths_ready` | `Bool` | MAPF → executor / mission |
| `/swarm/makespan_complete` | `Bool` | executor → mission |
| `/robotN/navigate_to_pose` | action | executor → Nav2 |
| `/robotN/compute_path_to_pose` | action | CBBA cost → Nav2 planner |
| `/robotN/tf`, `/robotN/tf_static` | TF | AMCL + odom_tf + RSP |

---

## 6. What “working” means (verified behavior)

| Capability | Status | Evidence / notes |
|------------|--------|------------------|
| Collaborative SLAM / known map | Implemented | `house_collab` map + SLAM launches |
| 4× Nav2 + AMCL TF | Working | Full `map→odom→base` smoke |
| Corner-swap MAPF + drive | Working | `Makespan complete` on swarm demo |
| CBBA assignment (Nav2/Euclidean cost) | Working | YAML bundles written |
| Full mission all **10** tasks | Working | `all_ten_tasks_done: true` / `ALL_TEN_TASKS_OK` |

Example successful assignment shape (illustrative):

```text
robot1: T02 → T01 → T03
robot2: T06 → T07 → T04
robot3: T08 → T10
robot4: T09 → T05
```

---

## 7. Design decisions & trade-offs

| Decision | Why | Trade-off |
|----------|-----|-----------|
| Namespaced TF per robot | Matches Nav2 multi-robot practice; avoids one giant shared TF tree | Swarm nodes must subscribe per-robot TF |
| AMCL owns `map→odom` | Single localization source; avoids static TF fights | Needs good initial pose / scan sync |
| Prioritized MAPF | Simple, fast, good enough for house demo | Not optimal; priority bias |
| Timestep `NavigateToPose` | Preserves MAPF time sync better than `NavigateThroughPoses` | Slower; many Nav2 goals |
| Nav2 as CBBA cost | Path length reflects real map obstacles | Planner load → timeouts; Euclidean fallback |
| K=2 + leftover attach | Stable CBBA under load; still covers all 10 tasks | Leftovers are greedy, not re-auctioned |

---

## 8. How to run (canonical)

### Full CBBA → MAPF → finish all tasks

```bash
source /opt/ros/jazzy/setup.bash
source ~/nav2_ws/install/setup.bash
source ~/multi_robot_slam/install/setup.bash
export FASTRTPS_DEFAULT_PROFILES_FILE=~/nav2_ws/fastdds_no_shm.xml

ros2 launch eticbba cbba_mapf_mission.launch.py headless:=True use_rviz:=False
```

Expect: `/tmp/cbba_mapf_mission_result.yaml` with `success: true` and `all_ten_tasks_done: true`.

### MAPF demo only (corner swap)

```bash
ros2 launch dual_robot_known_map four_robots_swarm_nav.launch.py
```

### Nav2 only

```bash
ros2 launch dual_robot_known_map four_robots_nav2.launch.py
```

---

## 9. Known limitations

1. **Four Nav2 stacks on one PC** stress DDS/CPU (TF blips, planner timeouts, step timeouts).
2. **CBBA ≠ motion** — timeouts during CBBA are usually cost-query overload, not driving failure.
3. **Prioritized MAPF** is suboptimal vs CBS/ECBS.
4. **Step timeouts** in the executor advance anyway; sync is soft under load.
5. Optional future: replace Nav2 execution with a thin tracker (Option B) to cut load.

---

## 10. Quick “where is X?” index

| Concept | Primary code |
|---------|----------------|
| CBBA math | `eticbba/cbba_solver.py` |
| CBBA + Nav2 cost | `eticbba/nav2_path_cost.py`, `cbba_assign_node.py` |
| Task list | `eticbba/tasks/house_10_tasks.yaml` |
| Full mission | `eticbba/cbba_mapf_mission.py` |
| MAPF | `dual_robot_known_map/scripts/prioritized_mapf.py` |
| Path execution | `dual_robot_known_map/scripts/mrpa_executor.py` |
| Nav2 / AMCL config | `dual_robot_known_map/config/nav2_robot*.yaml` |
| Odom TF | `dual_robot_known_map/scripts/prefix_odom_tf.py` |
| 4-robot Nav2 launch | `dual_robot_known_map/launch/four_robots_nav2.launch.py` |
| Swarm launch | `dual_robot_known_map/launch/four_robots_swarm_nav.launch.py` |
| Mission launch | `eticbba/launch/cbba_mapf_mission.launch.py` |

---

## 11. Related docs

- [README.md](../README.md) — user-facing overview and run commands  
- [SWARM_MAPF.md](SWARM_MAPF.md) — shorter MAPF/TF runbook  
- [TECHNICAL_OVERVIEW.tex](TECHNICAL_OVERVIEW.tex) — LaTeX version of this document  
- [eticbba/README.md](../src/eticbba/README.md) — CBBA package notes  
