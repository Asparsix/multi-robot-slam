#!/usr/bin/env python3
"""mapf_ros-style plumbing + PIBT brain (no Nav2).

Subscribes to a shared OccupancyGrid and /swarm/robotN/goal, runs Priority
Inheritance with Backtracking (PIBT) for collision-free space-time paths,
publishes /swarm/robotN/path and /swarm/paths_ready.

Pose for starts: TF map→base_footprint, else spawn fallback.
"""

from __future__ import annotations

import math
import sys
from collections import deque
from pathlib import Path as FsPath
from typing import Dict, List, Optional, Set, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float64MultiArray
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer

_SCRIPTS = FsPath(__file__).resolve().parent
if str(_SCRIPTS) not in sys.path:
    sys.path.insert(0, str(_SCRIPTS))

from mapf_grid_params import compute_mapf_grid, shrink_downsample_for_connectivity  # noqa: E402


Cell = Tuple[int, int]
ROBOTS = ['robot1', 'robot2', 'robot3', 'robot4']


def _bfs_dist(goal: Cell, is_free, stride: int, width: int, height: int) -> Dict[Cell, int]:
    """Manhattan-on-stride distance field from goal over free cells."""
    dist: Dict[Cell, int] = {goal: 0}
    q: deque[Cell] = deque([goal])
    nbrs = [(stride, 0), (-stride, 0), (0, stride), (0, -stride)]
    while q:
        c, r = q.popleft()
        d = dist[(c, r)]
        for dc, dr in nbrs:
            nc, nr = c + dc, r + dr
            if nc < 0 or nr < 0 or nc >= width or nr >= height:
                continue
            if not is_free(nc, nr):
                continue
            nxt = (nc, nr)
            if nxt in dist:
                continue
            dist[nxt] = d + 1
            q.append(nxt)
    return dist


def _cheb(a: Cell, b: Cell) -> int:
    return max(abs(a[0] - b[0]), abs(a[1] - b[1]))


def pibt_solve(
    starts: List[Cell],
    goals: List[Cell],
    is_free,
    stride: int,
    width: int,
    height: int,
    max_steps: int,
    min_sep: int = 1,
) -> Optional[List[List[Cell]]]:
    """PIBT (Okumura-style) one-shot MAPF on a 4-connected stride grid.

    ``min_sep``: Chebyshev separation between agents at each timestep
    (1 ⇒ no shared or adjacent cells — needed when cell size ≈ body size).

    Returns paths[agent] = list of cells including start, or None on failure.
    """
    n = len(starts)
    if n == 0:
        return []
    for s, g in zip(starts, goals):
        if not is_free(*s) or not is_free(*g):
            return None

    dist_maps = [_bfs_dist(g, is_free, stride, width, height) for g in goals]
    for i, s in enumerate(starts):
        if s not in dist_maps[i]:
            return None

    # Farther-from-goal agents get higher priority (classic PIBT heuristic)
    order = sorted(range(n), key=lambda i: -dist_maps[i].get(starts[i], 0))

    nbrs = [(stride, 0), (-stride, 0), (0, stride), (0, -stride), (0, 0)]
    paths: List[List[Cell]] = [[s] for s in starts]
    cur = list(starts)
    sep = max(0, int(min_sep))

    def conflicts_occupied(v: Cell, occupied: Dict[Cell, int], ai: int) -> bool:
        for cell, other in occupied.items():
            if other == ai:
                continue
            if _cheb(v, cell) <= sep:
                return True
        return False

    for _t in range(max_steps):
        if all(cur[i] == goals[i] for i in range(n)):
            break

        decided: Dict[int, Cell] = {}
        occupied: Dict[Cell, int] = {}  # cell -> agent assigned next

        def prefer(ai: int, cell: Cell) -> Tuple[int, int]:
            # lower dist better; prefer moving over waiting when equal
            d = dist_maps[ai].get(cell, 10**9)
            wait = 1 if cell == cur[ai] else 0
            return (d, wait)

        def try_move(ai: int, forbidden: Set[Cell]) -> bool:
            """Assign next cell for ai; may push lower-priority agents."""
            cands = []
            c0, r0 = cur[ai]
            for dc, dr in nbrs:
                nc, nr = c0 + dc, r0 + dr
                if nc < 0 or nr < 0 or nc >= width or nr >= height:
                    continue
                if not is_free(nc, nr):
                    continue
                cands.append((nc, nr))
            cands.sort(key=lambda v: prefer(ai, v))

            for v in cands:
                if v in forbidden:
                    continue
                # Vertex / neighborhood conflict with already-decided agent
                if conflicts_occupied(v, occupied, ai):
                    continue
                # Edge conflict (swap) with already-decided agent
                swap_bad = False
                for bj, uj in decided.items():
                    if uj == cur[ai] and v == cur[bj] and bj != ai:
                        swap_bad = True
                        break
                if swap_bad:
                    continue

                # Someone currently inside sep of v and not yet decided → push
                pushees = []
                for bj in range(n):
                    if bj == ai or bj in decided:
                        continue
                    if _cheb(cur[bj], v) <= sep and cur[bj] != cur[ai]:
                        pushees.append(bj)

                push_ok = True
                blocked = set(forbidden)
                blocked.add(v)
                # Also reserve neighborhood of v so pushees leave the pad
                if sep >= 1:
                    for dc in range(-sep, sep + 1):
                        for dr in range(-sep, sep + 1):
                            blocked.add((v[0] + dc, v[1] + dr))
                for pushee in pushees:
                    if not try_move(pushee, blocked):
                        push_ok = False
                        break
                if not push_ok:
                    continue

                decided[ai] = v
                occupied[v] = ai
                return True

            return False

        ok = True
        for ai in order:
            if ai in decided:
                continue
            if not try_move(ai, set()):
                ok = False
                break

        if not ok:
            # Fallback: everyone waits (keeps reservation-safe)
            for ai in range(n):
                if ai not in decided:
                    decided[ai] = cur[ai]

        # Ensure full assignment
        for ai in range(n):
            if ai not in decided:
                decided[ai] = cur[ai]

        nxt = [decided[i] for i in range(n)]
        # Final separation repair
        for i in range(n):
            for j in range(i):
                if _cheb(nxt[i], nxt[j]) <= sep:
                    nxt[i] = cur[i]
                    nxt[j] = cur[j]

        for i in range(n):
            paths[i].append(nxt[i])
        cur = nxt

    if not all(cur[i] == goals[i] for i in range(n)):
        return None
    return paths


class PibtPlanner(Node):
    def __init__(self):
        super().__init__('pibt_planner')
        self.declare_parameter('robots', ROBOTS)
        self.declare_parameter('map_topic', '/robot1/map')
        # Physical inputs (grid is derived from these + map resolution)
        self.declare_parameter('robot_radius', 0.18)
        self.declare_parameter('planning_margin', 0.08)
        self.declare_parameter('extra_sep_cells', 1)
        # Overrides: 0 / -1 means auto from robot_radius + map res
        self.declare_parameter('grid_stride', 0)
        self.declare_parameter('min_sep', -1)
        self.declare_parameter('max_steps', 2000)
        self.declare_parameter('fallback_starts', [
            -3.0, -3.0, 3.0, -3.0, -3.0, 3.0, 3.0, 3.0,
        ])

        self.robots: List[str] = [str(r) for r in self.get_parameter('robots').value]
        fb = [float(x) for x in self.get_parameter('fallback_starts').value]
        self._fallback_starts = {
            self.robots[i]: (fb[2 * i], fb[2 * i + 1])
            for i in range(len(self.robots))
            if 2 * i + 1 < len(fb)
        }
        self.robot_radius = float(self.get_parameter('robot_radius').value)
        self.planning_margin = float(self.get_parameter('planning_margin').value)
        self.extra_sep_cells = max(0, int(self.get_parameter('extra_sep_cells').value))
        self._stride_override = int(self.get_parameter('grid_stride').value)
        self._sep_override = int(self.get_parameter('min_sep').value)
        self.max_steps = int(self.get_parameter('max_steps').value)
        # Filled on first map; placeholders until then
        self.downsample = 1
        self.min_sep = 0
        self._neighbor_buffer = 0

        self._map: Optional[OccupancyGrid] = None
        self._fine_grid: Optional[np.ndarray] = None
        self._grid: Optional[np.ndarray] = None  # coarse blocked grid
        self._coarse_res = 0.05
        self._coarse_ox = 0.0
        self._coarse_oy = 0.0
        self._coarse_w = 0
        self._coarse_h = 0
        self._goals: Dict[str, PoseStamped] = {}
        self._goal_dirty = False

        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.create_subscription(
            OccupancyGrid,
            self.get_parameter('map_topic').value,
            self._on_map,
            map_qos,
        )

        for name in self.robots:
            self.create_subscription(
                PoseStamped,
                f'/swarm/{name}/goal',
                lambda msg, n=name: self._on_goal(n, msg),
                10,
            )

        self._path_pubs = {
            name: self.create_publisher(Path, f'/swarm/{name}/path', 10)
            for name in self.robots
        }
        self._ready_pub = self.create_publisher(Bool, '/swarm/paths_ready', 10)
        # Latched: [coarse_res, min_sep, downsample, neighbor_buffer, clearance]
        grid_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        self._grid_pub = self.create_publisher(
            Float64MultiArray, '/swarm/mapf_grid', grid_qos
        )

        self._tf_buffers: Dict[str, Buffer] = {}
        static_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        for name in self.robots:
            buf = Buffer(cache_time=Duration(seconds=30.0))
            self._tf_buffers[name] = buf
            self.create_subscription(
                TFMessage, f'/{name}/tf',
                lambda msg, b=buf: self._feed_tf(b, msg, False), 100)
            self.create_subscription(
                TFMessage, f'/{name}/tf_static',
                lambda msg, b=buf: self._feed_tf(b, msg, True), static_qos)

        self.create_timer(0.5, self._tick)
        self.get_logger().info(
            f'PIBT planner ready robots={self.robots} '
            f'robot_radius={self.robot_radius:.3f} '
            f'planning_margin={self.planning_margin:.3f} '
            f'extra_sep_cells={self.extra_sep_cells} '
            f'(grid auto from map resolution; '
            f'stride_override={self._stride_override} sep_override={self._sep_override})'
        )

    @staticmethod
    def _feed_tf(buf: Buffer, msg: TFMessage, is_static: bool):
        for tf in msg.transforms:
            if is_static:
                buf.set_transform_static(tf, 'pibt_planner')
            else:
                buf.set_transform(tf, 'pibt_planner')

    def _on_map(self, msg: OccupancyGrid):
        self._map = msg
        self._fine_grid = self._inflate(msg)
        self._coarse_ox = float(msg.info.origin.position.x)
        self._coarse_oy = float(msg.info.origin.position.y)
        res = float(msg.info.resolution)

        starts_xy = [self._fallback_starts.get(n, (0.0, 0.0)) for n in self.robots]
        # Prefer live goals for connectivity shrink when available; else start↔start
        if len(self._goals) == len(self.robots):
            goals_xy = [
                (self._goals[n].pose.position.x, self._goals[n].pose.position.y)
                for n in self.robots
            ]
        else:
            goals_xy = list(starts_xy)

        ideal = compute_mapf_grid(
            res,
            self.robot_radius,
            planning_margin=self.planning_margin,
            extra_sep_cells=self.extra_sep_cells,
            grid_stride_override=self._stride_override,
            min_sep_override=self._sep_override,
        )
        params, coarse = shrink_downsample_for_connectivity(
            self._fine_grid,
            starts_xy,
            goals_xy,
            (self._coarse_ox, self._coarse_oy),
            res,
            self.robot_radius,
            planning_margin=self.planning_margin,
            extra_sep_cells=self.extra_sep_cells,
            grid_stride_override=self._stride_override,
            min_sep_override=self._sep_override,
        )
        if params.downsample < ideal.downsample:
            self.get_logger().warn(
                f'Coarse grid shrunk for connectivity: '
                f'downsample {ideal.downsample}→{params.downsample} '
                f'(cell {ideal.coarse_res:.3f}→{params.coarse_res:.3f}m)'
            )

        self.downsample = params.downsample
        self.min_sep = params.min_sep
        self._neighbor_buffer = params.neighbor_buffer
        self._grid = coarse
        self._coarse_res = params.coarse_res
        self._coarse_h, self._coarse_w = self._grid.shape
        self._publish_grid_params(params)
        self.get_logger().info(
            f'Map {msg.info.width}x{msg.info.height} res={res:.3f} → '
            f'coarse {self._coarse_w}x{self._coarse_h} cell={self._coarse_res:.3f}m '
            f'downsample={self.downsample} min_sep={self.min_sep} '
            f'clearance={params.target_clearance:.3f}m '
            f'(from robot_radius={self.robot_radius:.3f} + margin={self.planning_margin:.3f})'
        )

    def _publish_grid_params(self, params) -> None:
        msg = Float64MultiArray()
        msg.data = [
            float(params.coarse_res),
            float(params.min_sep),
            float(params.downsample),
            float(params.neighbor_buffer),
            float(params.target_clearance),
        ]
        self._grid_pub.publish(msg)

    def _on_goal(self, name: str, msg: PoseStamped):
        self._goals[name] = msg
        self._goal_dirty = True
        self.get_logger().info(
            f'Goal {name}: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})'
        )

    def _inflate(self, msg: OccupancyGrid) -> np.ndarray:
        w, h = msg.info.width, msg.info.height
        data = np.array(msg.data, dtype=np.int16).reshape((h, w))
        # Occupied OR unknown — never plan through unmapped cells
        blocked = (data >= int(100 * 0.65)) | (data < 0)
        rad_cells = max(1, int(math.ceil(self.robot_radius / msg.info.resolution)))
        pad = np.pad(blocked.astype(np.uint8), rad_cells, mode='constant', constant_values=1)
        out = np.zeros_like(blocked, dtype=bool)
        for dy in range(-rad_cells, rad_cells + 1):
            for dx in range(-rad_cells, rad_cells + 1):
                if dx * dx + dy * dy > rad_cells * rad_cells:
                    continue
                out |= pad[
                    rad_cells + dy: rad_cells + dy + h,
                    rad_cells + dx: rad_cells + dx + w,
                ].astype(bool)
        return out

    @staticmethod
    def _downsample(fine: np.ndarray, factor: int) -> np.ndarray:
        """OR-pool fine blocked cells into coarse cells (true coarsening)."""
        if factor <= 1:
            return fine.copy()
        h, w = fine.shape
        nh, nw = h // factor, w // factor
        # reshape blocks then any()
        trimmed = fine[: nh * factor, : nw * factor]
        blocks = trimmed.reshape(nh, factor, nw, factor)
        return blocks.any(axis=(1, 3))

    def _world_to_cell(self, x: float, y: float) -> Optional[Cell]:
        c = int((x - self._coarse_ox) / self._coarse_res)
        r = int((y - self._coarse_oy) / self._coarse_res)
        if c < 0 or r < 0 or c >= self._coarse_w or r >= self._coarse_h:
            return None
        return c, r

    def _cell_to_world(self, c: int, r: int) -> Tuple[float, float]:
        x = self._coarse_ox + (c + 0.5) * self._coarse_res
        y = self._coarse_oy + (r + 0.5) * self._coarse_res
        return x, y

    def _is_free(self, c: int, r: int) -> bool:
        assert self._grid is not None
        if c < 0 or r < 0 or c >= self._coarse_w or r >= self._coarse_h:
            return False
        return not bool(self._grid[r, c])

    def _cheb_nearest_free(self, cell: Cell, max_r: int = 30) -> Optional[Cell]:
        """Spatially nearest free cell (may cross walls — only for local start snap)."""
        c0, r0 = cell
        if self._is_free(c0, r0):
            return cell
        for rad in range(1, max_r + 1):
            for dr in range(-rad, rad + 1):
                for dc in range(-rad, rad + 1):
                    c, r = c0 + dc, r0 + dr
                    if self._is_free(c, r):
                        return c, r
        return None

    def _nearest_free_in_component(self, target: Cell, seed: Cell) -> Optional[Cell]:
        """Free cell closest to ``target`` that is reachable from ``seed`` (no wall jump)."""
        seed_f = seed if self._is_free(*seed) else self._cheb_nearest_free(seed)
        if seed_f is None:
            return None
        tx, ty = target
        best: Optional[Cell] = None
        best_d = 10**9
        q: deque[Cell] = deque([seed_f])
        seen: Set[Cell] = {seed_f}
        while q:
            c, r = q.popleft()
            d = (c - tx) * (c - tx) + (r - ty) * (r - ty)
            if d < best_d:
                best_d = d
                best = (c, r)
            for dc, dr in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                nc, nr = c + dc, r + dr
                nxt = (nc, nr)
                if nxt in seen or not self._is_free(nc, nr):
                    continue
                seen.add(nxt)
                q.append(nxt)
        return best

    def _fine_segment_free(self, x0: float, y0: float, x1: float, y1: float) -> bool:
        """True if the straight segment stays in non-inflated-free fine cells."""
        if self._map is None or self._fine_grid is None:
            return False
        res = float(self._map.info.resolution)
        ox = float(self._map.info.origin.position.x)
        oy = float(self._map.info.origin.position.y)
        h, w = self._fine_grid.shape
        dist = math.hypot(x1 - x0, y1 - y0)
        n = max(1, int(math.ceil(dist / max(res * 0.5, 1e-3))))
        for i in range(n + 1):
            t = i / n
            x = x0 + (x1 - x0) * t
            y = y0 + (y1 - y0) * t
            c = int((x - ox) / res)
            r = int((y - oy) / res)
            if c < 0 or r < 0 or c >= w or r >= h or bool(self._fine_grid[r, c]):
                return False
        return True

    def _lookup_start(self, name: str) -> Optional[Tuple[float, float]]:
        buf = self._tf_buffers[name]
        try:
            t = buf.lookup_transform('map', 'base_footprint', rclpy.time.Time())
            return t.transform.translation.x, t.transform.translation.y
        except Exception as e:
            fb = self._fallback_starts.get(name)
            if fb is not None:
                self.get_logger().warn(
                    f'TF start fail {name}, using spawn fallback {fb}: {e}',
                    throttle_duration_sec=5.0,
                )
                return fb
            self.get_logger().warn(f'TF start fail {name}: {e}', throttle_duration_sec=2.0)
            return None

    def _tick(self):
        if not self._goal_dirty:
            return
        if self._map is None or self._grid is None:
            return
        if len(self._goals) < len(self.robots):
            return
        starts = {}
        for name in self.robots:
            s = self._lookup_start(name)
            if s is None:
                return
            starts[name] = s
        self._goal_dirty = False
        self._plan_all(starts)

    def _rebuild_grid_for_mission(self, starts_xy: Dict[str, Tuple[float, float]]):
        """Re-derive coarse grid using live start/goal pairs (connectivity shrink)."""
        assert self._map is not None and self._fine_grid is not None
        res = float(self._map.info.resolution)
        goals_xy = [
            (self._goals[n].pose.position.x, self._goals[n].pose.position.y)
            for n in self.robots
        ]
        start_list = [starts_xy[n] for n in self.robots]
        ideal = compute_mapf_grid(
            res,
            self.robot_radius,
            planning_margin=self.planning_margin,
            extra_sep_cells=self.extra_sep_cells,
            grid_stride_override=self._stride_override,
            min_sep_override=self._sep_override,
        )
        params, coarse = shrink_downsample_for_connectivity(
            self._fine_grid,
            start_list,
            goals_xy,
            (self._coarse_ox, self._coarse_oy),
            res,
            self.robot_radius,
            planning_margin=self.planning_margin,
            extra_sep_cells=self.extra_sep_cells,
            grid_stride_override=self._stride_override,
            min_sep_override=self._sep_override,
        )
        if (
            params.downsample != self.downsample
            or params.min_sep != self.min_sep
            or abs(params.coarse_res - self._coarse_res) > 1e-9
        ):
            self.get_logger().info(
                f'Mission grid: downsample={params.downsample} '
                f'cell={params.coarse_res:.3f}m min_sep={params.min_sep} '
                f'(ideal downsample={ideal.downsample})'
            )
        self.downsample = params.downsample
        self.min_sep = params.min_sep
        self._neighbor_buffer = params.neighbor_buffer
        self._grid = coarse
        self._coarse_res = params.coarse_res
        self._coarse_h, self._coarse_w = coarse.shape
        self._publish_grid_params(params)

    def _plan_all(self, starts_xy: Dict[str, Tuple[float, float]]):
        self._rebuild_grid_for_mission(starts_xy)

        start_cells: List[Cell] = []
        goal_cells: List[Cell] = []
        goal_poses: List[PoseStamped] = []

        snapped_goals_xy: List[Tuple[float, float]] = []
        for name in self.robots:
            sx, sy = starts_xy[name]
            g = self._goals[name]
            gx, gy = g.pose.position.x, g.pose.position.y
            sc = self._cheb_nearest_free(self._world_to_cell(sx, sy) or (0, 0))
            # Goal must stay in the start's free component (never snap through walls)
            gc = None
            if sc is not None:
                gc = self._nearest_free_in_component(
                    self._world_to_cell(gx, gy) or sc, sc
                )
            if sc is None or gc is None:
                self.get_logger().error(f'{name}: no free cell near start/goal')
                ready = Bool()
                ready.data = False
                self._ready_pub.publish(ready)
                return
            raw_g = self._world_to_cell(gx, gy)
            if raw_g is None or not self._is_free(*raw_g) or raw_g != gc:
                sgx, sgy = self._cell_to_world(*gc)
                self.get_logger().warn(
                    f'{name}: goal ({gx:.2f},{gy:.2f}) not free/reachable → '
                    f'snap to ({sgx:.2f},{sgy:.2f})'
                )
            start_cells.append(sc)
            goal_cells.append(gc)
            goal_poses.append(g)
            snapped_goals_xy.append(self._cell_to_world(*gc))

        self.get_logger().info(
            f'Running PIBT on coarse {self._coarse_w}x{self._coarse_h} '
            f'(cell={self._coarse_res:.3f}m min_sep={self.min_sep})…'
        )
        # Unit steps on the already-coarsened grid
        result = pibt_solve(
            start_cells,
            goal_cells,
            self._is_free,
            1,
            self._coarse_w,
            self._coarse_h,
            self.max_steps,
            min_sep=self.min_sep,
        )
        if result is None:
            self.get_logger().error('PIBT failed to find collision-free paths')
            for name in self.robots:
                empty = Path()
                empty.header.frame_id = 'map'
                empty.header.stamp = self.get_clock().now().to_msg()
                self._path_pubs[name].publish(empty)
            ready = Bool()
            ready.data = False
            self._ready_pub.publish(ready)
            return

        # Pad to common makespan (already equal length from PIBT loop)
        makespan = max(len(p) for p in result)
        for i, cells in enumerate(result):
            while len(cells) < makespan:
                cells.append(cells[-1])
            path = self._cells_to_path(
                cells, goal_poses[i], snapped_goal_xy=snapped_goals_xy[i]
            )
            name = self.robots[i]
            self._path_pubs[name].publish(path)
            self.get_logger().info(
                f'{name}: PIBT path len={len(cells)} poses={len(path.poses)}'
            )

        ready = Bool()
        ready.data = True
        self._ready_pub.publish(ready)
        self.get_logger().info(
            f'paths_ready=True ({len(self.robots)}/{len(self.robots)}) makespan={makespan}'
        )

    def _cells_to_path(
        self,
        cells: List[Cell],
        goal_pose: PoseStamped,
        snapped_goal_xy: Optional[Tuple[float, float]] = None,
    ) -> Path:
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()
        for i, (c, r) in enumerate(cells):
            x, y = self._cell_to_world(c, r)
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = x
            ps.pose.position.y = y
            if i + 1 < len(cells) and cells[i + 1] != (c, r):
                nx, ny = self._cell_to_world(*cells[i + 1])
                yaw = math.atan2(ny - y, nx - x)
                ps.pose.orientation.z = math.sin(yaw / 2.0)
                ps.pose.orientation.w = math.cos(yaw / 2.0)
            elif path.poses:
                ps.pose.orientation = path.poses[-1].pose.orientation
            else:
                ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        if path.poses:
            # Prefer requested goal only when the final segment is fine-grid clear;
            # otherwise keep the snapped free cell (avoids RViz paths through walls).
            gx = float(goal_pose.pose.position.x)
            gy = float(goal_pose.pose.position.y)
            lx = path.poses[-1].pose.position.x
            ly = path.poses[-1].pose.position.y
            if self._fine_segment_free(lx, ly, gx, gy):
                path.poses[-1].pose.position.x = gx
                path.poses[-1].pose.position.y = gy
            elif snapped_goal_xy is not None:
                path.poses[-1].pose.position.x = snapped_goal_xy[0]
                path.poses[-1].pose.position.y = snapped_goal_xy[1]
            path.poses[-1].pose.orientation = goal_pose.pose.orientation
        return path


def main():
    rclpy.init()
    node = PibtPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
