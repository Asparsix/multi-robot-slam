#!/usr/bin/env python3
"""SMART-style Action Dependency Graph (ported from smart-mapf/smart server/ADG).

Source reference:
  https://github.com/smart-mapf/smart/blob/master/server/src/ADG.cpp
  Hönig et al., Persistent and Robust Execution of MAPF Schedules (RA-L 2019)

Type-1 order is implicit (node index sequence per robot).
Type-2 edges: if agent i action start == agent k action goal and
time_i <= time_k, then k waits until i finishes that action.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Set, Tuple


XY = Tuple[float, float]


@dataclass
class Action:
    robot_id: int
    time: float
    start: XY
    goal: XY
    node_id: int = 0


@dataclass
class Edge:
    from_agent_id: int
    to_agent_id: int
    from_node_id: int
    to_node_id: int
    valid: bool = True


@dataclass
class ADGNode:
    action: Action
    node_id: int
    income_edges: List[Edge] = field(default_factory=list)
    out_edges: List[Edge] = field(default_factory=list)
    has_valid_in_edge: bool = True


def _key(p: XY, quant: float = 0.05) -> Tuple[int, int]:
    return (int(round(p[0] / quant)), int(round(p[1] / quant)))


def _near_cell(a: XY, b: XY, quant: float, neighbor: int) -> bool:
    """True if quantized cells are within Chebyshev distance ``neighbor``."""
    ka, kb = _key(a, quant), _key(b, quant)
    return max(abs(ka[0] - kb[0]), abs(ka[1] - kb[1])) <= neighbor


class SmartADG:
    """Python port of SMART ``ADG`` class (construction + release API)."""

    def __init__(
        self,
        plans: List[List[Action]],
        quant: float = 0.05,
        neighbor_buffer: int = 0,
    ):
        self.quant = quant
        self.neighbor_buffer = max(0, int(neighbor_buffer))
        self.num_robots = len(plans)
        self.finished_node_idx = [-1] * self.num_robots
        self.enqueue_nodes_idx: List[List[int]] = [[] for _ in range(self.num_robots)]
        self.graph: List[List[ADGNode]] = [[] for _ in range(self.num_robots)]
        self.type1_edges = 0
        self.type2_edges = 0
        self.total_nodes = 0

        for i, plan in enumerate(plans):
            for j, action in enumerate(plan):
                action.node_id = j
                action.robot_id = i
                self.graph[i].append(ADGNode(action=action, node_id=j))
                self.total_nodes += 1
            if len(plan) > 0:
                self.type1_edges += max(0, len(plan) - 1)

        # Type-2 edges (SMART ADG.cpp) + optional neighbor buffer / same-cell goals
        nb = self.neighbor_buffer
        for i in range(self.num_robots):
            for j, ai in enumerate(plans[i]):
                for k in range(i + 1, self.num_robots):
                    for l, ak in enumerate(plans[k]):
                        # Handoff: i leaves a cell k is entering (start_i ~ goal_k)
                        if _near_cell(ai.start, ak.goal, quant, nb) and ai.time <= ak.time:
                            e = Edge(i, k, j, l)
                            self.graph[i][j].out_edges.append(e)
                            self.graph[k][l].income_edges.append(e)
                            self.type2_edges += 1
                        elif _near_cell(ak.start, ai.goal, quant, nb) and ak.time <= ai.time:
                            e = Edge(k, i, l, j)
                            self.graph[k][l].out_edges.append(e)
                            self.graph[i][j].income_edges.append(e)
                            self.type2_edges += 1
                        # Same-time vertex: both targeting same / neighboring cell
                        elif (
                            _near_cell(ai.goal, ak.goal, quant, nb)
                            and abs(ai.time - ak.time) < 1e-6
                        ):
                            # Lower index finishes first (stable, acyclic)
                            e = Edge(i, k, j, l)
                            self.graph[i][j].out_edges.append(e)
                            self.graph[k][l].income_edges.append(e)
                            self.type2_edges += 1

        # Nodes with no income edges are immediately releasable
        for i in range(self.num_robots):
            for node in self.graph[i]:
                self._update_node(node)

    @staticmethod
    def _update_node(node: ADGNode) -> None:
        node.has_valid_in_edge = any(e.valid for e in node.income_edges)

    def get_available_nodes(self, robot_id: int) -> List[int]:
        """SMART getAvailableNodes: consecutive free nodes after finished."""
        plan = self.graph[robot_id]
        available: List[int] = []
        next_idx = self.finished_node_idx[robot_id] + 1
        enq = self.enqueue_nodes_idx[robot_id]
        for i in range(next_idx, len(plan)):
            if plan[i].has_valid_in_edge:
                self._update_node(plan[i])
            if plan[i].has_valid_in_edge:
                break
            if not enq or enq[-1] < i:
                available.append(i)
        return available

    def set_enqueue_nodes(self, robot_id: int, nodes: List[int]) -> None:
        cur = self.enqueue_nodes_idx[robot_id]
        if not cur:
            cur.extend(nodes)
            return
        start = 0
        for start, n in enumerate(nodes):
            if n > cur[-1]:
                break
        else:
            start = len(nodes)
        cur.extend(nodes[start:])

    def update_finished_node(self, robot_id: int, node_id: int) -> bool:
        """Mark nodes through node_id finished; invalidate outgoing Type-2 edges."""
        latest = self.finished_node_idx[robot_id]
        if node_id <= latest:
            return True
        enq = self.enqueue_nodes_idx[robot_id]
        if enq and node_id > enq[-1]:
            return False
        for tmp_idx in range(latest + 1, node_id + 1):
            for e in self.graph[robot_id][tmp_idx].out_edges:
                e.valid = False
        self.finished_node_idx[robot_id] = node_id
        while enq and enq[0] <= node_id:
            enq.pop(0)
        return True

    def is_agent_finished(self, robot_id: int) -> bool:
        n = len(self.graph[robot_id])
        if n == 0:
            return True
        return self.finished_node_idx[robot_id] >= n - 1

    def all_finished(self) -> bool:
        return all(self.is_agent_finished(i) for i in range(self.num_robots))

    def release_for_agent(self, robot_id: int, max_n: int = 0) -> List[int]:
        """Like SMART getPlan(): return newly available node ids and enqueue them.

        If max_n > 0, only release up to that many new actions (lookahead).
        """
        avail = self.get_available_nodes(robot_id)
        if max_n > 0:
            avail = avail[:max_n]
        if avail:
            self.set_enqueue_nodes(robot_id, avail)
        return avail


def paths_to_actions(
    paths: Dict[str, List[XY]],
    robot_names: List[str],
) -> List[List[Action]]:
    """Convert /swarm path poses to SMART Action lists (one move per step)."""
    plans: List[List[Action]] = []
    for rid, name in enumerate(robot_names):
        poses = paths.get(name, [])
        acts: List[Action] = []
        if not poses:
            plans.append(acts)
            continue
        if len(poses) == 1:
            x, y = poses[0]
            acts.append(Action(rid, 0.0, (x, y), (x, y), 0))
            plans.append(acts)
            continue
        for t in range(len(poses) - 1):
            acts.append(
                Action(
                    robot_id=rid,
                    time=float(t),
                    start=poses[t],
                    goal=poses[t + 1],
                    node_id=t,
                )
            )
        plans.append(acts)
    return plans
