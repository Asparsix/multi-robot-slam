"""CBBA solver (Choi et al. 2009) with pluggable path-cost model.

Adapted from keep9oing/consensus-based-bundle-algorithm (MIT License).
"""

from __future__ import annotations

import copy
from typing import Callable, List, Sequence, Tuple

import numpy as np


class CBBAAgent:
    def __init__(
        self,
        agent_id: int,
        task_num: int,
        agent_num: int,
        bundle_capacity: int,
        state_xy: Tuple[float, float],
        score_fn: Callable[[int, Sequence[int]], float],
    ):
        self.task_num = task_num
        self.agent_num = agent_num
        self.id = agent_id
        self.L_t = bundle_capacity
        self.state = np.array(state_xy, dtype=np.float64)
        self.score_fn = score_fn

        # Unassigned winner / zero bid (standard CBBA init).
        self.z = np.full(self.task_num, -1, dtype=np.int8)
        self.y = np.zeros(self.task_num, dtype=np.float64)
        self.b: List[int] = []
        self.p: List[int] = []
        self.time_step = 0
        self.s = {a: self.time_step for a in range(self.agent_num)}
        self.c = np.zeros(self.task_num, dtype=np.float64)
        self.Y = None

    def send_message(self):
        return self.y.tolist(), self.z.tolist(), self.s

    def receive_message(self, y_dict):
        self.Y = y_dict

    def _path_score(self, path: List[int]) -> float:
        return self.score_fn(self.id, path)

    def build_bundle(self):
        J = list(range(self.task_num))
        while len(self.b) < self.L_t:
            S_p = self._path_score(self.p) if self.p else 0.0

            best_pos = {}
            for j in J:
                if j in self.b:
                    self.c[j] = 0.0
                    continue
                c_list = []
                for n in range(len(self.p) + 1):
                    p_temp = copy.deepcopy(self.p)
                    p_temp.insert(n, j)
                    c_temp = self._path_score(p_temp)
                    c_list.append(c_temp - S_p)
                max_idx = int(np.argmax(c_list))
                self.c[j] = c_list[max_idx]
                best_pos[j] = max_idx

            h = self.c > self.y
            if not np.any(h):
                break
            self.c[~h] = 0.0
            j_star = int(np.argmax(self.c))
            n_j = best_pos[j_star]

            self.b.append(j_star)
            self.p.insert(n_j, j_star)
            self.y[j_star] = self.c[j_star]
            self.z[j_star] = self.id

    def __update(self, j, y_kj, z_kj):
        self.y[j] = y_kj
        self.z[j] = z_kj

    def __leave(self):
        pass

    def __reset(self, j):
        if j in self.b:
            self.b.remove(j)
        self.y[j] = 0.0
        self.z[j] = -1

    def update_task(self) -> bool:
        old_p = copy.deepcopy(self.p)
        if self.Y is None:
            return True

        id_list = list(self.Y.keys())
        id_list.insert(0, self.id)

        for aid in list(self.s.keys()):
            if aid in id_list:
                self.s[aid] = self.time_step
            else:
                s_list = [self.Y[nid][2][aid] for nid in id_list[1:] if aid in self.Y[nid][2]]
                if s_list:
                    self.s[aid] = max(s_list)

        for j in range(self.task_num):
            for k in id_list[1:]:
                y_k, z_k, s_k = self.Y[k]
                y_kj = y_k[j]
                z_kj = z_k[j]
                z_ij = self.z[j]
                y_ij = self.y[j]
                i = self.id

                if z_kj == k:
                    if z_ij == self.id:
                        if y_kj > y_ij:
                            self.__update(j, y_kj, z_kj)
                        elif abs(y_kj - y_ij) < np.finfo(float).eps and k < self.id:
                            self.__update(j, y_kj, z_kj)
                        else:
                            self.__leave()
                    elif z_ij == k:
                        self.__update(j, y_kj, z_kj)
                    elif z_ij != -1:
                        m = z_ij
                        if s_k[m] > self.s[m] or y_kj > y_ij:
                            self.__update(j, y_kj, z_kj)
                        elif abs(y_kj - y_ij) < np.finfo(float).eps and k < self.id:
                            self.__update(j, y_kj, z_kj)
                    elif z_ij == -1:
                        self.__update(j, y_kj, z_kj)
                elif z_kj == i:
                    if z_ij == i:
                        self.__leave()
                    elif z_ij == k:
                        self.__reset(j)
                    elif z_ij != -1:
                        m = z_ij
                        if s_k[m] > self.s[m]:
                            self.__reset(j)
                    elif z_ij == -1:
                        self.__leave()
                elif z_kj != -1:
                    m = z_kj
                    if z_ij == i:
                        if (s_k[m] >= self.s[m] and y_kj > y_ij) or (
                            s_k[m] >= self.s[m]
                            and abs(y_kj - y_ij) < np.finfo(float).eps
                            and m < self.id
                        ):
                            self.__update(j, y_kj, z_kj)
                    elif z_ij == k:
                        if s_k[m] > self.s[m]:
                            self.__update(j, y_kj, z_kj)
                        else:
                            self.__reset(j)
                    elif z_ij == m:
                        if s_k[m] > self.s[m]:
                            self.__update(j, y_kj, z_kj)
                    elif z_ij != -1:
                        n = z_ij
                        if (
                            (s_k[m] > self.s[m] and s_k[n] > self.s[n])
                            or (s_k[m] > self.s[m] and y_kj > y_ij)
                            or (
                                s_k[m] > self.s[m]
                                and abs(y_kj - y_ij) < np.finfo(float).eps
                                and m < n
                            )
                            or (s_k[n] > self.s[n] and self.s[m] > s_k[m])
                        ):
                            self.__update(j, y_kj, z_kj)
                    elif z_ij == -1 and s_k[m] > self.s[m]:
                        self.__update(j, y_kj, z_kj)
                elif z_kj == -1:
                    if z_ij == i:
                        self.__leave()
                    elif z_ij == k:
                        self.__update(j, y_kj, z_kj)
                    elif z_ij != -1:
                        m = z_ij
                        if s_k[m] > self.s[m]:
                            self.__update(j, y_kj, z_kj)
                    elif z_ij == -1:
                        self.__leave()

        n_bar = len(self.b)
        for n in range(len(self.b)):
            b_n = self.b[n]
            if self.z[b_n] != self.id:
                n_bar = n
                break

        tail = copy.deepcopy(self.b[n_bar + 1 :])
        if tail:
            self.y[tail] = 0.0
            self.z[tail] = -1
        if n_bar < len(self.b):
            del self.b[n_bar:]

        self.p = list(self.b)
        self.time_step += 1
        return old_p == self.p


def run_cbba(
    agent_states: List[Tuple[float, float]],
    task_xy: np.ndarray,
    bundle_capacity: int,
    score_fn: Callable[[int, Sequence[int]], float],
    max_iter: int = 100,
) -> List[CBBAAgent]:
    n_agents = len(agent_states)
    n_tasks = task_xy.shape[0]
    agents = [
        CBBAAgent(i, n_tasks, n_agents, bundle_capacity, agent_states[i], score_fn)
        for i in range(n_agents)
    ]
    comm = np.ones((n_agents, n_agents), dtype=np.int8)

    for t in range(max_iter):
        for ag in agents:
            ag.build_bundle()

        messages = [ag.send_message() for ag in agents]
        converged = []
        for i, ag in enumerate(agents):
            neighbors = [j for j in range(n_agents) if comm[i, j] and j != i]
            y_dict = {j: messages[j] for j in neighbors} if neighbors else None
            ag.receive_message(y_dict)
            converged.append(ag.update_task() if y_dict else True)

        if all(converged):
            break

    return agents
