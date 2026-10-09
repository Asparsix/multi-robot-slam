#!/usr/bin/env python3
"""Derive MAPF coarse-grid parameters from robot body + map resolution.

Not demo-specific: cell size and Chebyshev separation follow from geometry.

  target_clearance = 2 * robot_radius + planning_margin
  downsample       = ceil(target_clearance / map_resolution)   # coarse cell ≥ clearance
  coarse_res       = downsample * map_resolution
  min_sep          = max(0, ceil(target_clearance / coarse_res) - 1) + extra_sep_cells

With min_sep S, agents may not occupy cells at Chebyshev distance ≤ S, so the
minimum planned center gap is (S+1) * coarse_res ≥ target_clearance (plus
extra_sep_cells for cmd_vel / ADG tracking slack).
"""

from __future__ import annotations

import math
from typing import NamedTuple, Optional, Sequence, Tuple


class MapfGridParams(NamedTuple):
    downsample: int
    coarse_res: float
    min_sep: int
    target_clearance: float
    neighbor_buffer: int


def compute_mapf_grid(
    map_resolution: float,
    robot_radius: float,
    *,
    planning_margin: float = 0.08,
    extra_sep_cells: int = 1,
    grid_stride_override: int = 0,
    min_sep_override: int = -1,
) -> MapfGridParams:
    """Compute downsample / coarse cell / min_sep from physics + map res.

    Overrides (for debugging only):
      grid_stride_override > 0 → force that downsample
      min_sep_override >= 0    → force that separation
    """
    res = float(map_resolution)
    if res <= 0.0:
        raise ValueError(f'map_resolution must be > 0, got {res}')
    radius = max(1e-3, float(robot_radius))
    margin = max(0.0, float(planning_margin))
    clearance = 2.0 * radius + margin

    if grid_stride_override and int(grid_stride_override) > 0:
        downsample = max(1, int(grid_stride_override))
    else:
        downsample = max(1, int(math.ceil(clearance / res)))

    coarse_res = downsample * res

    if min_sep_override is not None and int(min_sep_override) >= 0:
        min_sep = int(min_sep_override)
    else:
        # Need (min_sep+1)*coarse_res >= clearance, then add tracking pad cells
        geometric = max(0, int(math.ceil(clearance / coarse_res)) - 1)
        min_sep = geometric + max(0, int(extra_sep_cells))

    # Type-2 neighbor pad should cover the same exclusion neighborhood
    neighbor_buffer = min_sep

    return MapfGridParams(
        downsample=downsample,
        coarse_res=coarse_res,
        min_sep=min_sep,
        target_clearance=clearance,
        neighbor_buffer=neighbor_buffer,
    )


def shrink_downsample_for_connectivity(
    fine_blocked,
    starts_xy: Sequence[Tuple[float, float]],
    goals_xy: Sequence[Tuple[float, float]],
    origin_xy: Tuple[float, float],
    map_resolution: float,
    robot_radius: float,
    *,
    planning_margin: float = 0.08,
    extra_sep_cells: int = 1,
    grid_stride_override: int = 0,
    min_sep_override: int = -1,
) -> Tuple[MapfGridParams, object]:
    """Pick grid params; if spawn↔goal graph disconnects, reduce downsample.

    Returns (params, coarse_blocked_bool_array).
    """
    import numpy as np
    from collections import deque

    base = compute_mapf_grid(
        map_resolution,
        robot_radius,
        planning_margin=planning_margin,
        extra_sep_cells=extra_sep_cells,
        grid_stride_override=grid_stride_override,
        min_sep_override=min_sep_override,
    )

    def downsample_or(fine, factor: int):
        if factor <= 1:
            return fine.copy()
        h, w = fine.shape
        nh, nw = h // factor, w // factor
        trimmed = fine[: nh * factor, : nw * factor]
        return trimmed.reshape(nh, factor, nw, factor).any(axis=(1, 3))

    def cheb_nearest_free(grid, cell, max_r: int = 30):
        c0, r0 = cell
        ch, cw = grid.shape
        for rad in range(0, max_r + 1):
            for dr in range(-rad, rad + 1):
                for dc in range(-rad, rad + 1):
                    c, r = c0 + dc, r0 + dr
                    if 0 <= c < cw and 0 <= r < ch and not grid[r, c]:
                        return (c, r)
        return None

    def nearest_in_component(grid, target, seed):
        """Closest free cell to target that is reachable from seed."""
        ch, cw = grid.shape
        if seed is None:
            return None
        if grid[seed[1], seed[0]]:
            seed = cheb_nearest_free(grid, seed)
        if seed is None:
            return None
        tx, ty = target
        best = None
        best_d = 10**9
        q = deque([seed])
        seen = {seed}
        while q:
            c, r = q.popleft()
            d = (c - tx) * (c - tx) + (r - ty) * (r - ty)
            if d < best_d:
                best_d = d
                best = (c, r)
            for dc, dr in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                nc, nr = c + dc, r + dr
                if 0 <= nc < cw and 0 <= nr < ch and not grid[nr, nc] and (nc, nr) not in seen:
                    seen.add((nc, nr))
                    q.append((nc, nr))
        return best

    def connected(grid, a, b) -> bool:
        if a is None or b is None:
            return False
        q = deque([a])
        seen = {a}
        ch, cw = grid.shape
        while q:
            c, r = q.popleft()
            if (c, r) == b:
                return True
            for dc, dr in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                nc, nr = c + dc, r + dr
                if 0 <= nc < cw and 0 <= nr < ch and not grid[nr, nc] and (nc, nr) not in seen:
                    seen.add((nc, nr))
                    q.append((nc, nr))
        return False

    ox, oy = origin_xy
    # If override forced, do not shrink
    factors = (
        [base.downsample]
        if grid_stride_override and int(grid_stride_override) > 0
        else list(range(base.downsample, 0, -1))
    )

    last_params = base
    last_grid = downsample_or(fine_blocked, base.downsample)
    for factor in factors:
        params = compute_mapf_grid(
            map_resolution,
            robot_radius,
            planning_margin=planning_margin,
            extra_sep_cells=extra_sep_cells,
            grid_stride_override=factor,
            min_sep_override=min_sep_override,
        )
        grid = downsample_or(fine_blocked, params.downsample)
        cres = params.coarse_res
        ok = True
        pairs = list(zip(starts_xy, goals_xy)) if goals_xy else [(s, s) for s in starts_xy]
        if not pairs:
            return params, grid
        for (sx, sy), (gx, gy) in pairs:
            sc0 = (int((sx - ox) / cres), int((sy - oy) / cres))
            gc0 = (int((gx - ox) / cres), int((gy - oy) / cres))
            sc = cheb_nearest_free(grid, sc0)
            gc = nearest_in_component(grid, gc0, sc)
            if not connected(grid, sc, gc):
                ok = False
                break
        last_params, last_grid = params, grid
        if ok:
            return params, grid

    return last_params, last_grid


if __name__ == '__main__':
    # Tiny self-check
    p = compute_mapf_grid(0.05, 0.18, planning_margin=0.08, extra_sep_cells=1)
    print(p)
    assert p.downsample >= 1
    assert (p.min_sep + 1) * p.coarse_res >= p.target_clearance - 1e-9
