#!/usr/bin/env python3
"""Pre-compile the numba kernels so no board test pays for it.

Run after syncing a change to planning_kernels.py or nav_search_kernels.py; edits to
the node files no longer invalidate these caches.
"""

import sys
import time

import numpy as np

sys.path.insert(0, "/userdata/x5/tinynav")

from tinynav.core.nav_search_kernels import (  # noqa: E402
    search_close_to_sdf_map_numba,
    search_within_sdf_map_numba,
)
from tinynav.core.planning_kernels import (  # noqa: E402
    generate_trajectory_library_3d,
    run_raycasting_loopy,
    score_trajectories_by_ESDF,
)
from tinynav.core.planning_node import (  # noqa: E402
    generate_predefined_trajectory_vocabularies,
    normalize_pose_trajectories,
)
from tinynav.core.robot_config import robot_config  # noqa: E402

MAP = "/userdata/x5/tinynav_db/map"


def timed(label, fn):
    t0 = time.monotonic()
    fn()
    print("%-28s %7.0f ms" % (label, (time.monotonic() - t0) * 1000), flush=True)


def main() -> int:
    grid = (100, 100, 10)
    origin = np.array(grid) * 0.1 / -2.0
    ip, iq = np.zeros(3), np.array([0.0, 0.0, 0.0, 1.0])
    robot = robot_config("lekiwi")
    front, rear, half_w = robot.footprint_from_control()

    # vx_max must be passed, not defaulted: numba treats an omitted default as a
    # distinct Omitted() type and compiles a second signature the node never uses.
    # Both depth dtypes: mono16 arrives as float32, 32FC1 as float64.
    depth32 = np.full((544, 640), 2.0, dtype=np.float32)
    timed("raycasting float32", lambda: run_raycasting_loopy(
        depth32, np.eye(4), grid, 400.0, 400.0, 320.0, 272.0, origin, 10, 0.1))
    timed("raycasting float64", lambda: run_raycasting_loopy(
        depth32.astype(np.float64), np.eye(4), grid, 400.0, 400.0, 320.0, 272.0, origin, 10, 0.1))

    trajs = [None]

    def build():
        t, p = generate_trajectory_library_3d(init_p=ip, init_q=iq, dt=0.1, vx_max=robot.max_vx)
        t = normalize_pose_trajectories(t)
        v, _ = generate_predefined_trajectory_vocabularies(init_p=ip, init_q=iq, dt=0.1)
        v = normalize_pose_trajectories(v)
        trajs[0] = np.concatenate([t, v], axis=0) if len(v) else t

    timed("trajectory library", build)
    esdf = np.full(grid[:2], 10.0, dtype=np.float32)
    timed("trajectory scoring", lambda: score_trajectories_by_ESDF(
        trajs[0], esdf, origin, 0.1, robot.safety_radius, front, rear, half_w))

    try:
        sdf = np.load(MAP + "/sdf_map.npy")
        occ = np.load(MAP + "/occupancy_grid.npy")
        meta = np.load(MAP + "/occupancy_meta.npy")
    except OSError as exc:
        print("no map at %s (%s) -- A* kernels not warmed" % (MAP, exc))
        return 0
    # Same dtypes as the real arrays; the shape does not enter the signature.
    small_sdf = np.ones((3, 3, 3), dtype=sdf.dtype)
    small_sdf[1, 1, 1] = 0.0
    small_occ = np.zeros((3, 3, 3), dtype=occ.dtype)
    z = np.array([0, 0, 0], dtype=np.int32)
    timed("A* search_close", lambda: search_close_to_sdf_map_numba(z, small_sdf, small_occ, 0.2))
    timed("A* search_within", lambda: search_within_sdf_map_numba(
        z, np.array([2, 2, 2], dtype=np.int32), small_sdf, small_occ, meta[3]))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
