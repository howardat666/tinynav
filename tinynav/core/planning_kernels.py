"""Numba kernels for the planning loop, kept in their own module on purpose.

numba invalidates a file's whole disk cache on any change to that file (the index
stores its mtime and size), so leaving these in planning_node.py meant every edit to
node logic cost a 70-90 s recompile. Here they only recompile when they change.
"""

import numpy as np
from numba import njit

from tinynav.core.math_utils import matrix_to_quat, quat_to_matrix, rotvec_to_matrix

@njit(cache=True)
def run_raycasting_loopy(depth_image, T_cam_to_world, grid_shape, fx, fy, cx, cy, origin, step, resolution, filter_ground = False):
    """
    A "C-style" version of run_raycasting that uses explicit loops instead of
    NumPy vector operations, designed for optimal Numba performance.
    Reference: https://numba.readthedocs.io/en/stable/user/performance-tips.html#loops
    """
    occupancy_grid = np.zeros(grid_shape)
    depth_height, depth_width = depth_image.shape

    grid_shape_x, grid_shape_y, grid_shape_z = grid_shape
    origin_x, origin_y, origin_z = origin

    cam_orig_x = T_cam_to_world[0, 3]
    cam_orig_y = T_cam_to_world[1, 3]
    cam_orig_z = T_cam_to_world[2, 3]

    start_voxel_x = int(np.floor((cam_orig_x - origin_x) / resolution))
    start_voxel_y = int(np.floor((cam_orig_y - origin_y) / resolution))
    start_voxel_z = int(np.floor((cam_orig_z - origin_z) / resolution))

    for v in range(0, depth_height, step):
        for u in range(0, depth_width, step):
            d = depth_image[v, u]
            if (not np.isfinite(d)) or d <= 0:
                continue

            # Project to camera coordinates
            px = (u - cx) * d / fx
            py = (v - cy) * d / fy
            pz = d
            is_ground = py > 0

            # Transform to world coordinates (manual matrix multiplication)
            pw_x = T_cam_to_world[0, 0] * px + T_cam_to_world[0, 1] * py + T_cam_to_world[0, 2] * pz + T_cam_to_world[0, 3]
            pw_y = T_cam_to_world[1, 0] * px + T_cam_to_world[1, 1] * py + T_cam_to_world[1, 2] * pz + T_cam_to_world[1, 3]
            pw_z = T_cam_to_world[2, 0] * px + T_cam_to_world[2, 1] * py + T_cam_to_world[2, 2] * pz + T_cam_to_world[2, 3]

            # Calculate end voxel
            end_voxel_x = int(np.floor((pw_x - origin_x) / resolution))
            end_voxel_y = int(np.floor((pw_y - origin_y) / resolution))
            end_voxel_z = int(np.floor((pw_z - origin_z) / resolution))

            # Bresenham's line algorithm (simplified)
            diff_x = end_voxel_x - start_voxel_x
            diff_y = end_voxel_y - start_voxel_y
            diff_z = end_voxel_z - start_voxel_z

            steps = max(abs(diff_x), abs(diff_y), abs(diff_z))
            if steps == 0:
                continue

            for i in range(steps + 1):
                t = i / steps
                interp_x = int(round(start_voxel_x + t * diff_x))
                interp_y = int(round(start_voxel_y + t * diff_y))
                interp_z = int(round(start_voxel_z + t * diff_z))

                if (0 <= interp_x < grid_shape_x and
                    0 <= interp_y < grid_shape_y and
                    0 <= interp_z < grid_shape_z):
                    occupancy_grid[interp_x, interp_y, interp_z] -= 0.05

            if (0 <= end_voxel_x < grid_shape_x and
                0 <= end_voxel_y < grid_shape_y and
                0 <= end_voxel_z < grid_shape_z):
                if filter_ground and is_ground:
                    pass
                else:
                    occupancy_grid[end_voxel_x, end_voxel_y, end_voxel_z] += 0.2

    # Explicit clipping loop
    for i in range(grid_shape_x):
        for j in range(grid_shape_y):
            for k in range(grid_shape_z):
                if occupancy_grid[i, j, k] < -0.1:
                    occupancy_grid[i, j, k] = -0.1
                elif occupancy_grid[i, j, k] > 0.1:
                    occupancy_grid[i, j, k] = 0.1

    return occupancy_grid


@njit(cache=True)
def low_obstacle_hits(depth_image, T_cam_to_world, grid_shape, fx, fy, cx, cy,
                      origin, step, resolution, floor_z, h_lo, h_hi, max_range):
    """Per-cell count of depth returns h_lo..h_hi above the floor plane.

    Exists because the z-span test in build_obstacle_map structurally cannot see an
    office chair's star base: at 0.1 m voxels its 4-8 cm legs share a layer with the
    floor, so their span is 0 no matter what the threshold is, and the phase between
    the voxel lattice and the floor is set by VIO z and drifts. Measured on the board
    2026-08-26: floor residuals are p99 1.2 cm within 1 m and 2.6 cm at 1-2 m, while
    the chair base sits at 3-12 cm -- but beyond 2 m the fitted floor itself rises
    4.5 cm, which is why max_range exists.
    """
    h, w = grid_shape[0], grid_shape[1]
    hits = np.zeros((h, w), dtype=np.int32)
    depth_height, depth_width = depth_image.shape
    origin_x, origin_y = origin[0], origin[1]
    for v in range(0, depth_height, step):
        for u in range(0, depth_width, step):
            d = depth_image[v, u]
            if (not np.isfinite(d)) or d <= 0 or d > max_range:
                continue
            px = (u - cx) * d / fx
            py = (v - cy) * d / fy
            pw_x = T_cam_to_world[0, 0] * px + T_cam_to_world[0, 1] * py + T_cam_to_world[0, 2] * d + T_cam_to_world[0, 3]
            pw_y = T_cam_to_world[1, 0] * px + T_cam_to_world[1, 1] * py + T_cam_to_world[1, 2] * d + T_cam_to_world[1, 3]
            pw_z = T_cam_to_world[2, 0] * px + T_cam_to_world[2, 1] * py + T_cam_to_world[2, 2] * d + T_cam_to_world[2, 3]
            height = pw_z - floor_z
            if height < h_lo or height > h_hi:
                continue
            i = int(np.floor((pw_x - origin_x) / resolution))
            j = int(np.floor((pw_y - origin_y) / resolution))
            if 0 <= i < h and 0 <= j < w:
                hits[i, j] += 1
    return hits


@njit(cache=True)
def generate_trajectory_library_3d(
    num_samples=15, duration=3.0, dt=0.1,
    init_p=np.zeros(3), init_q=np.array([0, 0, 0, 1]), vx_max=0.5, omega_max=np.pi / 3
):
    """Regular sampled lattice (forward-only)."""
    num_steps = int(duration / dt) + 1

    n_vx = max(3, int(num_samples / 2))
    vx_samples = np.linspace(0.0, vx_max, n_vx)
    # omega_max used to be a hardcoded pi/3 = 1.047 while cmd_vel_control clipped wz to
    # RobotConfig.max_yaw = 0.8, so every turn the planner scored was 31% faster than the
    # one the wheels received -- the same "predictions run ahead of reality" this file's
    # vx_max comment warns about. Derived now so the two cannot disagree again.
    omega_y_samples = np.linspace(-omega_max, omega_max, num_samples)

    num_samples = len(vx_samples) * len(omega_y_samples)

    # Per step state layout:
    # [0:3]=position(x,y,z), [3:7]=quaternion(x,y,z,w),
    # [7:10]=linear velocity(vx,vy,vz), [10:13]=angular velocity(wx,wy,wz) in world frame.
    trajectories = np.empty((num_samples, num_steps, 13))
    params = np.empty((num_samples, 2))

    # Loop invariants, hoisted. dq depends only on omega_y, never on the step index,
    # yet it was rebuilt on every one of the 31 steps of all 105 trajectories -- 3255
    # rotvec_to_matrix calls per planning cycle to produce 15 distinct matrices. The
    # body velocity and angular velocity rows were likewise reallocated per step.
    #
    # numba does not hoist these on its own: rotvec_to_matrix is a separate njit
    # function, and the compiler will not assume it is pure and move it out of the
    # loop. Measured on the X5 with both versions compiled and interleaved in one
    # process, output bit-identical: 23.55 ms -> 13.31 ms p50, a 1.77x cut of a stage
    # that runs on every planning cycle that has a navigation target.
    #
    # Plain arrays rather than lists: reflected lists are deprecated in numba.
    dq_by_omega = np.empty((len(omega_y_samples), 3, 3))
    ang_vel_by_omega = np.empty((len(omega_y_samples), 3))
    for i_omega in range(len(omega_y_samples)):
        w = omega_y_samples[i_omega]
        dq_by_omega[i_omega] = rotvec_to_matrix(np.array([0.0, w * dt, 0.0]))
        ang_vel_by_omega[i_omega] = np.array([0.0, w, 0.0])
    v_body_by_vx = np.zeros((len(vx_samples), 3))
    for i_vx in range(len(vx_samples)):
        v_body_by_vx[i_vx, 2] = vx_samples[i_vx]
    # Never written below: `q = q @ dq` rebinds to a fresh array on the first step.
    R_init = quat_to_matrix(init_q)

    k = -1
    for i_vx in range(len(vx_samples)):
        vx = vx_samples[i_vx]
        v_body = v_body_by_vx[i_vx]
        for i_omega in range(len(omega_y_samples)):
            k += 1
            omega_y = omega_y_samples[i_omega]
            dq = dq_by_omega[i_omega]
            ang_vel = ang_vel_by_omega[i_omega]
            p = init_p.copy()
            q = R_init
            # Write straight into the output block instead of filling a scratch
            # array and copying it in.
            traj = trajectories[k]
            for i in range(num_steps):
                q = q @ dq
                v_world = q @ v_body
                p += v_world * dt
                traj[i, :3] = p
                traj[i, 3:7] = matrix_to_quat(q)
                traj[i, 7:10] = v_world
                traj[i, 10:13] = ang_vel
            #hack
            traj[:, 2] = traj[0, 2]
            params[k, 0] = vx
            params[k, 1] = omega_y
    return trajectories, params


@njit(cache=True)
def score_trajectories_by_ESDF(trajectories, ESDF_map, origin, resolution,
                                hard_clearance=1e-3, soft_clearance=0.1,
                                front_len=0.35, rear_len=0.35, half_w=0.15,
                                is_circle=False):
    """Score trajectories by minimum ESDF clearance, inf on collision.

    A circle is one lookup at the centre -- exact, since the ESDF already is the distance
    to the nearest obstacle. Sampling it as its circumscribing square instead put the
    corners 41% too far out, and rotating. Squares keep the 5-corner sampling.
    """
    scores = []
    occ_points = []
    ESDF_rows, ESDF_cols = ESDF_map.shape
    # 方形底盘按"间距不超过一格"铺满车体，而不是只取中心加四角。0.28x0.35 的车体在 0.1 m
    # 栅格上，前缘两角相距 0.35 m 却中间不取样 —— 一个 0.1 m 的障碍能整个坐在空隙里，五个点
    # 全读"空"。上游平台是靠膨胀 2 格盖住这件事的，所以过去把膨胀关掉就等于把盖子拿走而没
    # 换东西（2026-08 实车撞过）。铺满之后膨胀才敢归零，而膨胀每格要多 0.1 m 的过道净宽。
    # 4x5=20 个点，间距 0.093 / 0.088 m 都小于一格；代价是取样量 4 倍。
    if is_circle:
        n_along = 1
        n_across = 1
    else:
        n_along = int(np.ceil((front_len + rear_len) / resolution)) + 1
        n_across = int(np.ceil(2.0 * half_w / resolution)) + 1
        if n_along < 2:
            n_along = 2
        if n_across < 2:
            n_across = 2

    for t in range(len(trajectories)):
        traj = trajectories[t]
        min_dist_for_traj = float('inf')
        closest_step_for_traj = -1

        for i in range(len(traj)):
            x_world, y_world = traj[i, 0], traj[i, 1]
            qx, qy, qz, qw = traj[i, 3], traj[i, 4], traj[i, 5], traj[i, 6]

            # world XY forward from quaternion (body +Z forward)
            fwd_x = 2.0 * (qx * qz + qw * qy)
            fwd_y = 2.0 * (qy * qz - qw * qx)
            n = (fwd_x * fwd_x + fwd_y * fwd_y) ** 0.5
            if n > 1e-6:
                fwd_x /= n
                fwd_y /= n
            else:
                fwd_x, fwd_y = 1.0, 0.0
            left_x = -fwd_y
            left_y = fwd_x

            span_along = front_len + rear_len
            span_across = 2.0 * half_w
            for ia in range(n_along):
                if n_along == 1:
                    off_a = 0.0
                else:
                    off_a = -rear_len + span_along * ia / (n_along - 1)
                for ic in range(n_across):
                    if n_across == 1:
                        off_c = 0.0
                    else:
                        off_c = -half_w + span_across * ic / (n_across - 1)
                    px = x_world + fwd_x * off_a + left_x * off_c
                    py = y_world + fwd_y * off_a + left_y * off_c
                    x_img = int((px - origin[0]) / resolution)
                    y_img = int((py - origin[1]) / resolution)
                    if 0 <= x_img < ESDF_rows and 0 <= y_img < ESDF_cols:
                        dist = ESDF_map[x_img, y_img]
                        if dist < min_dist_for_traj:
                            min_dist_for_traj = dist
                            closest_step_for_traj = i

        if min_dist_for_traj < hard_clearance:  # collision
            scores.append(float('inf'))
        elif min_dist_for_traj != float('inf'):
            if min_dist_for_traj > soft_clearance:
                scores.append(0.0)
            else:
                max_steps = len(traj)
                decay_factor = (max_steps - closest_step_for_traj) / max_steps
                # From the hard limit, so it blows up there. At the square's 1e-3 this is
                # the previous 1/(d + 1e-3).
                base_score = 1.0 / (min_dist_for_traj - hard_clearance + 1e-3)
                scores.append(decay_factor * base_score)
        else:
            scores.append(0.0)
        occ_points.append(closest_step_for_traj)
    return scores, occ_points
