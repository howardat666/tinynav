"""Numba A* kernels for the SDF map, kept in their own module on purpose.

numba invalidates a file's whole disk cache on any change to that file (the index
stores its mtime and size), so leaving these in map_node.py meant every edit to node
logic cost a 55 s recompile on the board. Here they only recompile when they change.
"""

import numpy as np
from numba import njit


@njit(cache=True)
def _nav_flat_index(x: int, y: int, z: int, ny: int, nz: int) -> int:
    return (x * ny + y) * nz + z

@njit(cache=True)
def _nav_unflat_index(index: int, ny: int, nz: int):
    x = index // (ny * nz)
    remainder = index - x * ny * nz
    y = remainder // nz
    z = remainder - y * nz
    return x, y, z

@njit(cache=True)
def _nav_heap_less(bucket_a: int, cost_a: float, node_a: int, bucket_b: int, cost_b: float, node_b: int) -> bool:
    if bucket_a != bucket_b:
        return bucket_a < bucket_b
    if cost_a != cost_b:
        return cost_a < cost_b
    return node_a < node_b

@njit(cache=True)
def _nav_heap_push(heap_nodes: np.ndarray, heap_buckets: np.ndarray, heap_costs: np.ndarray, heap_size: int, node: int, bucket: int, cost: float) -> int:
    i = heap_size
    heap_nodes[i] = node
    heap_buckets[i] = bucket
    heap_costs[i] = cost
    while i > 0:
        parent = (i - 1) // 2
        if not _nav_heap_less(heap_buckets[i], heap_costs[i], heap_nodes[i], heap_buckets[parent], heap_costs[parent], heap_nodes[parent]):
            break
        tmp_node = heap_nodes[parent]
        tmp_bucket = heap_buckets[parent]
        tmp_cost = heap_costs[parent]
        heap_nodes[parent] = heap_nodes[i]
        heap_buckets[parent] = heap_buckets[i]
        heap_costs[parent] = heap_costs[i]
        heap_nodes[i] = tmp_node
        heap_buckets[i] = tmp_bucket
        heap_costs[i] = tmp_cost
        i = parent
    return heap_size + 1

@njit(cache=True)
def _nav_heap_pop(heap_nodes: np.ndarray, heap_buckets: np.ndarray, heap_costs: np.ndarray, heap_size: int):
    node = heap_nodes[0]
    bucket = heap_buckets[0]
    cost = heap_costs[0]
    heap_size -= 1
    if heap_size > 0:
        heap_nodes[0] = heap_nodes[heap_size]
        heap_buckets[0] = heap_buckets[heap_size]
        heap_costs[0] = heap_costs[heap_size]
        i = 0
        while True:
            left = 2 * i + 1
            right = left + 1
            smallest = i
            if left < heap_size and _nav_heap_less(
                heap_buckets[left], heap_costs[left], heap_nodes[left],
                heap_buckets[smallest], heap_costs[smallest], heap_nodes[smallest],
            ):
                smallest = left
            if right < heap_size and _nav_heap_less(
                heap_buckets[right], heap_costs[right], heap_nodes[right],
                heap_buckets[smallest], heap_costs[smallest], heap_nodes[smallest],
            ):
                smallest = right
            if smallest == i:
                break
            tmp_node = heap_nodes[smallest]
            tmp_bucket = heap_buckets[smallest]
            tmp_cost = heap_costs[smallest]
            heap_nodes[smallest] = heap_nodes[i]
            heap_buckets[smallest] = heap_buckets[i]
            heap_costs[smallest] = heap_costs[i]
            heap_nodes[i] = tmp_node
            heap_buckets[i] = tmp_bucket
            heap_costs[i] = tmp_cost
            i = smallest
    return node, bucket, cost, heap_size

@njit(cache=True)
def _nav_reconstruct_path(parent: np.ndarray, current: int, ny: int, nz: int) -> np.ndarray:
    count = 1
    node = current
    while parent[node] != node and parent[node] >= 0:
        node = parent[node]
        count += 1

    path = np.empty((count, 3), dtype=np.int32)
    node = current
    for i in range(count - 1, -1, -1):
        x, y, z = _nav_unflat_index(node, ny, nz)
        path[i, 0] = x
        path[i, 1] = y
        path[i, 2] = z
        if parent[node] == node or parent[node] < 0:
            break
        node = parent[node]
    return path

@njit(cache=True)
def _nav_sdf_bucket(sdf_value: float) -> int:
    if sdf_value < 0.2:
        return 0
    if sdf_value < 0.5:
        return 1
    if sdf_value < 1.0:
        return 2
    if sdf_value < 2.0:
        return 3
    if sdf_value < 5.0:
        return 4
    if sdf_value < 10.0:
        return 5
    return 6

@njit(cache=True)
def _nav_heuristic_idx(x: int, y: int, z: int, gx: int, gy: int, gz: int, resolution: float) -> float:
    dx = (x - gx) * resolution
    dy = (y - gy) * resolution
    dz = (z - gz) * resolution
    return np.sqrt(dx * dx + dy * dy + dz * dz) + 20.0 * np.abs(dz)

@njit(cache=True)
def search_close_to_sdf_map_numba(start_index: np.ndarray, sdf_map: np.ndarray, occupancy_map: np.ndarray, stop_distance: float) -> np.ndarray:
    nx, ny, nz = sdf_map.shape
    total = nx * ny * nz
    sx = int(start_index[0])
    sy = int(start_index[1])
    sz = int(start_index[2])
    start_node = _nav_flat_index(sx, sy, sz, ny, nz)

    parent = np.full(total, -1, dtype=np.int64)
    visited = np.zeros(total, dtype=np.bool_)
    in_open = np.zeros(total, dtype=np.bool_)
    heap_nodes = np.empty(total, dtype=np.int64)
    heap_buckets = np.zeros(total, dtype=np.int32)
    heap_costs = np.empty(total, dtype=np.float64)

    parent[start_node] = start_node
    in_open[start_node] = True
    heap_size = _nav_heap_push(heap_nodes, heap_buckets, heap_costs, 0, start_node, 0, float(sdf_map[sx, sy, sz]))

    while heap_size > 0:
        current, _, _, heap_size = _nav_heap_pop(heap_nodes, heap_buckets, heap_costs, heap_size)
        in_open[current] = False
        if visited[current]:
            continue
        visited[current] = True
        cx, cy, cz = _nav_unflat_index(current, ny, nz)
        current_sdf = float(sdf_map[cx, cy, cz])
        if current_sdf < stop_distance:
            return _nav_reconstruct_path(parent, current, ny, nz)

        for dx in range(-1, 2):
            for dy in range(-1, 2):
                for dz in range(-1, 2):
                    if dx == 0 and dy == 0 and dz == 0:
                        continue
                    nbx = cx + dx
                    nby = cy + dy
                    nbz = cz + dz
                    if nbx < 0 or nbx >= nx or nby < 0 or nby >= ny or nbz < 0 or nbz >= nz:
                        continue
                    if occupancy_map[nbx, nby, nbz] == 2:
                        continue
                    neighbor = _nav_flat_index(nbx, nby, nbz, ny, nz)
                    if visited[neighbor] or in_open[neighbor]:
                        continue
                    parent[neighbor] = current
                    in_open[neighbor] = True
                    heap_size = _nav_heap_push(heap_nodes, heap_buckets, heap_costs, heap_size, neighbor, 0, float(sdf_map[nbx, nby, nbz]))

    return np.empty((0, 3), dtype=np.int32)

@njit(cache=True)
def search_within_sdf_map_numba(start: np.ndarray, goal: np.ndarray, sdf_map: np.ndarray, occupancy_map: np.ndarray, resolution: float) -> np.ndarray:
    nx, ny, nz = sdf_map.shape
    total = nx * ny * nz
    sx = int(start[0])
    sy = int(start[1])
    sz = int(start[2])
    gx = int(goal[0])
    gy = int(goal[1])
    gz = int(goal[2])
    start_node = _nav_flat_index(sx, sy, sz, ny, nz)
    goal_node = _nav_flat_index(gx, gy, gz, ny, nz)

    parent = np.full(total, -1, dtype=np.int64)
    visited = np.zeros(total, dtype=np.bool_)
    in_open = np.zeros(total, dtype=np.bool_)
    heap_nodes = np.empty(total, dtype=np.int64)
    heap_buckets = np.empty(total, dtype=np.int32)
    heap_costs = np.empty(total, dtype=np.float64)

    parent[start_node] = start_node
    in_open[start_node] = True
    start_bucket = _nav_sdf_bucket(float(sdf_map[sx, sy, sz]))
    start_cost = _nav_heuristic_idx(sx, sy, sz, gx, gy, gz, resolution)
    heap_size = _nav_heap_push(heap_nodes, heap_buckets, heap_costs, 0, start_node, start_bucket, start_cost)

    while heap_size > 0:
        current, _, _, heap_size = _nav_heap_pop(heap_nodes, heap_buckets, heap_costs, heap_size)
        in_open[current] = False
        if visited[current]:
            continue
        visited[current] = True
        if current == goal_node:
            return _nav_reconstruct_path(parent, current, ny, nz)

        cx, cy, cz = _nav_unflat_index(current, ny, nz)
        for dx in range(-1, 2):
            for dy in range(-1, 2):
                for dz in range(-1, 2):
                    if dx == 0 and dy == 0 and dz == 0:
                        continue
                    nbx = cx + dx
                    nby = cy + dy
                    nbz = cz + dz
                    if nbx < 0 or nbx >= nx or nby < 0 or nby >= ny or nbz < 0 or nbz >= nz:
                        continue
                    if occupancy_map[nbx, nby, nbz] == 2:
                        continue
                    neighbor = _nav_flat_index(nbx, nby, nbz, ny, nz)
                    if visited[neighbor] or in_open[neighbor]:
                        continue
                    parent[neighbor] = current
                    in_open[neighbor] = True
                    bucket = _nav_sdf_bucket(float(sdf_map[nbx, nby, nbz]))
                    cost = _nav_heuristic_idx(nbx, nby, nbz, gx, gy, gz, resolution)
                    heap_size = _nav_heap_push(heap_nodes, heap_buckets, heap_costs, heap_size, neighbor, bucket, cost)

    return np.empty((0, 3), dtype=np.int32)
