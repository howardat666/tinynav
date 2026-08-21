import socket

import psutil
from fastapi import APIRouter

from ..manager_client import get_manager_json, is_display_role
from ..state import runner

router = APIRouter(tags=['device'])


@router.get('/info')
def device_info():
    return {
        'deviceId': socket.gethostname(),
        'firmwareVersion': '0.1.0',
        'capabilities': ['bag_record', 'map_build', 'navigation'],
    }


@router.get('/status')
def device_status():
    if is_display_role():
        manager_status = get_manager_json('/device/status')
        if manager_status is not None:
            return manager_status

    node = runner.node
    if node is None:
        return {'online': False, 'battery': None, **_empty_status()}
    status = node.get_status()
    return {
        'online': True,
        'battery': None,   # no battery topic yet
        **status,
    }


@router.get('/platform')
def device_platform():
    """Robot geometry, actuator and odometry source this backend launched with.

    Read-only: these come from environment variables that have to be fixed before
    the first node starts, so there is nothing to PUT. It exists because the
    comparison runs differ only in those variables and each one fails quietly when
    set wrong -- the wrong robot_type merely tracks badly, the wrong pose topic
    merely never relocalizes.
    """
    if is_display_role():
        manager_platform = get_manager_json('/device/platform')
        if manager_platform is not None:
            return manager_platform

    node = runner.node
    if node is None:
        return {'online': False}
    return {'online': True, **node.get_platform_config()}


# Where the data actually lives. disk_usage('/') answered for a 2.0 GB rootfs while
# maps, bags and code sat on a 53 GB /userdata -- the panel read 46% full when the
# volume that can actually fill up was at 43% of fifty gigabytes. Reported per mount,
# because 'disk' is not one number on this board.
_STORAGE_LABELS = (('data', None), ('root', '/'), ('app', '/app'))


def _storage_entries() -> list[dict]:
    from .files import _db_root
    seen, out = set(), []
    for label, path in _STORAGE_LABELS:
        p = str(_db_root()) if path is None else path
        try:
            u = psutil.disk_usage(p)
        except OSError:
            continue
        # Same device twice is one row: where the db lives under / there is no separate
        # data partition to report.
        key = (u.total, u.used)
        if key in seen:
            continue
        seen.add(key)
        out.append({'label': label, 'path': p, 'percent': round(u.percent, 1),
                    'used_gb': round(u.used / 1024 ** 3, 2),
                    'total_gb': round(u.total / 1024 ** 3, 2)})
    return out


def _meminfo_kb() -> dict:
    try:
        out = {}
        with open('/proc/meminfo') as f:
            for line in f:
                k, _, v = line.partition(':')
                out[k] = int(v.split()[0])
        return out
    except (OSError, ValueError, IndexError):
        return {}


def _mem_breakdown() -> dict:
    mi = _meminfo_kb()

    def mb(kb):
        return round(kb / 1024.0, 1)

    return {
        # available, not free: free excludes reclaimable cache and reads alarmingly low
        # on a healthy board. available is what a new allocation can actually get.
        'available_mb': mb(mi.get('MemAvailable', 0)),
        'free_mb': mb(mi.get('MemFree', 0)),
        'cached_mb': mb(mi.get('Cached', 0) + mi.get('Buffers', 0)),
        'shmem_mb': mb(mi.get('Shmem', 0)),
        'swap_total_mb': mb(mi.get('SwapTotal', 0)),
        'swap_free_mb': mb(mi.get('SwapFree', 0)),
        # Contiguous memory carved out for the camera and BPU pipelines, and not part of
        # MemAvailable -- shrinking it is how this board went from 1.3 to 1.7 GB of
        # usable RAM, so it belongs next to the rest rather than nowhere.
        'cma_total_mb': mb(mi['CmaTotal']) if 'CmaTotal' in mi else None,
        'cma_free_mb': mb(mi['CmaFree']) if 'CmaFree' in mi else None,
    }


def _bpu_percent() -> float | None:
    # D-Robotics X5: a plain integer percent, the same number hrut_somstatus prints as
    # 'ratio'. Read from sysfs rather than shelling out to that tool, which costs
    # ~200 ms on a board where this endpoint is polled.
    for path in ('/sys/devices/system/bpu/ratio', '/sys/devices/system/bpu/bpu0/ratio'):
        try:
            with open(path) as f:
                return float(int(f.read().strip()))
        except (OSError, ValueError):
            continue
    return None


def _temps_c() -> dict:
    # Two zones on the X5, thermal-cpu and thermal-ddr; the BPU temperature is not
    # exposed here. Throttling starts at 95 C, so this is a headroom readout.
    import glob
    out = {}
    for zone in sorted(glob.glob('/sys/class/thermal/thermal_zone*')):
        try:
            with open(f'{zone}/type') as f:
                name = f.read().strip().replace('thermal-', '')
            with open(f'{zone}/temp') as f:
                out[name] = round(int(f.read().strip()) / 1000.0, 1)
        except (OSError, ValueError):
            continue
    return out


def _jetson_gpu_percent() -> float | None:
    # Jetson: /sys/devices/gpu.0/load reports 0-1000 (tenths of a percent)
    try:
        with open('/sys/devices/gpu.0/load') as f:
            return round(int(f.read().strip()) / 10.0, 1)
    except Exception:
        return None


@router.get('/sysinfo')
def device_sysinfo():
    # 一次阻塞采样同时给出合计和每核，两个数必须来自同一个时间窗。
    # 以前合计用 interval=0.2（真实窗口）而每核用 interval=0.0（距上次调用的增量，
    # 也就是距上次刷新这个面板的整段时间），后端刚起时那一次更是"开机至今的平均" ——
    # 于是面板上"合计"和"每核"天生对不上，而且第一眼看到的每核值是历史平均。
    per_core = psutil.cpu_percent(interval=0.2, percpu=True)
    cpu = sum(per_core) / max(1, len(per_core))
    mem = psutil.virtual_memory()
    storage = _storage_entries()
    # Kept so an older frontend keeps working: the first row, now the data volume.
    legacy = ({'disk_percent': storage[0]['percent'], 'disk_used_gb': storage[0]['used_gb'],
               'disk_total_gb': storage[0]['total_gb']} if storage else
              {'disk_percent': 0.0, 'disk_used_gb': 0.0, 'disk_total_gb': 0.0})
    return {
        'cpu_percent': round(cpu, 1),
        'cpu_per_core': [round(x, 1) for x in per_core],
        'load_1m': round(psutil.getloadavg()[0], 2),
        'mem_percent': round(mem.percent, 1),
        'mem_used_gb': round(mem.used / 1024 ** 3, 1),
        'mem_total_gb': round(mem.total / 1024 ** 3, 1),
        'mem_breakdown': _mem_breakdown(),
        'storage': storage,
        **legacy,
        'bpu_percent': _bpu_percent(),
        'temps_c': _temps_c(),
        'gpu_percent': _jetson_gpu_percent(),
    }


def _empty_status():
    return {
        'bagStatus': 'idle',
        'bagFileReady': False,
        'mapStatus': 'idle',
        'mappingPercent': 0.0,
        'navStatus': 'idle',
        'rawState': 'unknown',
        # Present-but-null rather than absent, so a client can read the key
        # unconditionally instead of branching on whether the node is up.
        'relocalization': None,
        'poiStatus': None,
    }
