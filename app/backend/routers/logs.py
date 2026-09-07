"""Bundle a single run's logs and hand them to the browser.

The board's logs are append-only and never scoped to a run: app.log has held
44k lines spanning a year, and board_health.log grows at 6 lines/minute
forever. Finding the 400 lines belonging to one navigation attempt meant
grepping the lot. So "start collecting" records byte offsets rather than
starting any writer, and "stop" slices everything from those offsets forward.
"""
import os
import json
import re
import shutil
import subprocess
import tarfile
import tempfile
import time
from pathlib import Path

from fastapi import APIRouter, HTTPException
from fastapi.responses import FileResponse

router = APIRouter(prefix='/logs', tags=['logs'])

# Only the never-rotated systemd logs need slicing. app.log and the node logs are
# already one file per run, and slicing app.log would cut off its run header --
# scheme and config -- which is written before anyone presses "start".
_SLICED = ('board_health.log', 'board_netheal.log')
_WHOLE = ('app.log',)
_NODES = ('map_node', 'build_map_node', 'diffcar_control', 'wheel_odometry',
          'looper_bridge', 'planning', 'cmd_vel_control', 'perception')
# 节点日志名是 <2026_08_24_16_47_12>_<node>.txt。不能用 glob：'*_map_node.txt' 会
# 匹配到 build_map_node，把建图日志当成导航的收进来。
_NODE_RE = r'^(\d{4}(?:_\d{2}){5})_%s\.txt$'
_MAX_BUNDLES = 10
# Per-process CPU/RSS and BPU are the one thing nothing samples continuously, and
# 10 s (board_health) is too coarse to see a stall. So this is the one writer the
# start button really does start.
# The Python sampler, not run_stats.sh: the shell one forked a `tr` per process per
# node name and measured 5.5 s a sample on this board, so it distorted the very load
# it was there to record.
_STATS_PY = '/userdata/x5/tinynav/tool/x5_board/run_stats.py'
_STATS_PERIOD_S = '1'
# 第二个"start 真的会启动"的写入者：指令 / 编码器实测 / VIO 三路速度的数值时间序列。
# 节点日志只说规划器**认为**发生了什么，这个说车**实际**怎么动的 —— 撞车分析要分开
# "决策错了"和"执行错了"，缺一边就只能猜。
# nice +10：板上 CPU 常态 7/8 核，瓶颈是排队不是吞吐，让它输给所有导航节点。
# TINYNAV_RUN_PROBE=0 关掉（要一次 app 重启），用来取"没有探针"的延迟基线。
_PROBE_PY = '/userdata/x5/tinynav/tool/x5_board/abcd_probe.py'
_PROBE_STREAMS = ('cmd', 'teleop', 'odom', 'vio')


def _nav_pose_topic() -> str:
    """导航实际闭环用的位姿话题。

    探针那一路的默认值是 /camera/camera/vio_image，而关掉固件 VIO 之后
    （user_params.json 的 vio_enabled=false）那个话题【根本不 advertise】——
    订阅它只会安静地产出一个空 CSV，采回来才发现白跑一趟。
    跟着 TINYNAV_ODOM_SOURCE 走，和 node_manager._control_pose_topic() 同一个判据。
    文件名仍叫 *_vio.csv：改名会打断所有已有的分析脚本，而"哪条话题"写在
    CSV 头里更可靠。"""
    return ('/wheel/camera_pose'
            if os.environ.get('TINYNAV_ODOM_SOURCE', 'vio') == 'wheel'
            else '/camera/camera/vio_image')
_PROBE_MAX_S = '7200'


def _log_dir() -> Path:
    return Path(os.environ.get('TINYNAV_APP_LOG_DIR', '/userdata/x5/logs'))


def _nodes_dir() -> Path:
    # app_start.sh already exports TINYNAV_LOG_DIR pointing at the per-node logs;
    # reusing it for the run log dir sent every lookup one level too deep.
    return Path(os.environ.get('TINYNAV_LOG_DIR', str(_log_dir() / 'nodes')))


def _bundle_dir() -> Path:
    d = Path(os.environ.get('TINYNAV_BUNDLE_DIR', '/userdata/x5/run_logs'))
    d.mkdir(parents=True, exist_ok=True)
    return d


def _marker_path() -> Path:
    return _log_dir() / '.collect_marker.json'


def _start_sampler() -> str | None:
    """1 Hz per-process sampler. Returns the TSV it will write, or None."""
    if not Path(_STATS_PY).exists():
        return None
    before = set(_log_dir().glob('run_stats_*.tsv'))
    try:
        subprocess.Popen(['python3', _STATS_PY, _STATS_PERIOD_S],
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                         start_new_session=True)
    except Exception:
        return None
    # The script names its own file from the clock, so the new one has to be found
    # rather than chosen -- it writes the header before the first sample.
    for _ in range(20):
        time.sleep(0.1)
        fresh = set(_log_dir().glob('run_stats_*.tsv')) - before
        if fresh:
            return fresh.pop().name
    return None


def _stop_sampler() -> None:
    subprocess.run(['pkill', '-f', f'python3 {_STATS_PY}'], capture_output=True)


def _probe_enabled() -> bool:
    return os.environ.get('TINYNAV_RUN_PROBE', '1').strip() not in ('0', 'false', 'no')


def _start_probe() -> str | None:
    """三路速度采集。返回文件名前缀（不含 _cmd.csv 那一截），或 None。"""
    if not _probe_enabled() or not Path(_PROBE_PY).exists():
        return None
    d = _log_dir()
    before = set(d.glob('abcd_*_odom.csv'))
    try:
        # 环境靠继承：app_start.sh 已经 source 过 env.sh，uvicorn 自己就带着 ROS 的
        # AMENT_PREFIX_PATH / PYTHONPATH，另拼一份反而会漏。
        subprocess.Popen(['nice', '-n', '10', 'python3', _PROBE_PY,
                          '--drive', 'none', '--duration', _PROBE_MAX_S,
                          '--vio-topic', _nav_pose_topic(),
                          '--out', str(d / 'abcd')],
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                         start_new_session=True)
    except Exception:
        return None
    # 它按自己的时钟命名，所以只能找不能猜；四个文件在 spin 之前就建好了。
    for _ in range(30):
        time.sleep(0.1)
        fresh = set(d.glob('abcd_*_odom.csv')) - before
        if fresh:
            return fresh.pop().name[:-len('_odom.csv')]
    return None


def _stop_probe() -> None:
    # SIGTERM 不是 SIGKILL：脚本自己捕获了它，会把缓冲刷干净再退。
    subprocess.run(['pkill', '-TERM', '-f', f'python3 {_PROBE_PY}'], capture_output=True)


def _probe_rows(prefix: str | None) -> dict | None:
    """每一路已经落了多少行。某一路 0 行本身就是证据：没人在发那个话题。"""
    if not prefix:
        return None
    out = {}
    for name in _PROBE_STREAMS:
        try:
            out[name] = max(0, (_log_dir() / f'{prefix}_{name}.csv'
                                ).read_bytes().count(b'\n') - 1)
        except Exception:
            out[name] = None
    return out


def _validate_bundle(name: str) -> str:
    if not re.fullmatch(r'run_[0-9]{8}_[0-9]{6}\.tar\.gz', name):
        raise HTTPException(400, 'Invalid bundle name')
    return name


def _read_marker() -> dict | None:
    p = _marker_path()
    if not p.exists():
        return None
    try:
        m = json.loads(p.read_text())
        return m if isinstance(m, dict) else None
    except Exception:
        return None


def _slice(path: Path, offset: int) -> bytes:
    """Bytes written since the marker. Falls back to the whole file if it shrank
    under us (rotation) -- slicing at a stale offset would silently lose the run."""
    if not path.exists():
        return b''
    start = 0 if path.stat().st_size < offset else offset
    with path.open('rb') as f:
        f.seek(start)
        return f.read()


def _app_started_at() -> float | None:
    p = _log_dir() / 'app.pid'
    return p.stat().st_mtime if p.exists() else None


def _newest_node_logs(since: float) -> tuple[list[Path], list[str]]:
    """This run's node logs, plus the names of the nodes that produced none.

    Selection is on the stamp in the filename, not mtime: diffcar_control logs
    four lines at startup and then goes quiet, so its mtime falls before whenever
    someone pressed "start" and an mtime rule drops it. The cutoff is the earlier
    of app start and the marker -- app start normally, the marker when collection
    spans a restart and both sessions are wanted.

    Stale logs are excluded rather than included-just-in-case: an 08-18
    wheel_odometry.txt shipped in one of these bundles was read as current and
    sent a diagnosis down the wrong path.
    """
    nodes = _nodes_dir()
    if not nodes.is_dir():
        return [], list(_NODES)
    started = _app_started_at()
    cutoff = min(started, since) if started else since
    files, missing = [], []
    for n in _NODES:
        pat = re.compile(_NODE_RE % re.escape(n))
        cand = []
        for p in nodes.iterdir():
            m = pat.match(p.name)
            if not m:
                continue
            try:
                stamp = time.mktime(time.strptime(m.group(1), '%Y_%m_%d_%H_%M_%S'))
            except ValueError:
                continue
            # 1 s of slack: the stamp is formatted a moment before app.pid lands.
            if stamp >= cutoff - 1.0:
                cand.append((stamp, p))
        if cand:
            files.append(max(cand)[1])
        else:
            missing.append(n)
    return files, missing


def _prune() -> None:
    bundles = sorted(_bundle_dir().glob('run_*.tar.gz'),
                     key=lambda p: p.stat().st_mtime, reverse=True)
    for p in bundles[_MAX_BUNDLES:]:
        p.unlink(missing_ok=True)


@router.get('/collect/status')
def collect_status() -> dict:
    m = _read_marker()
    return {
        'collecting': m is not None,
        'startedAt': m.get('t') if m else None,
        'elapsedS': round(time.time() - m['t'], 1) if m else None,
        'label': m.get('label') if m else None,
        'probe': m.get('probe') if m else None,
        'probeRows': _probe_rows(m.get('probe')) if m else None,
    }


@router.post('/collect/start')
def collect_start(label: str = '') -> dict:
    d = _log_dir()
    marker = {'t': time.time(), 'label': label[:80], 'files': {}}
    for name in _SLICED:
        p = d / name
        marker['files'][name] = p.stat().st_size if p.exists() else 0
    _stop_sampler()          # 上一次没正常停的话，别留两个采样器同时写
    _stop_probe()
    marker['stats'] = _start_sampler()
    marker['probe'] = _start_probe()
    _marker_path().write_text(json.dumps(marker, indent=2))
    return collect_status()


@router.post('/collect/stop')
def collect_stop() -> dict:
    m = _read_marker()
    if m is None:
        raise HTTPException(409, 'Not collecting')
    _stop_sampler()
    _stop_probe()
    time.sleep(0.4)          # 探针捕获了 SIGTERM，给它把最后一批行刷完的时间
    d = _log_dir()
    stamp = time.strftime('%Y%m%d_%H%M%S', time.localtime(m['t']))
    out = _bundle_dir() / f'run_{stamp}.tar.gz'
    contents: list[dict] = []

    with tempfile.TemporaryDirectory() as tmp:
        staged = Path(tmp)
        for name in _SLICED:
            data = _slice(d / name, m['files'].get(name, 0))
            if data:
                (staged / name).write_bytes(data)
                contents.append({'name': name, 'size': len(data),
                                 'lines': data.count(b'\n')})
        stats = m.get('stats')
        if stats and (d / stats).exists():
            shutil.copy2(d / stats, staged / stats)
            raw = (d / stats).read_bytes()
            contents.append({'name': stats, 'size': len(raw),
                             'lines': raw.count(b'\n')})
        probe = m.get('probe')
        if probe:
            pdir = staged / 'abcd'
            pdir.mkdir()
            for name in _PROBE_STREAMS:
                src = d / f'{probe}_{name}.csv'
                if src.exists() and src.stat().st_size:
                    shutil.copy2(src, pdir / src.name)
                    raw = src.read_bytes()
                    contents.append({'name': f'abcd/{src.name}', 'size': len(raw),
                                     'lines': raw.count(b'\n')})
        # 跟踪器的逐条指令记录。它是唯一能分开"规划器选错了"和"跟踪器执行反了"的东西。
        dbg = d / 'cmd_vel_debug'
        if dbg.is_dir():
            ddir = staged / 'cmd_vel_debug'
            ddir.mkdir()
            for src in sorted(dbg.iterdir()):
                if src.is_file() and src.stat().st_size:
                    shutil.copy2(src, ddir / src.name)
                    raw = src.read_bytes()
                    contents.append({'name': f'cmd_vel_debug/{src.name}',
                                     'size': len(raw), 'lines': raw.count(b'\n')})
        for name in _WHOLE:
            src = d / name
            if src.exists() and src.stat().st_size:
                shutil.copy2(src, staged / name)
                raw = src.read_bytes()
                contents.append({'name': name, 'size': len(raw),
                                 'lines': raw.count(b'\n')})
        nodes_dir = staged / 'nodes'
        nodes_dir.mkdir()
        node_files, node_missing = _newest_node_logs(m['t'])
        for p in node_files:
            shutil.copy2(p, nodes_dir / p.name)
            raw = p.read_bytes()
            contents.append({'name': f'nodes/{p.name}', 'size': len(raw),
                             'lines': raw.count(b'\n')})
        # dmesg is not in any file: the ring buffer is the only place a USB
        # re-enumeration or an OOM kill leaves a trace, and it dies with power.
        try:
            dmesg = subprocess.run(['dmesg'], capture_output=True, timeout=10).stdout
            if dmesg:
                (staged / 'dmesg.txt').write_bytes(dmesg)
                contents.append({'name': 'dmesg.txt', 'size': len(dmesg),
                                 'lines': dmesg.count(b'\n')})
        except Exception:
            pass
        # 哪些节点这次一个字都没写下来 —— 这本身就是证据，不该靠"包里少一个文件"去推。
        (staged / 'run.json').write_text(json.dumps(
            {'startedAt': m['t'], 'stoppedAt': time.time(),
             'label': m.get('label', ''), 'nodesSilent': node_missing,
             'probe': m.get('probe'), 'probeRows': _probe_rows(m.get('probe'))},
            indent=2))
        with tarfile.open(out, 'w:gz') as tf:
            for item in sorted(staged.iterdir()):
                tf.add(item, arcname=item.name)

    _marker_path().unlink(missing_ok=True)
    _prune()
    return {'name': out.name, 'size': out.stat().st_size,
            'durationS': round(time.time() - m['t'], 1), 'contents': contents,
            'nodesSilent': node_missing, 'probeRows': _probe_rows(m.get('probe'))}


@router.get('/bundles')
def list_bundles() -> dict:
    bundles = sorted(_bundle_dir().glob('run_*.tar.gz'),
                     key=lambda p: p.stat().st_mtime, reverse=True)
    return {'files': [{'name': p.name, 'size': p.stat().st_size,
                       'mtime': p.stat().st_mtime} for p in bundles]}


@router.get('/bundles/{name}')
def download_bundle(name: str) -> FileResponse:
    p = _bundle_dir() / _validate_bundle(name)
    if not p.exists():
        raise HTTPException(404, 'No such bundle')
    return FileResponse(p, media_type='application/gzip', filename=name)


@router.delete('/bundles/{name}')
def delete_bundle(name: str) -> dict:
    p = _bundle_dir() / _validate_bundle(name)
    if not p.exists():
        raise HTTPException(404, 'No such bundle')
    p.unlink()
    return {'deleted': name}
