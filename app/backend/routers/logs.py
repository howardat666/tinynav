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
    }


@router.post('/collect/start')
def collect_start(label: str = '') -> dict:
    d = _log_dir()
    marker = {'t': time.time(), 'label': label[:80], 'files': {}}
    for name in _SLICED:
        p = d / name
        marker['files'][name] = p.stat().st_size if p.exists() else 0
    _stop_sampler()          # 上一次没正常停的话，别留两个采样器同时写
    marker['stats'] = _start_sampler()
    _marker_path().write_text(json.dumps(marker, indent=2))
    return collect_status()


@router.post('/collect/stop')
def collect_stop() -> dict:
    m = _read_marker()
    if m is None:
        raise HTTPException(409, 'Not collecting')
    _stop_sampler()
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
             'label': m.get('label', ''), 'nodesSilent': node_missing}, indent=2))
        with tarfile.open(out, 'w:gz') as tf:
            for item in sorted(staged.iterdir()):
                tf.add(item, arcname=item.name)

    _marker_path().unlink(missing_ok=True)
    _prune()
    return {'name': out.name, 'size': out.stat().st_size,
            'durationS': round(time.time() - m['t'], 1), 'contents': contents,
            'nodesSilent': node_missing}


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
