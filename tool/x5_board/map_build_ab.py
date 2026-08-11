#!/usr/bin/env python3
"""Build the same bag into a VIO map and an odometry map, and time both. Runs ON THE BOARD.

    bash tool/x5_board/app_start.sh stop
    python3 tool/x5_board/map_build_ab.py --bag /userdata/x5/tinynav_db/rosbags/bag_...

WHY ONE BAG AND NOT TWO RUNS OF THE ROBOT
    Driving the same corridor twice gives two different paths, two different sets of
    keyframes and two different lighting moments, so any difference between the maps
    is confounded with the drive. run_rosbag_record.sh already records both
    /camera/camera/vio_image and /wheel/camera_pose for exactly this reason: the
    build takes its pose from a topic, so a single recording answers "how much worse
    is the odometry map" with the drive held fixed.

WHY IT IMPORTS node_manager INSTEAD OF SPELLING OUT THE ARGV
    The bridge and build_map_node take a dozen tuned arguments -- sync queue depth,
    sync window, play rate, vocabulary, skip topics -- whose values differ between
    the live and offline paths and were already found drifted once between the two
    call sites that build them. Importing the builders means this tool cannot drift
    from what the app does; it can only differ in TINYNAV_MAP_ODOM_SOURCE, which is
    the one thing being compared. That variable is read at import, so each source
    runs in its own child process with the environment already set.

WHY THE APP MUST BE STOPPED
    The board has 1338 MB and no swap. The vocabulary alone peaks at 407 MB and the
    running app holds ~950 MB during navigation, so a build alongside it is an
    OOM kill rather than a slow build.
"""

from __future__ import annotations

import argparse
import json
import os
import re
import shutil
import signal
import subprocess
import sys
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
_TINYNAV_ROOT = os.path.abspath(os.path.join(_HERE, '..', '..'))
ENV_SH = os.environ.get('ENV_SH', '/userdata/x5/env.sh')
DB_PATH = os.environ.get('TINYNAV_DB_PATH', '/userdata/x5/tinynav_db')
LOG_DIR = os.environ.get('LOG_DIR', '/userdata/x5/logs')
# Away from the app's domain so a forgotten node cannot feed or steal keyframes.
BUILD_DOMAIN_ID = os.environ.get('TINYNAV_AB_DOMAIN_ID', '77')


def sh(cmd, timeout=60):
    return subprocess.run(cmd, shell=True, capture_output=True, text=True,
                          timeout=timeout, check=False)


def dir_bytes(path):
    total = 0
    for root, _, files in os.walk(path):
        for f in files:
            try:
                total += os.path.getsize(os.path.join(root, f))
            except OSError:
                pass
    return total


def reexec_with_env():
    """Re-run under env.sh if it has not been sourced.

    Without it rosbag2_py cannot find libtinyxml2.so.9, and the failure surfaces as
    an import error deep inside the build rather than at startup."""
    if os.environ.get('_MAP_AB_ENV') == '1':
        return
    if not os.path.exists(ENV_SH):
        sys.exit(f'missing {ENV_SH} -- ROS and its deps will not resolve')
    os.environ['_MAP_AB_ENV'] = '1'
    quoted = ' '.join(f"'{a}'" for a in [sys.executable, os.path.abspath(__file__)] + sys.argv[1:])
    os.execvp('bash', ['bash', '-c', f'. {ENV_SH} && exec {quoted}'])


# --------------------------------------------------------------------------- #
# One build
# --------------------------------------------------------------------------- #

def build_one(source: str, bag: str, map_path: str, log_path: str) -> dict:
    """Runs in a child process whose TINYNAV_MAP_ODOM_SOURCE is already set."""
    sys.path.insert(0, _TINYNAV_ROOT)
    from app.backend.node_manager import (  # noqa: E402
        _bridge_argv, _build_map_argv, _MAP_SKIP_TOPICS_LOOPER, _POSE_TOPIC_WHEEL,
    )

    bridge_argv = _bridge_argv(for_map_build=True)
    build_argv = _build_map_argv(map_path, bag, skip_topics=_MAP_SKIP_TOPICS_LOOPER)
    expect = _POSE_TOPIC_WHEEL if source == 'wheel' else '/camera/camera/vio_image'
    if expect not in bridge_argv:
        raise SystemExit(f'bridge argv does not use {expect}: {bridge_argv}')

    if os.path.exists(map_path):
        shutil.rmtree(map_path)

    env = dict(os.environ)
    env['ROS_DOMAIN_ID'] = BUILD_DOMAIN_ID
    env.setdefault('NUMBA_CACHE_DIR', '/userdata/x5/numba_cache')
    os.makedirs(env['NUMBA_CACHE_DIR'], exist_ok=True)

    with open(log_path, 'w') as log:
        log.write(f'# source={source}\n# bridge: {" ".join(bridge_argv)}\n'
                  f'# build : {" ".join(build_argv)}\n\n')
        log.flush()
        bridge = subprocess.Popen(bridge_argv, cwd=_TINYNAV_ROOT, env=env,
                                  preexec_fn=os.setsid, stdout=log,
                                  stderr=subprocess.STDOUT)
        # The bridge pays a numba warmup before it can match anything; starting the
        # build first drops the keyframes played during it, silently.
        time.sleep(20)
        t0 = time.monotonic()
        build = subprocess.Popen(build_argv, cwd=_TINYNAV_ROOT, env=env,
                                 preexec_fn=os.setsid, stdout=log,
                                 stderr=subprocess.STDOUT)
        rc = build.wait()
        wall = time.monotonic() - t0
        try:
            os.killpg(os.getpgid(bridge.pid), signal.SIGTERM)
            bridge.wait(timeout=20)
        except (ProcessLookupError, subprocess.TimeoutExpired):
            pass

    return {'source': source, 'rc': rc, 'wall_s': wall, 'map_path': map_path,
            'log': log_path, **scan_log(log_path), 'bytes': dir_bytes(map_path)}


_KEYFRAME_RE = re.compile(r'\bmapping_loop\b.*?count[=: ]+(\d+)', re.I)
_STAGE_RE = re.compile(r'^\s*(\w+)\s+.*?total[=: ]+([\d.]+)', re.I)


def scan_log(path: str) -> dict:
    """Keyframe count and error tally. Deliberately tolerant: build_map_node's timer
    format has changed before, and a missing count must not fail the comparison."""
    keyframes, errors = None, 0
    try:
        with open(path, errors='replace') as fh:
            for line in fh:
                if 'Traceback' in line or 'ERROR' in line:
                    errors += 1
                m = _KEYFRAME_RE.search(line)
                if m:
                    keyframes = int(m.group(1))
    except OSError:
        pass
    return {'keyframes': keyframes, 'errors': errors}


# --------------------------------------------------------------------------- #
# Driver
# --------------------------------------------------------------------------- #

def app_running() -> bool:
    """Ask app_start.sh's pidfile, not pgrep. The backend runs as a plain `python3
    -m uvicorn ...`, so no pattern spelling the app's name matches it and a pgrep
    check silently reports "stopped" while 950 MB is resident."""
    pidfile = os.path.join(LOG_DIR, 'app.pid')
    try:
        with open(pidfile) as fh:
            pid = int(fh.read().strip())
        os.kill(pid, 0)
        return True
    except (OSError, ValueError):
        return False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--bag', required=True, help='bag directory to build from')
    ap.add_argument('--sources', default='vio,wheel',
                    help='comma separated: vio, wheel, or both (default both)')
    ap.add_argument('--tag', default='ab', help='suffix for the output map names')
    ap.add_argument('--force', action='store_true',
                    help='build even if the app looks like it is running')
    ap.add_argument('--_single', default='', help=argparse.SUPPRESS)
    ap.add_argument('--_map-path', default='', help=argparse.SUPPRESS)
    ap.add_argument('--_log-path', default='', help=argparse.SUPPRESS)
    args = ap.parse_args()

    reexec_with_env()

    if args._single:
        print(json.dumps(build_one(args._single, args.bag, args._map_path, args._log_path)))
        return 0

    if not os.path.isdir(args.bag):
        print(f'no such bag: {args.bag}')
        return 1
    if app_running() and not args.force:
        print('the app is running; a build alongside it is OOM-killed on this board.\n'
              '  bash tool/x5_board/app_start.sh stop      (or pass --force)')
        return 1

    sources = [s.strip() for s in args.sources.split(',') if s.strip()]
    os.makedirs(LOG_DIR, exist_ok=True)
    results = []
    for source in sources:
        name = f'map_{args.tag}_{source}'
        map_path = os.path.join(DB_PATH, 'maps', name)
        log_path = os.path.join(LOG_DIR, f'build_{args.tag}_{source}.log')
        print(f'\n=== building {source} map -> {map_path}')
        print(f'    log: {log_path}')
        env = dict(os.environ)
        env['TINYNAV_MAP_ODOM_SOURCE'] = source
        child = subprocess.run(
            [sys.executable, os.path.abspath(__file__), '--bag', args.bag,
             '--_single', source, '--_map-path', map_path, '--_log-path', log_path],
            env=env, capture_output=True, text=True)
        if child.returncode != 0 or not child.stdout.strip():
            print(f'    FAILED rc={child.returncode}\n{child.stderr[-2000:]}')
            continue
        r = json.loads(child.stdout.strip().splitlines()[-1])
        results.append(r)
        print(f'    rc={r["rc"]}  {r["wall_s"] / 60:.2f} min  '
              f'keyframes={r["keyframes"]}  {r["bytes"] / 1e6:.0f} MB  errors={r["errors"]}')

    if len(results) < 2:
        return 0
    print('\n' + '=' * 62)
    a, b = results[0], results[1]
    print(f'  {"":<12}{a["source"]:>14}{b["source"]:>14}')
    print(f'  {"wall min":<12}{a["wall_s"] / 60:>14.2f}{b["wall_s"] / 60:>14.2f}')
    print(f'  {"keyframes":<12}{str(a["keyframes"]):>14}{str(b["keyframes"]):>14}')
    print(f'  {"MB":<12}{a["bytes"] / 1e6:>14.0f}{b["bytes"] / 1e6:>14.0f}')
    if a['keyframes'] and b['keyframes'] and a['keyframes'] != b['keyframes']:
        print('\n  ⚠️  different keyframe counts: the two maps do not cover the same\n'
              '      places, so size and timing are not comparable. The usual cause is\n'
              '      one pose topic having gaps in the bag.')
    print('\n  Timing is NOT the point of this comparison -- both builds do the same\n'
          '  visual work. What matters is map quality, which needs a relocalization\n'
          '  run against each map.')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
