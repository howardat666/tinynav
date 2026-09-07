"""全链延迟统计。

每个节点在自己的入口/出口记一次「now − 原始戳」，窗口到点打一行 p50/p90/max。
之所以能这样拆：depth 的 header.stamp 被 bridge 的 relabel_depth 原样传递，
planning 又把它写进 path.header.stamp，所以从相机到轨迹话题是同一个戳 ——
各节点量到的数直接相减就是那一跳的耗时。位姿是另一条戳（发布时打的 now），
所以两条必须分开记，不能混在一个 in_age 里。

日志格式固定为 `LAT <tag> key=p50/p90/max(n) ...`，由 tool/x5_board/lat_report.py 解析。
"""
import os
import time


class LatStats:
    def __init__(self, tag, log_fn, period_s=None):
        self.tag = tag
        self._log = log_fn
        if period_s is None:
            period_s = os.environ.get('TINYNAV_LAT_LOG_S', '10')
        self._period = float(period_s)
        self._d = {}
        self._t0 = time.monotonic()

    @property
    def enabled(self):
        return self._period > 0

    def add(self, key, value_s):
        if self._period > 0:
            self._d.setdefault(key, []).append(float(value_s))

    def tick(self):
        """每周期末调用一次；窗口未到就立即返回。"""
        if self._period <= 0 or not self._d:
            return
        now = time.monotonic()
        if now - self._t0 < self._period:
            return
        parts = []
        for k in sorted(self._d):
            v = sorted(self._d[k])
            n = len(v)
            parts.append(f"{k}={v[n // 2]:.3f}/{v[min(n - 1, int(n * 0.9))]:.3f}/{v[-1]:.3f}({n})")
        self._log(f"LAT {self.tag} " + " ".join(parts))
        self._d.clear()
        self._t0 = now
