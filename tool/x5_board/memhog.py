#!/usr/bin/env python3
"""占住 N MB 常驻内存（真正写页，不是只申请）。用法: memhog.py <MB> <秒数>"""
import sys, time
MB = int(sys.argv[1]); DUR = float(sys.argv[2])
blocks = []
for _ in range(MB // 50):
    b = bytearray(50 * 1024 * 1024)
    for i in range(0, len(b), 4096):
        b[i] = 1
    blocks.append(b)
t = time.monotonic()
while time.monotonic() - t < DUR:
    time.sleep(0.5)
