#!/usr/bin/env python3
"""裸 TCP 吞吐测量。两端只要 python3。**取接收端的计时** —— sendall 只写进缓冲就返回，
两端读数实测差过 7 秒。用法见 docs/x5/board_bringup.md 2.5 节。"""
import socket, sys, time
MODE, HOST, PORT = sys.argv[1], sys.argv[2], int(sys.argv[3])
NBYTES = int(sys.argv[4]) if len(sys.argv) > 4 else 3 * 1024 * 1024
CHUNK = 64 * 1024
BLOB = b'\xa5' * CHUNK

if MODE == 'recv':                       # 接收端计时（权威）
    s = socket.socket(); s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(('0.0.0.0', PORT)); s.listen(1)
    print(f'listening on {PORT}', flush=True)
    c, addr = s.accept()
    n, t0 = 0, None
    while True:
        b = c.recv(CHUNK)
        if not b: break
        if t0 is None: t0 = time.monotonic()
        n += len(b)
    dt = max(1e-9, time.monotonic() - t0)
    print(f'RECV {n} B in {dt:.2f} s = {8*n/dt/1e6:.3f} Mbit/s', flush=True)
    c.close(); s.close()
elif MODE == 'send':
    c = socket.create_connection((HOST, PORT), timeout=60)
    sent, t0 = 0, time.monotonic()
    while sent < NBYTES:
        c.sendall(BLOB); sent += CHUNK
    c.shutdown(socket.SHUT_WR)
    dt = time.monotonic() - t0
    print(f'SEND {sent} B in {dt:.2f} s = {8*sent/dt/1e6:.3f} Mbit/s (发送端读数，偏乐观)', flush=True)
    c.close()
