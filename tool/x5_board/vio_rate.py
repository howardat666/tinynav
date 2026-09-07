import time, math
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

got = []
class P(Node):
    def __init__(s):
        super().__init__('vio_rate')
        # 队列开深一点：默认 depth=1 时执行器一忙就直接丢，量出来的是"我收到多少"
        # 而不是"话题发了多少"
        s.create_subscription(PoseStamped, '/camera/camera/vio_image',
            lambda m: got.append((time.monotonic(),
                                  m.header.stamp.sec + m.header.stamp.nanosec*1e-9)),
            QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT))

rclpy.init(); n = P()
t0 = time.monotonic()
while time.monotonic() - t0 < 20.0:
    rclpy.spin_once(n, timeout_sec=0.05)
if len(got) < 3:
    print("收到 %d 条，太少" % len(got)); rclpy.shutdown(); raise SystemExit
dur = got[-1][0] - got[0][0]
print("20 秒收到 %d 条，平均 %.2f Hz" % (len(got), (len(got)-1)/dur))
# 用消息自带的时间戳算间隔（不受本进程调度影响）
st = sorted(g[1] for g in got)
iv = sorted(st[i]-st[i-1] for i in range(1, len(st)))
f = lambda q: iv[min(int(len(iv)*q), len(iv)-1)]
print("消息时间戳间隔: p10=%.0fms p50=%.0fms p90=%.0fms max=%.0fms" % (
    f(.1)*1000, f(.5)*1000, f(.9)*1000, iv[-1]*1000))
print("超过固件旋转先验门限 0.15 s 的间隔: %d / %d (%.0f%%)" % (
    sum(1 for x in iv if x > 0.15), len(iv), 100*sum(1 for x in iv if x > 0.15)/len(iv)))
# 本进程收到的墙钟间隔（含调度抖动）
rv = sorted(got[i][0]-got[i-1][0] for i in range(1, len(got)))
print("本进程收到的墙钟间隔: p50=%.0fms p90=%.0fms max=%.0fms" % (
    rv[len(rv)//2]*1000, rv[int(len(rv)*.9)]*1000, rv[-1]*1000))
rclpy.shutdown()
