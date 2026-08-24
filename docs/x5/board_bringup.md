# 把 tinynav 的重定位跑在 Looper 相机内的 X5 上

**状态：跑通了。** ORB + DBoW3 的纯 CPU 重定位在 X5 上端到端运行，**p50 374 ms / p90 420 ms**，峰值 RSS **407 MB**，5 秒预算有约 **13 倍余量**。

这篇记录怎么从零把板子准备到这个状态，以及过程中撞到的每一个坑 —— 其中有四个是文档里没有、只能实测撞出来的。

- 硬件：Looper 相机内的 D-Robotics X5（8× Cortex-A55 @1.5 GHz，1× Bayes-e BPU，MemTotal **1307 MB**，无 swap）
- 相机固件：2.1.2，`insight_full`，已应用 [`depth_frame_skip.md`](depth_frame_skip.md) 的 5 Hz 降频和 [`fix_64gb_mipi.md`](fix_64gb_mipi.md) 的 1lane 修复
- 板上目录约定：一切都在 **`/userdata/x5/`** 下，不动固件的 `/userdata/install`

---

## 0. 快速上手

> **连哪台？** 同一台板子，USB 直连是 `root@169.254.10.1`；2026-08-18 加装 USB WiFi 后也可以走
> `DEEP-RD` → `root@192.168.19.218`（密码不变）。**无线的「发」方向只有约 1.7 Mbit/s：推文件上去可以，
> 从板上拉 bag 和看彩色预览都不行，见 §2.5。**
> 脚本一律用环境变量切目标，不改代码：`BOARD=root@192.168.19.218` / `HOST=...` / `BOARD_IP=...`。

```bash
# PC 侧：每次板子重启后都要跑（没有 RTC，见 §2）
tool/x5_board/sync_board_time.sh

# 板上：每个 shell 都要 source
. /userdata/x5/env.sh

# 跑一次重定位评测
python3 /userdata/x5/reloc_offline_eval.py \
    --map /userdata/x5/maps/map_gt \
    --query-map /userdata/x5/maps/map_day \
    --vocab /userdata/x5/voc/voc_office_k10L5.dbow3 \
    --transform-json /userdata/x5/results/transform_gt_day.json \
    --db-path /userdata/x5/scratch/reloc_db \
    --out-prefix /userdata/x5/results/x5_gt_day
```

板上布局：

| 路径 | 内容 |
|---|---|
| `/userdata/x5/env.sh` | 环境变量（§1） |
| `/userdata/x5/tinynav/` | tinynav 代码；`tinynav_cpp_bind.so` 必须放在包**内部**（§4） |
| `/userdata/x5/pydbow3/` | 交叉编译的 `pydbow3` + 4 个依赖库，3.2 MB |
| `/userdata/x5/cpp_bind/` | 交叉编译的 `tinynav_cpp_bind` + Ceres/SuiteSparse 等 12 个库，5.8 MB |
| `/userdata/x5/pylibs/` | 不走 pip 的纯 Python 包：`sensor_msgs_py`、`decord`（§5） |
| `/userdata/x5/wheels/` | 离线 wheel + `install_on_board.sh`，119 MB |
| `/userdata/x5/maps/` | `map_gt`、`map_day`，各 1.8 GB |
| `/userdata/x5/voc/` | `ORBvoc.dbow3`、`voc_office_k10L4/L5.dbow3` |
| `/userdata/x5/cache/numba/` | numba 磁盘缓存，决定启动是 66 s 还是 34 s（§6） |

---

## 1. 板上 ROS 2 的 Python 环境是坏的，但只是环境变量的问题

`/opt/ros/humble` 和 `/opt/tros/humble` 都是 `/userdata/hobot/opt/ros/humble` 的符号链接。固件自带的
`/etc/init.d/looper/setting/ros2_env.conf` 足够跑 C++ 的 `insight_full`，但跑任何 Python 节点都不行，**有四处缺失**：

| # | 问题 | 后果 |
|---|---|---|
| 1 | `PYTHONPATH` 只有 `humble/lib/python3.10/site-packages` | 那里只有 82 个 `ament_*` 构建工具。**`rclpy`、`cv_bridge`、`message_filters`、`tf2_py` 和所有消息模块都在 `humble/local/lib/python3.10/dist-packages`**（86 个包），完全没被引用 |
| 2 | `AMENT_PREFIX_PATH` 根本没设 | `ros2` CLI 报 `PackageNotFoundError: ros2cli` |
| 3 | `ROS_DISTRO` 是 `/opt/ros/humble/humble` | 是个路径，而且是错的；应该是发行版名 `humble` |
| 4 | Python 的 `bin/` 不在 `PATH` | `pip` 找不到，只能用 `python3 -m pip` |

> ⚠️ **我自己在这里栽过一次，值得写下来当反面教材。** 第一次排查时我用 `find ... -path "*site-packages*"` 找包，这个条件恰好把
> `local/lib/python3.10/dist-packages/` 排除掉了，于是我得出「板上 ROS 2 是纯 C++ 安装、Python 节点根本跑不了」的结论 ——
> 那会是个周级别的工程。实际上只是环境变量少了一行。**用 `find` 判断"某个东西不存在"时，先确认你的过滤条件没把它排除掉。**

修法见 [`tool/x5_board/env.sh`](../../tool/x5_board/env.sh)。**不改 `ros2_env.conf`** ——
它属于固件，OTA 会覆盖。修完 `ros2 node list` / `ros2 topic list` / `ros2 topic hz` 全部可用（这是板子历史上第一次）。

---

## 2. 时钟：两个独立的问题，必须一起修

### 2.1 X5 没有 RTC 电池

`hwclock -r` 读出 1970-01-01，也没有任何 NTP daemon 在跑（`ntpdate` 二进制存在但没人调用）。
实测板子开机后是 **2025-08-26**，而 PC 是 **2026-08-04** —— 差了近 11 个月。

`tool/x5_board/sync_board_time.sh` 用 NTP 的办法修（把远端读数夹在两次本地读数中间，补偿半个往返），
一轮就收敛到 **-2 ms**。**每次板子重启后都要重跑**，因为没电池。

### 2.2 `insight_full` 默认用 CLOCK_MONOTONIC 打时间戳

这个比时钟偏差隐蔽得多，也严重得多。实测：

```
uptime          = 850.76
rclpy clock now = 1785809011.750       ← REALTIME
depth  header.stamp = 838.571          ← 约等于 uptime
infra1 header.stamp = 838.721
imu    header.stamp = 838.757
```

相机给消息打的是**单调时钟**（≈开机秒数），而 tinynav 节点用
`self.get_clock().now()`（REALTIME）给自己的输出打戳 —— 两者差 **17.8 亿秒**。后果：

- 用图像时间戳查 TF 必然抛异常（在 t=1.78e9 填充的 buffer 里查 t=838）
- `message_filters` 的时间同步永远匹配不上，而且是**静默丢弃**，不报错

`map_node.py` 和 `planning_node.py` 里有 12 处 `get_clock().now()`，所以这不是理论问题。

**好消息是不用改代码。** 固件本来就支持墙上时钟，只是判定条件没满足
（`insight_full_node.cpp:321`）：

```cpp
std::ifstream f(kTimeSyncFlagPath);          // /etc/init.d/looper/setting/is_time_sync
if (f >> flag && flag == 1) {
  int64_t offset = computeRealtimeOffset();  // REALTIME - MONOTONIC
  threads_.time_offset_ns.store(offset, ...);
} else {
  RCLCPP_WARN(..., "NTP sync not ready (%s). Timestamps will use CLOCK_MONOTONIC.");
}
```

这个 flag 文件在我们的板子上**根本不存在**。而且 `insight_full_node_vio.cpp:500` 有个
`timeSyncMonitorThread()` 每秒重读一次，所以 **写入 `1` 立即生效，不用重启**。

`sync_board_time.sh` 在同步完时钟后会自动设这个 flag —— 两件事必须一起做：光同步时钟不设 flag，
戳还是单调的；光设 flag 不同步时钟，戳会是错误的墙上时间。设完之后：

```
depth  header.stamp = 1785810317.561   wall - stamp = +0.2 s   ← depth 是 5 Hz，首帧年龄正常
infra1 header.stamp = 1785810317.711   wall - stamp = +0.0 s
imu    header.stamp = 1785810317.733   wall - stamp = +0.0 s
vio_100hz                              wall - stamp = +0.054 s
```

原始 flag 状态备份在 `/userdata/fixbak/`（本机是「文件不存在」，所以留的是
`is_time_sync.absent_marker`）。

### 2.3 🔴 flag 必须**跳变** 0→1，光是「值为 1」没用

这个坑是 2026-08-04 板子重启后实际踩到的，很隐蔽：

`computeRealtimeOffset()` **每次跳变只算一次**，结果缓存在 `threads_.time_offset_ns` 里；
`timeSyncMonitorThread()` 只在看到 flag 从「未设」变成「已设」时才重算。而
**flag 文件在 `/etc/init.d/looper/setting/` 下，重启后仍然存在，但时钟不会存活**。

于是重启后的时序是：flag 已经是 1 → 启动时就用**错误的时钟**（2025-08-26）算好偏移 →
我事后把时钟改对，但监控线程看到 flag 一直是 1，`was_synced` 保持 true，**永远不重算**。
现象是 PC 侧看到相机时间戳差 **29600733 秒（343 天）**，尽管板子 `date` 已经完全正确。

所以 `sync_board_time.sh` 的做法是：对完时钟后**先写 0、等 3 秒让监控线程观察到、再写 1**。

⚠️ **自测时不要用「板上两个话题互相配对」来验证** —— 它们共享同一个时钟，对错都能配上，
这个测试对该 bug 完全不敏感。要拿一个**板上话题**和一个**本机自己打戳的话题**去配。

### 2.4 USB 链路本身有约 240 ms 延迟

时钟对齐后（板内 `date` 与 PC 差 2 ms），PC 侧订阅板上话题实测：

| 话题 | PC 收到时刻 − header.stamp |
|---|---|
| `imu` | +0.215 s |
| `infra1/image_rect_raw` | +0.240 s |
| `depth/image_rect_raw` | +0.383 s |

这**不是时钟误差，是真实的传输 + 序列化延迟**（544×640 mono8 @20 Hz ≈ 7 MB/s 过 USB gadget + DDS）。
后果：跨机 `message_filters` 在 50 ms 容差下配对 **0 次**。要么把容差放到 0.5 s 以上，
要么（更好）**把消费者放在板上跑**，不要让同步跨越 USB 链路。

### 2.5 走 WiFi 时「发」比「收」慢 4–6 倍，瓶颈在网卡驱动（2026-08-18 实测）

2026-08-18 起板子加装了一个 USB WiFi 网卡，可以走无线：`DEEP-RD` → `root@192.168.19.218`（**仍是同一台板子**，
MemTotal 与本文开头记录一致）。`usb0` 仍在 `169.254.10.1` 但没接 PC，`eth0` 是 `down`。
脚本一律用环境变量切目标，不用改代码：

```bash
BOARD=root@192.168.19.218 bash tool/x5_board/sync_to_board.sh
HOST=root@192.168.19.218 ./tool/x5_board/sync_board_time.sh
```

拓扑多了一层：**Looper —(串口 rx/tx)→ ESP32 —→ 差速车驱动板**，串口 `/dev/ttyS3` @ 115200，
被 `/root/car/teleop.py` 占用。注意这跟 LeKiwi 那套不同 —— 那边是 Feetech 舵机总线直挂 ttyS3（见 `servo_bus.md`）。

> ⚠️ 板上默认路由是 `0.0.0.0 via 169.254.10.2 dev usb0`（USB 网关，现在没接 PC）→ **板子上不了公网**。
> 要在板上装东西得先改路由，或者插回 USB。

#### 实测吞吐：严重不对称

| 方向 | TCP 4×5 s | UDP 全速灌包 |
|---|---|---|
| **板→PC（发）** | 1.33 / 1.90 / 1.97 / 1.77 Mbit/s | **2.5 Mbit/s 封顶，0% 丢包** |
| **PC→板（收）** | 9.99 / 7.07 / 4.13 / 7.92 Mbit/s | 14.5 Mbit/s |

#### 🔴 2026-08-20 复测：焊线修好了，但比 08-18 慢 2–5 倍，原因换人了

**背景**：网卡一度完全不通（`error -71` ×108，连设备描述符都读不出来）。**硬件同事查出是之前焊的线掉了**
—— 印证了当初「−71 出现在 100 mA 设备描述符阶段，驱动/射频/固件都还没参与 → 是电气层」的判断。
焊回去之后枚举正常（`lsusb`: `0bda:b711 Realtek RTL8188GU`，驱动 `rtl8710bu`），丢包从 20–90% 回到 **0%**。

**但吞吐没有回到 08-18 的水平**，两个方向都慢 2–5 倍：

| 方向 | 08-18 基线（裸 TCP） | **08-20 裸 TCP** | **08-20 走 SSH** | 退化 |
|---|---|---|---|---|
| **板→PC（发）** | 1.33–1.97 Mbit/s | **0.874** | 0.76 | 1.5–2.3× |
| **PC→板（收）** | 4.13–9.99 Mbit/s | **2.043** | 1.67 | 2–5× |

⚠️ **对照必须同口径**：08-18 基线是裸 TCP（iperf3），所以先用裸 TCP 复测才有意义。
测法：`python3 tput.py sink <port>` / `source <host> <port> <MB>`，**取接收端的计时** ——
发送端 `sendall` 只写进 socket 缓冲就返回，实测两端差 7 s（12.33 vs 19.20 s）。

**排除掉的三个嫌疑：**

| 嫌疑 | 判据 | 结论 |
|---|---|---|
| SSH 加密 | 裸 TCP 0.874 vs SSH 0.76（发）、2.043 vs 1.67（收） | ❌ 只占 **13–18%**，不是瓶颈 |
| CPU | 同一份 socket 代码走 loopback 是 648 Mbit/s（08-18 实测） | ❌ 差 700 倍，CPU 完全没参与 |
| USB 总线 | `/sys/bus/usb/devices/1-1/speed` = **480**（High-Speed） | ❌ 2 Mbit/s 的流量对 480 无意义 |

⚠️ 顺带一个发现：**这颗 A55 没有 ARM 加密扩展** —— `/proc/cpuinfo` 的 Features 里
有 `crc32` 但**没有 `aes` / `sha1` / `sha2` / `pmull`**，AES/SHA 全走软件。
本来担心它是 SSH 的瓶颈，上面的裸 TCP 对照证明不是（这个链路太慢，加密还没成为约束）。

**真正退化的是射频链路，两个可测的因素：**

| 指标 | 08-18 | **08-20** | 含义 |
|---|---|---|---|
| `signal` | −48 dBm | **−52 ~ −54 dBm**（10 次采样稳定） | 🔴 **差 6 dB = 功率差 4 倍** |
| 活的 `rx_rate` | MCS7（72.2 Mbit/s PHY） | **MCS4（39 Mbit/s PHY）** | 🔴 调制降档，PHY 容量少 1.85× |
| `Total False Alarm` | 未记录 | **739**（cck 299 + ofdm 440） | 🟡 2.4 GHz 噪声 |
| `current_igi` | 未记录 | **0x3d** | 🟡 驱动主动降低灵敏度来对抗噪声 |
| TCP `RetransSegs` | — | 37 / 5012 OutSegs = **0.74%** | 🟡 有但不是主因 |
| `rx_dropped` / errors | — | 5224 / **0 errors** | 🟡 |

🔑 **6 dB 是最大的单项,而且它是稳定的不是波动的** —— 稳定的固定损耗指向 RF 通路，
不是干扰。**信号降档 MCS7→MCS4 单独就解释 1.85×**，叠加噪声占用的空口时间（739 false alarm、
IGI 拉到 0x3d）够解释 2–5×。

⚠️ **`signal` 是我们收 AP 的 RSSI，和供电无关** —— 所以「焊线接触电阻导致功放掉压」解释不了它。
两个更可能的原因，按可能性排：**①板子被搬动了位置/朝向**（它去了同事那边一趟）；
②如果焊的那根是天线相关的线，接头损耗会直接体现为 6 dB。
**两者都靠「挪一下位置重测」区分，零成本。**

🔑 **另一个重要区别：08-18 时「发」是被驱动封顶的（TCP 1.7 / UDP 2.5），现在「发」是被链路压住的（0.874）。**
所以修好 6 dB 最多回到约 1.7 Mbit/s —— **驱动的发送天花板还在上面等着**，这两件事要分开算。

#### 怎么办（按性价比）

| | 做法 | 收益 | 代价 |
|---|---|---|---|
| 1 | **先挪位置/朝向重测** | 可能直接拿回 2–5× | 零 |
| 2 | **app 预览改二进制帧 + 选 `infra1`/`depth` 不选 `color`** | base64 省 33%；`color` 要 3.59 Mbit/s（超），`infra1` q70 0.86、`depth` q50 0.26 | `ws.py` 改动已写好 |
| 3 | **大文件不走 WiFi** | 拉 2 GB 的 bag 走 WiFi 要 5 小时 | C 口 host 模式可插 U 盘 |
| 4 | **换双频 USB 网卡**（RTL8811AU / 8812AU / 8821CU / MT7921AU） | 🔑 **唯一的根治** —— 同时换掉发送瓶颈的驱动、避开 2.4 GHz 的噪声 | 要买 |
| 5 | ~~换信道~~ | ❌ 08-18 已验证只换信道没用，且 2.4 GHz 全段拥挤 | — |

⚠️ **方向决定可行性**：往板上推（代码、模型）走的是快方向，能用；**从板上拉大文件不行**。
我们接下来的活（推代码、拉日志）正好都在能用的一侧。

#### 根因：驱动的 USB 发送路径，参数调不动（2026-08-18 实测排除）

⚠️ **先纠正一个曾经写在这里的错误结论。** `sta_tp_info` 报 `tx_rate : CCK_1M(L)`，
持续发送时 40/40 个采样都是它，看起来像"发送速率被钉死在 1 Mbit/s"。**这个读数不可信** ——
CCK_1M 的物理速率就是 1 Mbit/s，而实测板子发出 **2.5–2.6 Mbit/s 的净载荷**，
**物理上穿不过一条 1 Mbit/s 的链路**。同一个文件里的 `TP {Tx,Rx,Total}` 在满速传输时恒为 `{0,0,0}`，
同样是没实现的字段。**这个驱动的 proc 输出只有 `rx_rate` 是活的**（它会在 MCS4↔MCS7 之间跳）。

同理，`iw dev ... link` 报的 `tx bitrate: 72.2 MBit/s` 也是标称值，不是实测值。
**这个驱动没有任何一个可信的发送速率读数。**

真正站得住的是这几条：

1. **对照实验**：同一份灌包代码在板上走 loopback 是 **57 890 包/秒（648 Mbit/s）**，
   走 WiFi 只有 **225 包/秒（2.5 Mbit/s）**，差 **257 倍** → Python、CPU、socket API 全部排除
2. **UDP 0% 丢包**：紧循环不限速也一个包不丢 → 包不是丢在空中，是 `sendto()` 每次阻塞约 4.4 ms
   发不出去。真的信道拥挤会表现为重传和丢包
3. **同一条信道、同一块芯片，「收」14.5 Mbit/s 而「发」只有 2.5** → 空口和芯片都有余量
4. **驱动有 `rtw_usb_rxagg_mode=2`（USB 接收聚合），却没有任何发送聚合参数** ——
   92 个模块参数里没有一个 tx agg 相关的。这跟收发差 6 倍的现象是自洽的

#### 试过的都无效（别再重复）

| 手段 | 结果 |
|---|---|
| 运行时写 `rtw_lowrate_two_xmit` 1→0 | 2.50 Mbit/s，无变化（该参数只在加载时生效） |
| 重新关联（`wifi-connect.sh`） | 无变化。顺带观察到 `rtsen` 自己从 1 变 0，**所以 RTS/CTS 也不是原因** |
| 重载模块 `rtw_wifi_spec=1 rtw_lowrate_two_xmit=0` | **2.32 Mbit/s，没有改善** |
| 恢复原始参数后复测 | 2.22 / 2.61 / 2.59 Mbit/s |

**结论：主要的软件参数无效。** 92 个模块参数里只有 `rtw_tx_ampdu_amsdu`（现为 2=auto）和
`rtw_busy_thresh` 还没试，但已经试过的三个都毫无变化，继续试的期望值不高。剩下两个方向：

- 🔬 **未验证的物理假设，值得试**：发送比接收耗电大得多（功放要拉几百 mA 的突发电流）。
  如果 C 口 host 模式的 VBUS 供电偏弱，发送时掉压就会**只压垮发送、不影响接收** ——
  正好是实测的形态。**用一个外部供电的 USB hub 接网卡就能证伪**，成本最低
- 换一个**双频 USB 网卡**（RTL8811AU / 8812AU / 8821CU / MT7921AU），换掉这个驱动

> 📌 顺带查出 `wifi-connect.sh` 有个 bug：第 82 行用了板上不存在的 `ip`(1)（前面步骤都用 `busybox ip`），
> 所以它**每次都以 `ERROR: no IP assigned` 退出**，哪怕 DHCP 已经拿到地址。别被这条日志误导。

#### 后果与对策

**方向决定可行性**：往板上推东西（`sync_to_board.sh`、模型、代码）**能用**，1 GB 约 20 分钟；
**从板上拉 bag 不行**，1 GB 要 1.5 小时、2 GB 的过道 bag 要 3 小时。

> 🔴 **注意：`169.254.10.1` 那条 USB 直连现在是断的**，不能再当后路 —— 见 §2.6。
> 拉大文件目前只有两条路：插 U 盘（C 口已经是 host，网卡就是靠它供电的，U 盘能直接用），
> 或者临时把角色切回 device。

**app 预览走的正是慢的「发」方向。** 预览是 5 fps（`node_manager.py` 的 `_PREVIEW_MIN_INTERVAL = 0.2`），
用真实 Looper 帧实测编码后：

| 预览话题 | JPEG 字节 | base64 后 | 5 fps 需要 | 走 WiFi |
|---|---|---|---|---|
| `color`（压缩帧直接透传） | 67 320 | 89 760 | **3.59 Mbit/s** | ❌ 超了 |
| `infra1`（q70） | 16 128 | 21 504 | 0.86 Mbit/s | ✅ |
| `depth`（q50） | 4 966 | 6 621 | 0.26 Mbit/s | ✅ |

**要看画面就选 `infra1` 或 `depth`，别选 `color`。**
另外 `ws.py` 的预览是 `send_text(base64.b64encode(...))`，**改成二进制帧直接省 33%** ——
链路这么窄时这是性价比最高的一处改动。

真要根治就**换一个双频 USB 网卡**（RTL8811AU / 8812AU / 8821CU / MT7921AU 之类），
既上 5 GHz 也换掉这个驱动。只换 2.4 GHz 信道没用。

---

### 2.6 C 口的 USB 角色：曾被切成 host，2026-08-20 起已固定回 device （USB 直连恢复）

> 🟢 **现状（2026-08-20）：已把 `system_init.sh:47` 那行注释掉，C 口开机就是 device 模式，
> USB 直连 `169.254.10.1` 常在（实测 0% 丢包 / 1.9 ms），多次重启验证过。**
>
> ```sh
> cp /etc/init.d/looper/system_init.sh /etc/init.d/looper/system_init.sh.bak
> sed -i 's|^echo host > /sys/class/usb_role|#echo host > /sys/class/usb_role|' \
>     /etc/init.d/looper/system_init.sh
> ```
>
> 只想临时切换不改文件：`echo device > /sys/class/usb_role/35100000.usb-role-switch/role`
> （纯运行时，重启后由开机脚本恢复）。
>
> **这是二选一，不是两个都要**：host 模式是枚举任何 USB 设备的前提，注释掉之后
> USB WiFi 网卡不是「变慢」而是完全不存在。当前选 device 的理由是那块 USB WiFi 网卡
> 已经硬件损坏（`error -71` × 108，连设备描述符都读不出来；同网段其他设备 0% 丢包 / 12 ms，
> 对照组证明是板子侧），而且 USB 直连比 WiFi 快两个数量级。
>
> ⚠️ 确认过 `switch_check_init.sh`（`system_init.sh:23`）只按物理拨杆改 `usb-gadget.sh` 里的
> `USE_UVC` / `USE_HID` / `USE_NCM`，**不碰第 47 行**，所以这个改动会稳定生效。
>
> 下面是当初切成 host 时的机制记录，保留备查。


装 WiFi 网卡时，`/etc/init.d/looper/system_init.sh:47` 加了一行：

```sh
echo host > /sys/class/usb_role/35100000.usb-role-switch/role
```

于是两个 USB 控制器的角色对调了（对比 [`usb_host_mode.md`](usb_host_mode.md) 记录的原始状态）：

| 控制器 | 速率 | 原来 | 现在 |
|---|---|---|---|
| `35100000.usb` | super-speed | device（`usb0` = 169.254.10.1，SSH 唯一通路） | **host** ← WiFi 网卡插在这 |
| `35300000.usb` | high-speed | 空闲 | **device**，是当前唯一可用的 UDC |

**但 gadget 没跟着搬家** —— `/sys/kernel/config/usb_gadget/g_comp_usb3.0/UDC` 里仍然写着
`35100000.usb`，而那个口已经是 host 了。所以 `usb0` 这个网卡还在、IP 还在，**但没有载波**，
插 PC 也不会通。这就是"走不了有线"的确切机制。

#### 三点判断

1. **不是不可逆的改动**。那是一次运行时 sysfs 写，没动设备树、没动固件。
   `echo device > /sys/class/usb_role/35100000.usb-role-switch/role` 就能切回去 ——
   代价是 WiFi 网卡立刻掉线。**现在没有有线兜底，切之前必须先安排好退路**（比如挂个定时 `reboot`，
   开机脚本会把一切恢复原样）。
2. **理论上可以两个都要**：`35300000.usb` 还在 device 模式且空闲，把 gadget 重新绑到它上面
   （往 `UDC` 写 `35300000.usb`）就能在保留 WiFi 的同时恢复 USB 网络。
   **前提是那个 USB2 口引到了壳外** —— 这正是 [`usb_host_mode.md § 3`](usb_host_mode.md) 留下的未解问题，
   得问硬件同事。
3. ✅ **顺带解决了 `usb_host_mode.md § 3.2` 的疑问：host 模式下有 VBUS。**
   那个 WiFi 网卡是总线供电的，它能工作就说明 5 V 出得来，**不需要外部供电 hub**。
   → 推论：**插 U 盘拷 bag 是可行的**，这是目前拉大文件最实际的办法。

---

### 2.7 ion 内存重分配：`MemTotal` 1307 → 1787 MiB（2026-08-20 完成，已持久化）

🟢 **已解决。** `ion_cma` 从 512 MiB 砍到 32 MiB，释放的 480 MiB 落回内核通用内存，
开机即生效，不需要任何人工干预。

| 区域 | 物理地址 | ion heap | 用途 | 容量 | 实测水位 |
|---|---|---|---|---|---|
| `ion_reserved_ge4g` | `0x0A4100000` | `cma_reserved` | 媒体 / VPF 缓冲 | 1024 MiB | **274 MiB** |
| `ion_carveout_ge4g` | `0x0E4100000` | `carveout` | **BPU 张量**（深度推理、VIO，我们的 SuperPoint 也在这） | 1024 MiB | **113 MiB** |
| `ion_cma_ge4g` | `0x124100000` | `ion_cma` | —— | **32 MiB**（原 512） | **恒为 0** |
| `adsp_ddr` | `0x09FE80000` | — | DSP | 34 MiB | — |
| **Linux `MemTotal`** | | | 其中 `linux,cma` 382 MiB | **1787 MiB**（原 1307） | |

设备树 `memory` 节点 `<0x84000000 0x7C000000>` + `<0x100000000 0x80000000>` = 4032 MiB。
`ion_*` 有 `_1g` / `_2g` / `_ge4g` 三套，生效的是 `_ge4g` —— 这套切法是「检测到 ≥4 GB 就按最阔绰的来」，
不是按实际需求算的，所以闲置极多。

**为什么砍 `ion_cma` 而不是另两个**：它整个会话零 client 零字节，而且在三个区**最顶上** ——
释放的 480 MiB 与内核原有空闲区连成连续块，不会在中间留洞（砍中间的 `carveout`，
若 U-Boot 不重算后续基址就会留空洞）。

#### 操作步骤

```bash
# 1) 板上（SSH 或串口都行）—— 让 U-Boot 主动停下
systemctl reboot uart
```

U-Boot 打印 `boot action: UART` 后停在 `Hobot>` 提示符。**这一步必须有串口**
（`console=ttyS0,921600n8`）：那一刻 Linux 还没起，`usb0` 是 Linux 用户态
`system_init.sh:26` 才创建的，所以没有 SSH、没有 USB 网络。

```
# 2) 串口上
Hobot> setenv ion_cma_size 0x2000000
Hobot> saveenv                        # → Saving Environment to MMC... Writing to MMC(0)... OK
Hobot> setenv bootdelay -2            # ⚠️ 必须，见下
Hobot> saveenv
Hobot> boot
```

三个变量名：`ion_reserved_size` / `ion_carveout_size` / `ion_cma_size`。
U-Boot 每次启动都会打印实际生效值，可以对账：

```
Set Mem[ion_reserved_ge4g] Size to 0x0000000040000000@0xa4100000
Set Mem[ion_carveout_ge4g] Size to 0x0000000040000000@0xe4100000
Set Mem[ion_cma_ge4g]      Size to 0x0000000002000000@0x124100000
```

#### 🔴 陷阱一：送键打不断倒计时，`systemctl reboot uart` 是唯一入口

默认环境是 `bootdelay=-2`，U-Boot 的 `abortboot()` 遇到负值**直接跳过整个按键检测循环** ——
连 `Hit any key to stop autoboot` 那行都不打印。所以「重启时猛敲键盘进 U-Boot」在这块板子上
永远不会成功，没有提示符可进。

#### 🔴 陷阱二：`saveenv` 会把 `bootdelay` 写成 `-1`，之后每次开机卡在提示符

`boot action: UART` 进入时 U-Boot 把 `bootdelay` 设为 **−1**（语义是「完全不自动启动」），
`saveenv` 会把这个 −1 一起持久化 → **之后每次开机都停在 `Hobot>`，Linux 不启动、USB 和网络全无**。
改回 `-2` 即可。

#### 🟢 `saveenv` 只动 `ubootenv`，不碰引导程序

改前后 md5 对账：

| 分区 | 结果 |
|---|---|
| p2 `miniboot` | 未变 |
| p3 `miniboot_bak1` | 未变（顺带发现：与 p2 逐字节相同，是完整镜像） |
| p7 `ubootenv` | 变了 —— 正是该写的地方 |

#### 🔑 环境区的确切位置：以后改 ion 不再需要串口

从 `saveenv` 写完的分区反解（增量 CRC 逐字节扫描）：

**p7 `ubootenv` 偏移 `0`，`CONFIG_ENV_SIZE = 0x30000`（192 KiB），布局 `crc32(4 字节小端) + 数据`，
非冗余（无 flags 字节）。**

192 KiB 不是 2 的幂次，这就是穷举 4K/8K/16K/32K/64K/128K/256K 全部落空的原因。
已用这个信息**从 Linux 成功改过环境变量**：读 p7 → 校验 CRC → 改值 → 重算 CRC → `dd` 回去 → 重启生效。
所以后续调 ion 尺寸走 Linux 就行。

判据：环境加载成功打印 `Loading Environment from MMC... (Tuning Ok!) OK`，
失败是 `*** Warning - bad CRC, using default environment`（然后用默认值正常启动，属于安全的失败）。

#### 不需要串口也能读 U-Boot 输出

`/userdata/log/uboot/archive/X5_Uboot-NNNN-*.Log`，100 份环形归档。

⚠️ **时间戳全是 `2025_08_26`**（板子无 RTC，日志由启动早期写入，此时还没对时），
所以 `ls -t` 挑不出最新的一份 —— **按文件名里的编号排序，编号最大的才是最新**。

#### 作废的旧结论

| 旧说法 | 为什么作废 |
|---|---|
| 「改 `/app/prebuilts/dtb/x5-evb-lp4-demi.dtb`」 | U-Boot 启动时会覆盖静态文件里的值。判据：文件里三个 `ion_*_ge4g` 是 `status=disabled`，活动设备树里是 `okay` |
| 「重打包 FIT 写 `boot_b`」（原路线 B） | 不必要 —— 环境变量这条路已走通。A/B 槽信息保留在 § 2.9 供刷机时参考 |
| 「现在不要做，没有有线兜底」 | 已过时。C 口现在固定 device 模式且已持久化（见 § 2.6），USB 直连 `169.254.10.1` 常在 |
| 「预期 `MemTotal` 约 1819 MiB」 | 实际 1787 MiB —— 因为留了 32 MiB 而不是设 0，释放 480 而非 512 |
| 「只写一个变量 = 真砖」 | **这个担心是对的**，规避办法是让 U-Boot 自己 `saveenv`（它导出完整运行时环境，61 个默认变量都在），而不是手工拼环境镜像 |

另一条踩过的死路：**别试图靠猜偏移从 Linux 写环境**。板上没有 `fw_setenv` / `fw_printenv`，
也没有 `/etc/fw_env.config`；按分区名定位算出的偏移（分区末尾 − ENV_SIZE）是错的；
U-Boot 源码里的回退偏移 `CONFIG_ENV_OFFSET = 0x2F8000` 换算成扇区 6080，
**落在 `miniboot_bak1`（p3）里面**，往那写要覆盖备份引导程序。
正确顺序是：先用一次串口让 `saveenv` 建立合法环境，再反解位置，之后就自由了。

#### 验证：相机完全没受影响

| 验证项 | 结果 |
|---|---|
| 三个传感器 | `sc132gs flow0` / `sc132gs flow1` / `imx415 flow2` 全部 `start done` |
| 故障特征 `hsize count:0x0` | 一条都没有 |
| 丢帧告警（`Image data is lost` 等） | 0 次 |
| 固件初始化 | 编码器 / IMU / VIO 状态估计器全部 `Init success` |
| ion 实测水位 | `carveout` 113.2 / `cma_reserved` 274.1 MiB —— 与改动前**完全一致** |
| `looper_bridge_node` CPU | 84.5% —— 有帧进来才会有负载，数据流确认 |
| ion 分配失败 | 0（dmesg 里的 swiotlb / 模块签名 / eth0 无 phy 都是旧有告警） |

#### 想要更多内存的话

另两个池还有 1024 − 274 = 750 MiB 和 1024 − 113 = 911 MiB 空着，但**现在不能砍**：
那两个水位是在「彩色关（`TINYNAV_DISABLE_COLOR=1`）、3D 网格关（`enable_scene_mesh: false`）、
深度 5 Hz（`depth_frame_skip=4`）、我们的 SuperPoint 没跑 BPU」下测的，**真实峰值从未测过**。
而且我们的 SP 是往 `carveout` 里加的（编译器报告 24.3 MB DDR/帧 + 1.64 MB 模型 + 约 20 MB BPU 基座）。

要砍先在最重工况复测峰值。余量原则：池子留太大只是浪费，切太小会让相机固件申请缓冲失败、
相机起不来 —— 失败代价严重不对称，所以余量宁可留 3 倍以上。

---

### 2.8 板上自动对时（2026-08-18 起，不用再从 PC 跑脚本）

有了 WiFi 之后板子能自己 NTP 对时，`tool/x5_board/board_autotime.sh` +
`board-autotime.service` 已装好并验证：开机后自动把时钟从 2025-08-26 拉到真实时间，
并让 `is_time_sync` 跳变 0→1，相机时间戳实测偏差 **−0.01 s**。

**它是在 `insight_full` 已经在跑之后才改时钟的，这没问题。** 实测两次开机都是同一顺序
（固件先起，约 27 s 后时钟前跳约 3.08e7 秒），相机健康的那一次时间戳照样落在正确的 epoch：
`stamp` 相对本机时钟 `infra` +0.066 s / `depth` +0.152 s，首帧 `stamp` = 当天真实时间。
原因是厂商自己的 `/etc/init.d/looper/time_sync.sh` 就是这个设计 —— 它也在运行期改时钟、
靠 `is_time_sync` 的跳变通知固件重新对齐基准，固件本来就要处理这件事。

（那个脚本用的时间源是相机自己 AP 的网关；实测网关比公网 NTP 慢 822 秒，所以
`board_autotime.sh` 故意改用公网 NTP。）

装法（已在这台板子上做完）：

```sh
cp board_autotime.sh /userdata/x5/ && cp board-autotime.service /etc/systemd/system/
systemctl daemon-reload && systemctl enable --now board-autotime
# 日志：/userdata/x5/logs/autotime.log
```

脚本要处理三件事，缺一件都不成：

1. **等网卡拿到 IP** —— `wifi-connect.sh` 是 `system_init.sh` 里同步跑的，但 AP 关联本身有
   延迟；脚本轮询最多 60 s
2. **修默认路由** —— DHCP 装不上，因为死掉的 `usb0` 那条 `0.0.0.0 via 169.254.10.2` 一直占着
   metric 0（见 §2.6）。不修则 `ntpdate` 一定超时
3. **让 `is_time_sync` 跳变 0→1** —— 见 §2.3，光把时钟改对没用

> ⚠️ **别拿网关当时间源。** 实测 `192.168.19.254` 自己比公网 NTP **慢 822 秒**，
> 用它对时会引入 14 分钟误差。脚本里只用 `ntp.aliyun.com` / `cn.pool.ntp.org`。

> ⚠️ 服务装在 `/etc/systemd/system/`，**OTA 可能覆盖**，升级固件后要复查
> `systemctl is-enabled board-autotime`。

---

### 2.9 分区表与 A/B 双槽（刷机时参考）

| 分区 | 设备 | 起始扇区 | 大小 | 说明 |
|---|---|---|---|---|
| `mbr` | p1 | 40 | 4 KiB | |
| `miniboot` / `miniboot_bak1` | p2 / p3 | 48 / 4656 | 各 2.25 MiB | **实测两者逐字节相同**，p3 是完整镜像 |
| `misc` | p4 | — | 4 KiB | 槽元数据，偏移 `0x800` 起 magic `BCAB` |
| `uboot_a` / `uboot_b` | p5 / p6 | 9272 / 13368 | 各 2 MiB | |
| **`ubootenv`** | **p7** | **17464** | **256 KiB** | 环境变量，实际只用偏移 0 起的 192 KiB（见 § 2.7） |
| `vbmeta` | p8 | 17976 | 16 KiB | AVB 校验数据 |
| `boot_a` / `boot_b` | p9 / p10 | 18008 / 83544 | 各 32 MiB | FIT 镜像（offset 0 就是 `d00dfeed`，实测 10.9 MiB） |
| `system_a` / `system_b` | p11 / p12 | 149080 / 4343384 | 各 2048 MiB | |
| `hbre_a` / `hbre_b` | p13 / p14 | — | 各 200 MiB | |
| `app` | p15 | 9356888 | 700 MiB | |
| `private` | p16 | 10790488 | 256 KiB | |
| `userdata` | p17 | 10791000 | 53.1 GiB | |

`/proc/cmdline` 里 `hobotboot.slot_suffix=_a`，当前跑 A 槽。`misc` 偏移 0x800 实测：

```
61 00 00 00 42 43 41 42 01 02 00 00 9f 00 9e 00
            B  C  A  B  ver nb   A槽  B槽
```

按 Android `slot_metadata` 位域拆（低 4 位 priority、接着 3 位 tries_remaining、
最高位 successful_boot）：**A 槽 priority 15 / B 槽 14，两者都已标记启动成功**。
U-Boot 里有 `ANDROID: Attempting slot %c, tries remaining %d` 和
`if the returned slot runs out of boot attempts` —— **给 B 槽写坏镜像会在重试用尽后自动回退到 A 槽**。

⚠️ 板上的 `bootctl` 是 systemd 的 EFI 工具，跟这套 A/B 无关，别用错。
FIT 打包源文件是现成的：`/app/prebuilts/dtb/x5.its`，本板走 `boardid-0x0731` → `fdt2` →
`x5-evb-lp4-demi.dtb`。U-Boot 里有 `avb verify failed with status %d`，
重打包的 `boot_b` 过不了 AVB 就起不来，但这个失败会被 A/B 重试兜住。

U-Boot 里还有完整的 fastboot（`Android Fastboot`、`boot action: FASTBOOT USB2.0/USB3.0`、
`do_fastboot_usb`）和 recovery（`boot action: entry recovery mode`），
BCB 命令字符串是 `reboot-fastboot` / `reboot-recovery`，`oem` 只支持 `set_medium`。

---


### 2.10 🔴 换板子：64GB 那台已交出，所有板上改动都要重做

> **2026-08-20 起手上只有最早调试的两台标准版 Looper**（15.7 GB eMMC）。
> **08-03 之后的全部工作都是在 64GB 改装版上做的**（见 [`x5.md § 2.1`](x5.md) 台账），
> 那台机器已经交给别人 —— 下面每一项都是**板子本地状态，不会跟着代码走**。

认设备：`cat /sys/class/socinfo/soc_uid`（`/etc/machine-id` 所有机器相同，认不出来）。
标准版 `cat /sys/block/mmcblk0/size` ×512 = **15.7 GB**，`/userdata` **9.2 G**。

#### 要重做的（都有本地备份，不用重新解决）

| 项 | 新板子的状态 | 怎么恢复 | 耗时 |
|---|---|---|---|
| **ion 内存扩容** | ❌ 回到 `MemTotal` 1307 MiB | 环境区位置已知（p7 偏移 0，`0x30000`），**从 Linux 直接改，不用串口**，见 § 2.7 | ~5 min |
| **C 口 device 模式** | ⚠️ 先查 `system_init.sh:47` 在不在（标准版固件可能没这行） | 有就注释掉，见 § 2.6 | ~2 min |
| **自定义固件**（`depth_frame_skip` + `vio_enabled`） | ❌ 是原版 | `LooperHub/tros_ws/install/lib/libinsight_full_plugin.so` **已编好在本地**（2026-08-11，md5 `cb732ae8…`，两个开关的字符串都在），推过去 + 改 `user_params.json` | ~10 min |
| **代码部署** `/userdata/x5/` | ❌ 空 | `BOARD=root@<ip> bash tool/x5_board/sync_to_board.sh` | ~10 min |
| **Python 依赖**（scipy/numba） | ❌ 无 | `/home/dm/looper/x5_wheels/`（145 M，官方 aarch64 wheel） | ~5 min |
| **对时** | ❌ 每次上电回 2025-08-26 | `tool/x5_board/sync_board_time.sh` | ~1 min |
| **BPU 模型** | ❌ 无 | `/home/dm/looper/x5_work/bpu/hbout/sp_backbone.bin` | ~1 min |

#### 不用做的

- 🟢 **1-lane 修复不需要** —— 那是 64GB 改装版独有的旧 sensor 库残留（`libsc132gs.so` 是 2026-04 的
  OTA 残留）。标准版三台相机固件本来就正常，`stereo_sensor_name` 保持 `-2lane`。
- 🟢 **`ros2` CLI 在标准版上是好的** —— 64GB 那台报 `PackageNotFoundError: ros2cli`，
  标准版 source `/etc/init.d/looper/setting/ros2_env.conf` 后可用。

#### 🔴 存储：这是真正的新约束，不是重做一遍就行

| | 标准版 | 64GB 版 |
|---|---|---|
| `/userdata` 总量 | **9.2 G** | 53 G |
| 实测剩余 | **2.3–2.6 G** | 44 G |

**一个完整地图目录 3.2 G**（`x5_work/maps2/map_day` 实测），**装不进 2.3–2.6 G**：

| 文件 | 大小 | 导航需要吗 |
|---|---|---|
| `depths.db` | **1.6 G** | 🟡 PnP 要 3D 点，但存的是整幅深度图 —— 换成稀疏 landmark 可压到几 MB（需改代码） |
| `patch_tokens.db` | **881 M** | 🟢 **不需要** —— DINOv2 patch token 是旧方案（方案 6）的，SP+VLAD 用不上 |
| `features.db` | 595 M | ✅ 要（关键点 + 描述子） |
| `vlad_descriptors.db` | 114 M | ✅ 要 |
| `rgb_images_db` / `infra1_images_db` | 86 M / 19 M | 🟡 只在可视化/调试用 |
| `sdf_map.npy` + `occupancy_grid.npy` + `poses.npy` | 10 M | ✅ 要 |

#### 🟢 但按 23 cm 密度建图，双时段也装得下（2026-08-20 算清）

**地图大小严格线性：每关键帧 2.95 MB**（三个地图实测 2.95 / 2.95 / 2.96，1161 / 1120 / 1163 帧）。
现有地图是**全密度** 1161 帧才 3.43 G —— 而 23 cm 密度只要 **280 帧**：

| 配置 | 每帧 | 一个地图 | **双时段** | 装得下？ |
|---|---|---|---|---|
| 全密度 1161 帧 | 2.95 MB | 3.43 G | 6.9 G | 🔴 一个都放不下 |
| 全密度，去 `patch_tokens` | 2.20 MB | 2.55 G | 5.1 G | 🔴 |
| **23 cm / 280 帧** | 2.95 MB | 826 M | 1.65 G | 🟡 紧但可以 |
| ⭐ **23 cm + 去 `patch_tokens`** | **2.20 MB** | **616 M** | **1.23 G** | 🟢 **余约 1.1 G** |

🔑 **而 23 cm 恰好也是精度最高的配置**（INT8 + 23 cm：70.0% R@3 / 75.5% R@10，
比全量 1161 帧的 68.4% 还高）—— 瘦身和精度这次不冲突。
`patch_tokens.db` 是 DINOv2 旧方案（方案 6）的，SP+VLAD 用不上，本来就该不存。

⚠️ **瘦身是 PC 侧的事**（按 23 cm 重建，或裁剪现有 DB），板子只收结果。

还能再挖：`depths.db` 占 1.38 MB/帧（整幅深度图），换成稀疏 landmark 可压到几 MB，
但**在 23 cm 前提下已经不是必须的了**，优先级降低。
外挂 U 盘（C 口切 host 有 VBUS）和 USB WiFi / USB 直连互斥，最后手段。

---


## 3. 相机话题的 QoS 和消息类型不统一

订阅时用错了收不到消息，而且**只有 `RELIABILITY` 不兼容时才会打警告**，类型用错则完全没有提示。

| 话题 | 类型 | Reliability |
|---|---|---|
| `depth/image_rect_raw` | `sensor_msgs/Image`（mono16, mm） | RELIABLE |
| `infra1,2/image_rect_raw` | `sensor_msgs/Image`（mono8, 544×640） | RELIABLE |
| `color/image_rect_raw/compressed` | `sensor_msgs/CompressedImage` | RELIABLE |
| `imu` | `sensor_msgs/Imu` | **BEST_EFFORT** ⚠️ 用默认 QoS 订阅收不到 |
| `vio_100hz` | **`geometry_msgs/PoseStamped`** ⚠️ 不是 `Odometry` | RELIABLE |

`vio_100hz` 是 `PoseStamped` 而 `map_node.py:461` 订阅的是 `Odometry, '/slam/odometry'`，
所以中间必须有转换（`x5_work/vio_relay.py` 干的就是这件事）。

---

## 4. 依赖：全部有官方 aarch64 wheel，没有一个需要板上编译

板上**已有**：`rclpy`、`cv_bridge`、`message_filters`、`tf2_ros`、`tf2_py`、全部 msg、
`rosbag2_py`、`rosidl_runtime_py`、`yaml`、`onnxruntime 1.18.0`、`pyserial`。

装上去的（`/userdata/x5/wheels/`，119 MB，2m12s 装完）：

| 包 | 版本 | 备注 |
|---|---|---|
| numpy | 1.26.1 | 从板上原有的 1.21.5 升级 |
| scipy | 1.15.3 | |
| llvmlite / numba | 0.44.0 / 0.61.2 | ⚠️ 见下方 manylinux 坑 |
| opencv-python-**headless** | 4.11.0.86 | ⚠️ 见下方 GTK 坑 |
| codetiming / einops / fufpy / tqdm | | 纯 Python |
| async-lru | 2.3.0 | ⚠️ 连带要升 `typing_extensions` |
| typing_extensions | 4.16.0 | 板上原有 3.10.0.2 太老，没有 `Self` |
| av | 17.1.0 | decord 替代品要用（§5） |

### 四个坑

**① numba/llvmlite 的 wheel 是 `manylinux_2_28`，不是 `manylinux2014`。**
只给 `--platform manylinux2014_aarch64` 时 pip 报的是

```
ERROR: Could not find a version that satisfies the requirement numba==0.61.2 (from versions: ... 0.60.0)
```

看着像 0.61.2 这个版本不存在，实际只是 0.61 系列把 glibc 门槛从 2.17 提到了 2.28。
板上 glibc 2.35，完全没问题。下载时要同时给
`--platform manylinux_2_28_aarch64 --platform manylinux_2_27_aarch64`。

**② 必须用 `opencv-python-headless`，因为板上完全没有 GTK。**
`ls /usr/lib/aarch64-linux-gnu/libgtk*` 什么都没有。板上系统自带的那个 cv2 是坏的 ——
注意它的报错会随 `LD_LIBRARY_PATH` 变化：修好库路径之前报 `libopencv_hdf.so.4.5d`，
修好之后才暴露真正的拦路虎 `libgtk-3.so.0`。**别被第一层报错带偏。**
`headless` wheel 的 `readelf -d` 里对 libgtk / libX11 / libGL 的引用数为 0，
而且仓库里没有任何 `cv2.imshow` / `namedWindow` / `waitKey`，所以无功能损失。
装的时候把坏的 `cv2.cpython-310-aarch64-linux-gnu.so` 显式改名备份，
**不要依赖「同目录下包优先于扩展模块」这种隐式 import 顺序**。

**③ `tinynav_cpp_bind.so` 必须放在 `tinynav` 包内部。**
`map_node.py:21` 写的是 `from tinynav.tinynav_cpp_bind import pose_graph_solve`，
所以光把它放到 `sys.path` 上不够，会报 `ModuleNotFoundError: No module named 'tinynav.tinynav_cpp_bind'`。
要 `cp` 进 `/userdata/x5/tinynav/tinynav/`，并把 `/userdata/x5/cpp_bind`（库是平铺的，
没有 `lib/` 子目录）加进 `LD_LIBRARY_PATH`。

**④ 升 numpy 会不会搞坏那些按 numpy 1.21 头文件编的二进制模块？实测不会。**
这是整个第 1 步里我最担心的一项，因为失败特征（`ndarray size changed`、
`_ARRAY_API not found`）只在**真正跑起来**时才出现，光 import 看不出来。
所以 `abi_check.py` 里每一项都做真实计算，8/8 通过：

| 检查 | 结果 |
|---|---|
| `cv_bridge` mono16 + mono8 往返 | 逐字节一致 |
| `rclpy` 建节点 + 构造 `PointCloud2` | OK |
| `onnxruntime 1.18.0` numpy 透传 | OK |
| `tf2_ros` + `tf2_py` | OK |
| `pydbow3` uint8 建库 / float64 被拒 | OK（dtype dispatch 生效） |
| `tinynav_cpp_bind` 三个符号 | OK |

> 🔴 **绝对不要用「升到 numpy 2」来解决问题** —— numpy 1.x 内部保证「旧头文件编、新运行时跑」
> 这个方向兼容，跨大版本不保证，会真的搞坏 `cv_bridge` 和 `onnxruntime`。

---

## 5. `decord` 在 aarch64 上不存在，用 PyAV 顶掉

`decord` 和 `eva-decord` **都没有 aarch64 wheel，也没有 sdist**，从源码编要把整套
ffmpeg + CMake 搬上板。而 `tool/video_db.py` 在模块顶层 import 它，
`TinyNavDB.__init__`（`build_map_node.py:453`）又**无条件**构造 `VideoDB` ——
所以**只要打开任何一张地图就会炸**（`TypeError: 'NoneType' object is not callable`，
因为 `build_map_node.py:35-37` 的 try/except 已经把 `VideoDB` 置成 `None` 了）。

而 tinynav 用到的 `decord` API 只有两个操作：`len(reader)` 和
`reader[i].asnumpy()` 返回 RGB 帧。PyAV 有 aarch64 wheel，能干这两件事。
所以 [`tool/x5_board/decord_shim/decord.py`](../../tool/x5_board/decord_shim/decord.py)
是个**真能用的替代实现，不是抛异常的假桩**：关键帧 seek + 前向解码 + 单帧缓存。

验证方式是在 PC 上跟**真 decord 0.6.0 逐帧比对**（顺序读、前跳、回跳、重复读、首帧、末帧
共 12 个位置），`max|diff| = 0` 全部逐字节一致。板上实测 1123 帧地图视频可正常随机访问。

> 写这个 shim 时踩了一个自己造的坑，记一下：`_seek_to()` 为了知道 seek 落到哪一帧，
> 必须先解一帧读它的 pts，这帧被消费掉并放进缓存了；但 `_decode_at()` seek 完之后
> 没检查缓存就继续往后找，于是找 index 0 时从 index 1 开始扫，扫完 1123 帧报
> `ran out of frames`。**seek 消费掉的那一帧可能就是你要的那一帧。**

只在板上把它放到 PYTHONPATH 里。装了真 decord 的机器不要用 —— 真 decord 快得多。

---

## 6. 实测结果

地图 `map_gt`（1123 关键帧），词典 `voc_office_k10L5`，每 37 帧取 1 个查询，28 个计时查询
（另有 2 个 warmup 被丢弃）。判定：`relocalize` 返回 True **且** XY 误差 ≤ 0.5 m **且**
旋转测地误差 ≤ 10°。

| | gt→gt 自一致 | **gt→day 跨时段** | PC 上的 gt→day（参考） |
|---|---|---|---|
| 成功率 | 27/28 = **96.4 %** | 27/28 = **96.4 %** | 97.3 %（1100+ 查询） |
| wall p50 | 457 ms | **374 ms** | 41 ms |
| wall p90 | 516 ms | **420 ms** | — |
| wall max | 1001 ms | 449 ms | — |
| 峰值 RSS | 429 MB | **407 MB** | 1735 MB（大词典）/ 663 MB（小词典） |
| 启动 `node_build_s` | 66.1 s（numba 冷） | **34.2 s（numba 热）** | 2.7 s |

**X5 比 PC 慢约 9 倍**，和 PC 上按核数缩放外推的 0.2–1.1 s 吻合。
**5 秒预算有约 13 倍余量 —— 时间明确不是瓶颈。**

### 冷启动优化：板上实测（2026-08-04）

`map_day`（1161 关键帧）+ `voc_office_k10L5`，numba 与页缓存均已预热，同一份
`profile_startup.py`，只换 `core/` 三个文件：

| | 优化前 | 优化后 | |
|---|---|---|---|
| `MapNode.__init__` | 34795 / 34981 ms | **21205 / 20068 / 19647 ms** | **−41.8%** |
| ├ `LoopClosure(1161)`（DBoW3 重建） | 30131 / 30418 ms | 17849–19258 ms | −38.6% |
| ├ `LoopClosure(0)` | 474 / 475 ms | 76–94 ms | −83% |
| └ `_warmup_nav_path_search`（numba JIT） | 2708 / 2785 ms | 移出构造函数 | −100%（关键路径上） |
| RSS | 363 MiB | **304 MiB** | −16% |
| 峰值 RSS | 363 MiB | 310 MiB | |

重复性：优化前散布 0.5%，优化后 3.8%。

剩下的 **~18 s 就是 DBoW3 数据库重建，而且这是设计下限** —— `Database::save/load`
已经实现并验证正确（top-10 完全一致，`max|ΔScore| = 0`），但 k10L5 的加载比重建慢
**387 倍**（595.5 s vs 1.54 s），根因是 `cv::FileNode::operator[](int)` 的线性遍历
被套在逐节点循环里，是 O(n²) 复杂度缺陷。所以这 18 s 收不回来。

`_warmup_nav_path_search` 那 2.7 s 只是移出了构造函数（后台线程），并没有被藏进重建里
—— **也不能藏**，见 `map_node.py` 里那段注释：把线程提到重建之前实测让 `__init__`
恶化到 44481 / 32345 ms，因为 DBoW3 重建不释放 GIL，numba 编译器和它互相抢，
其中一趟 JIT 自身从 2.7 s 被拖到 21.7 s。**串行比争用快。**

分段耗时（gt→day，ms，mean/p50）：

| 阶段 | mean | p50 | 说明 |
|---|---:|---:|---|
| `match` | 132.3 | 137.6 | 最大头，3× FLANN-LSH |
| `depth3d` | 59.1 | 63.1 | 纯 Python for 循环，有优化空间 |
| `feature_extract` | 63.4 | 63.9 | ORB |
| `db_load` | 46.5 | 47.7 | eMMC 读 |
| `candidate_search` | 25.0 | 24.8 | DBoW3 检索 |
| `pnp` | 8.3 | 7.6 | |
| `publish` | 2.9 | 2.7 | |

### 内存和温度

`MemTotal` 只有 1307 MB，所以**词典选择是生死问题**：默认双份 `ORBvoc` 要 1735 MB，
在这块板上必然 OOM。换 `voc_office_k10L5` 后峰值 407 MB，运行期最低可用内存 517 MB，
峰值 CPU 温度 80 °C（降频阈值 95 °C 被动 / 110 °C 临界，有余量）。

### numba

10 个 `@njit` **全部在导航路径搜索里，一个都不在重定位路径上**。代价发生在
`MapNode.__init__ → _warmup_nav_path_search()`。`NUMBA_CACHE_DIR` 生效后
**启动从 66.1 s 降到 34.2 s（省 32 s）**，缓存 25 个文件 / 576 KB。

> 🔴 `.nbc` 是**目标 CPU 的机器码**，不能在 x86 上预编译再拷到 aarch64，
> 必须在板上真跑一次来生成。

---

## 7. 还没解决的

按对「白天办公室导航」这个当前目标的阻塞程度排序：

| | 问题 | 影响 |
|---|---|---|
| ✅ | ~~**PnP 之后没有任何几何校验**（只检查 `landmarks>40` 且 `inliers≥20`）。板上独立复现：gt→day 有 1/28 个查询返回 `success=True` 但 XY 误差 **1.9e14 m**、旋转 **112°**，还带着 weight>0.9 进 Ceres 污染 map→odom~~ 已修（`8f67e8f`）。根因不是错匹配而是**几何退化**：无纹理帧只出 1 个 ORB 点 → `ORBMatcher` 因训练集给不出第二近邻而把 `k` 降到 1，**完全绕过 Lowe ratio test**，352 个匹配全指向那一个像素 → 无穷远处的相机把所有 3D 点投到同一像素，`solvePnPRansac` 得到重投影误差 **恰好 0.00**、352/352 内点、权重 1.0。**任何基于计数的检查都看不见它**。改为校验观测点是否真的张开（`distinct >= 20`、`spread >= 8 px`）+ 位姿是否落在地图外 50 m 内。板上 1159 白天 / 1172 夜间查询实测命中率 0.5% / 27.7% | ⚠️ 上游 main 同样不免疫：它更严的 `rerank_by_pnp_inliers`（80 点 / 50 内点）被 127 全部满足 |
| 🔴 | **夜间成功率只有 14.2 %**，且根因在**特征层不在检索层**：夜间每帧只有 326 个 ORB 点（白天 908–926），甚至有整帧 0 个。换词典只能到 16.7 % | 换词典/调检索是死路。要走补光 / 换特征 / 夜间图只夜间用。**「晚上没灯」这个前提下，纯可见光无主动照明的方案不成立** |
| ✅ | ~~启动 34 s：`LoopClosure.__init__`(bow) 用 `get_depth_embedding_features_images` 取描述子，白读 **1564 MB depth**~~ 已修（`35628bf`），改用 `TinyNavDB.get_features()`。板上实测 34.9 → **20.3 s**，见上 | 冷启动时间 |
| ✅ | ~~`__init__` 建两个 `LoopClosure` 各载一份完整词典~~ 已修（`35628bf`）。实际驻留的是**四份**不是两份（2 份 Python + 2 份 `Database` 深拷贝）；共享后 ORBvoc 1189 → 628 MiB、k10L5 123 → 63 MiB，板上 RSS 363 → 304 MiB | 大词典下白费 596 MB |
| 🟡 | ORB 二值描述子被存成 float32/255，`features.dat` 145 MB（本可 36 MB），每次匹配还要转回 uint8 | 磁盘 + `db_load` 耗时 |
| ⚪ | `depth3d` 是纯 Python for 循环，59 ms | 有优化空间但当前不是瓶颈 |
| ⚪ | LeKiwi 轮速里程计代码已写完自测通过，但**没在真硬件上标定过**；还有和 `lekiwi_control.py` 的串口独占冲突、`base_link → camera` 静态变换缺失 | 见 [`wheel_odometry.md`](wheel_odometry.md) |

不影响当前目标但记一下：`embeddings` 是 Dummy 零向量（所以 `--descriptor embedding` 报错是正常的，
不是 bug）；三张地图的描述子宽度都是 32，是真 ORB 不是 SuperPoint。

### 固件在「VIO 收不到图像」的重启死循环里 —— **只有物理断电能救**（2026-08-18）

现象是 web 端 left / right / depth 全部空白。判据不是「话题没了」而是**发布者数为 0**：

```
/camera/camera/depth/image_rect_raw    0 帧/10s  pub=0
/camera/camera/infra1/image_rect_raw   0 帧/10s  pub=0
/camera/camera/vio_image               0 帧/10s  pub=0
ros2 topic list  只剩 /parameter_events 和 /rosout
```

`insight-ctl s99 status` 里刷屏的是这一对，每秒几十轮：

```
key_frame_state_estimator.cc:773] Image data is lost for a long time, vio will be stopped.
[insight_full]: VIO tracking lost detected, restart required
```

**已排除的（都测过，都不是原因）：**

| 假设 | 排除依据 |
|---|---|
| 时钟跳变（自动对时在固件启动后才改时钟） | 时钟已正确后单独重启固件，循环照旧 |
| FastDDS 共享内存残留 | 清过 `/dev/shm`（31→4 个文件）无效；整机重启后循环依然存在 |
| `is_time_sync` 没跳变 | 跳过 0→1，NTP 偏差只有 0.0004 s，无效 |
| 传感器没起来 | dmesg 里 `sensor_start sc132gs flow0/flow1`、`sensor_start imx415 flow2` 三颗全部 `start done` |
| MIPI 链路报错 | `/sys/class/vps/mipi_host{0,2,3}/fatal/*` 和 `status/icnt` 全为 0，`state : 2(start)` |
| ion 内存耗尽 | 167 / 1024 MiB、118 / 1024 MiB，无孤儿、dmesg 无分配失败 |
| 系统内存不足 | `MemAvailable` 773 MiB |
| 相机配置被改坏 | `looper.vio.runtime.json` 内容正常（`image_interval_max: 0.5` 正是那 0.5 s 判据的来源） |

**已定位到的范围**：IMU 是好的 —— 每轮重启 `mono_static_initializer` 都成功
（`StaticInit time 0.30 acc bias: ...`）。所以断点**只在图像通路上**，位置在
「ISP/VSE 出图 → `insight_full` 的图像回调」之间，内核层面以下都是干净的。

⚠️ 一个线索：只有 `mipi0`（彩色 imx415，4 lane）打印了 `entry hs reception`，
左右目的 `mipi2` / `mipi3`（各 1 lane）只有 `start cmd: 0 real` 没有进入高速接收。
但两者错误计数都是 0，无法据此定论。

**✅ 修法：物理断电重新上电。** 已验证 —— 断电重启后一次就好：

```
/camera/camera/depth/image_rect_raw    4.7 Hz   pub=1     （depth_frame_skip=4，符合预期）
/camera/camera/infra1/image_rect_raw    20 Hz   pub=1
/camera/camera/vio_image                20 Hz   PoseStamped
/camera/camera/vio_100hz                99 Hz
/camera/camera/vio_status            TRACKING_STATIC
固件日志最近 90 秒：No entries    （之前是每秒几十行）
```

**为什么只有断电有效**：`insight-ctl s99 restart` 和整机 `reboot` 都试过，都不行。
`reboot` 是热复位，**不掉传感器的电源轨**，所以传感器里被锁住的状态能跨过重启活下来。
这也解释了驱动层为什么一路报成功：驱动是通过 I²C 写寄存器然后报 `start done`，
它并不知道传感器实际上没在出数据。唯一观察到的不对称是只有 `mipi0`（彩色）进了
`entry hs reception`，左右目的 `mipi2`/`mipi3` 没有。

怎么进到这个状态的没有定论，两个都说得通、从软件侧分不开：(a) 那个重启循环本身 ——
`insight_full` 以每分钟约 18 次的频率反复拆掉重建 VIO，持续了近 50 分钟，每一轮都重跑一遍
传感器启动序列，中途被打断的传感器一旦锁住，后面每次初始化都会以同样方式失败（自我维持）；
(b) 供电瞬时跌落。

**判断规则**：如果 `insight-ctl s99 restart` 之后 30 s 内 `Publisher count` 没回到 1，
而日志里有 `Image data is lost for a long time`，就**别再试软件手段了，直接断电**。

⚠️ 顺带一个探针陷阱：`/camera/camera/vio_image` 是 `geometry_msgs/PoseStamped`（VIO 位姿），
**不是图像**。用 `Image` 订阅它会永远收不到消息而 `count_publishers` 照样是 1 —— 看起来
像故障，其实是类型不匹配。

---

## 7.5 🔴 外来的 DDS 发现报文会把板上任何进程 OOM 掉（2026-08-20 定案，已修）

**症状**：任何 ROS 2 进程随机涨到约 1.5 GB 被 OOM 杀。表现极具误导性 —— "app 起来几十秒又
消失"、"相机不发数据"、"planning_node 内存泄漏"。一天里发生 14+ 次，看不出规律。

**根因**：办公室 WiFi（DEEP-RD）网段上有 **5 台别的机器**在往 DDS 发现多播地址
`239.255.0.1:7400` 发包（`ROS_DOMAIN_ID` 默认 0）。其中的 `ParticipantEntitiesInfo`
本板 FastDDS 解不开，长度字段被当成垃圾值，FastCDR 照着它申请约 1.5 GB：

```
Fast CDR exception deserializing message of type rmw_dds_common::msg::dds_::ParticipantEntitiesInfo_
''Bad alloc' exception deserializing message of type ... ParticipantEntitiesInfo_
```

ion 扩容后 `MemAvailable` 正好约 1.4 GB，所以这个申请必然触发**全局** OOM。

**逐步剥离实验**（每步都实测）：

| 实验 | 结果 |
|---|---|
| 订阅 `camera_info`（几百字节） | 1.54 GB 被杀 |
| **收到 0 条消息** | 照样被杀 → 与消息无关 |
| 订阅一个不存在的话题 | 照样被杀 |
| **完全不订阅**，只 `rclpy.spin_once()` | 照样被杀 |
| 只 `rclpy.init()` + `time.sleep` | **41 MB，活着** |

→ 凶手是 `spin_once` 处理发现报文，和话题/订阅/消息/发布者全都无关。

**修法：两处都改成 `ROS_LOCALHOST_ONLY=1`**，少一处白改（固件和 tinynav 必须同域）：

| 文件 | 归属 |
|---|---|
| `/etc/init.d/looper/setting/ros2_env.conf` | 固件（备份 `.bak.pre_localhost`） |
| `tool/x5_board/env.sh` | tinynav（已提交） |

代价：PC 端 `ros2 topic list` 看不到板上话题。web UI 走 HTTP，不受影响。
另一个等效修法是 `ROS_DOMAIN_ID=42`（换多播端口），实测同样 0 报错，好处是仍能被 PC 看到。

**修后验证**：连跑 11 次探针，RSS 恒 53→54 MB、每次收 240–241 条、0 报错；app 连续 125 s
五个进程 RSS 完全不动。

### 两个连带陷阱

🔴 **`/userdata/x5/env.sh` 是会分叉的第二份拷贝。** `app_start.sh` 用的是**它**
（`ENV_SH="${ENV_SH:-/userdata/x5/env.sh}"`），不是仓库里的 `tool/x5_board/env.sh`。
只改仓库那份，app 照样 OOM。**板上已做成软链**指向仓库那份。

🔴 **固件被 OOM 连杀 5 次后 systemd 彻底放弃拉起。** `S99all_run.service` 是
`Restart=always` 但 `StartLimitBurst=5`，耗尽后变成
`Start request repeated too quickly` + `failed (Result: oom-kill)`，**再也不自动重启**。
判据是 `systemctl status S99all_run`，恢复要：

```bash
systemctl reset-failed S99all_run && systemctl start S99all_run
```

⚠️ **诊断纪律**：这个 bug 的全部线索都在 **stderr**。用 `2>&1 | tail -1` 看会崩的进程
等于把唯一的证据扔了 —— 我因此编出过"每条消息涨 179 MB"和"BEST_EFFORT vs RELIABLE"
两个错误解释。排查时先给探针加 `resource.setrlimit(RLIMIT_AS, ...)`，把 `bad_alloc` 变成
本进程的 `MemoryError`，别把固件一起拖死。

## 7.6 持久化 journal:光开 `Storage=persistent` 不够,时钟会把它废掉

排查重启原因需要跨重启的日志,所以 2026-08-21 开了持久化
(`/etc/systemd/journald.conf` 的 `Storage=persistent` + 64 M 上限 + `mkdir /var/log/journal`)。
**开完之后重启一次,上一次启动的日志依然没留住。**

原因是时钟,不是配置。X5 没有 RTC,每次上电回到 **2025-08-26**;`board-autotime.service`
会用 NTP 修好,但它要等 WiFi 拿到 IP,**比 journald 晚约 13 秒**。journald 自己给出了确证:

```
Notice: journal has been rotated since unit was started, output may be incomplete.
```

往前跳一年那一下**强制轮转日志文件**,而 `journalctl --list-boots` 按时间排序 ——
每次启动都从 2025-08-26 开始,于是所有启动塌成一条,看不出边界。

**修法:开机极早期先把时钟顶到"上次见过的时间",让跳变从一年缩到几分钟。**

| 文件 | 作用 |
|---|---|
| `board_savetime.sh save` | 把 `date +%s` 写到 `/var/lib/board-lasttime` |
| `board_savetime.sh restore` | 读回来,**只在保存值更新时**才 `date -s`(否则会把 NTP 校准过的时间拽回去) |
| `board-savetime.timer` | 每 5 分钟存一次 |
| `board-savetime.service` | `ExecStop` 再存一次,免得关机前 5 分钟白丢 |
| `board-restoretime.service` | `DefaultDependencies=no` + `Before=systemd-journald.service`,**必须早于 journald,晚了就白做** |

⚠️ **`[ "$saved" -gt "$now" ] && date -s ...` 这个写法有坑**:测试为假时脚本以 1 退出,
systemd 把单元标成 failed —— 而"不需要顶"恰恰是最常见的正常情况。必须显式 `exit 0`。
四种情形都要验:不需要顶 / 文件里是垃圾 / 文件不存在 / 保存值在未来。

⚠️ **`board_autotime.sh` 本身没问题**,它需要网络所以不可能更早;这两个服务是互补的:
`restore` 负责让 journald 看到大致正确的时间,`autotime` 负责最终校准。

顺带:`hostapd.service` 因为没有 `/etc/hostapd/hostapd.conf` 在死循环重启,
**每 2 秒刷 7 行日志**,会把 64 M 上限撑爆、把有用的记录挤掉。已 `systemctl disable --now hostapd`
—— 我们用 WiFi 客户端模式,不需要它当热点。

## 8. 相关文件

| 文件 | 作用 |
|---|---|
| [`tool/x5_board/env.sh`](../../tool/x5_board/env.sh) | 板上环境变量，修 §1 的四处缺失 |
| [`tool/x5_board/sync_board_time.sh`](../../tool/x5_board/sync_board_time.sh) | 时钟同步 + 设 `is_time_sync` flag（§2） |
| [`tool/x5_board/decord_shim/decord.py`](../../tool/x5_board/decord_shim/decord.py) | PyAV 实现的 decord 替代（§5） |
| `x5_work/reloc_offline_eval.py` | 离线重定位评测脚本（不在仓库里） |
| `x5_work/reloc_pc_baseline.md` | PC 侧基线，含更大样本量的成功率 |
| [`depth_frame_skip.md`](depth_frame_skip.md) | depth 降到 5 Hz |
| [`fix_64gb_mipi.md`](fix_64gb_mipi.md) | 64GB 相机 MIPI 修复 |

## 🔴 持久日志和时钟保存:两个"看着装好了其实没生效"的坑

**2026-08-21 板子掉线过一次,想查原因时发现日志根本不在。** 两个独立的失败:

**① `/var/log/journal` 建好了,但当次启动 journald 还在往 `/run/log/journal`(内存)写。**
`Storage=persistent` + 建目录**不足以让正在跑的 journald 切过去** —— 要么重启 journald,
要么 `journalctl --flush`。所以持久日志是从**下一次启动**才真正生效的,而故障就发生在这之前。
判据:`journalctl` 各时段行数出现空洞(实测 15:00–16:00 有 194 行、**16:00–18:50 零行**、
18:50 之后 5909 行),而且 `/var/log/journal` 里只有一个 `system.journal`、创建时间等于
本次启动的早期时刻。

**② `board-savetime.timer` 每次启动只触发一次。**
`board-savetime.service` 原来是 `Type=oneshot` + `ExecStop=`(为了关机存一次)+
**`RemainAfterExit=yes`**,于是单元常驻 active,而 **`OnUnitActiveSec` 对一个从不退出 active
的单元不会再排下一次** —— 实测 `LAST=开机+2min, NEXT=n/a`,存档永远停在"上次开机时刻"。
后果:2026-08-21 那次重启 `board-restoretime` 恢复出来的时间**慢了 3 小时 35 分**,
journal 早期条目被打上 3.5 小时前的时间戳,`--list-boots` 因此看起来只有一个横跨 4 小时的启动。

修法是**拆成两个单元**:定时那个是纯 `oneshot`(绝不能有 `RemainAfterExit`),关机那个单独一个
`board-savetime-shutdown.service`(必须有 `RemainAfterExit=yes`,否则 `ExecStop` 没机会跑)。
⚠️ **改完必须 `systemctl stop board-savetime.service`** —— `daemon-reload` 不会把已经处于
active 的单元拽回来,不停掉的话 `NEXT` 依旧是 `n/a`(我第一次就是这样,以为没修好)。
验收判据:`systemctl list-timers board-savetime.timer` 的 `NEXT` 有具体时刻、
`systemctl is-active board-savetime.service` 跑完是 `inactive`、`/var/lib/board-lasttime` 每 5 分钟变。

## 掉线要能事后分析,需要三层,现在都装上了

2026-08-21 那次掉线**没有任何可用证据**:journal 里只有 systemd 和内核消息,
既没有"当时链路什么样"也没有"当时电压什么样"的时间序列 —— 而那正是唯一能区分
「WiFi 掉了」和「板子死了」的东西。补了三层:

| 层 | 装了什么 | 覆盖什么 |
|---|---|---|
| 内核崩溃 | **pstore 本来就是开的**(`pstore: success mode=2`,挂在 `/sys/fs/pstore`) | panic / oops 能跨硬复位留下 |
| 链路与热 | `board-health.service`(`tool/x5_board/board_health.py`),每 10 s 一行进 journal | RSSI / 误报警 / IGI / rx_rate / link / 温度 / 负载 / 可用内存 / 到 PC 的 RTT |
| 电压 | `diffcar_control` 每 2 s 发 `e`,发布 `/battery`,并在最低值 < 10.0 V 时告警 | 电池带载塌压 —— 只有 ESP32 知道,而串口只有它持有 |

**健康行写 stdout 交给 journald**,这样它和内核/驱动消息**按时间穿插在一起**,排查时不用对
两份时间戳。⚠️ 但「不写自己的文件」这个决定 2026-08-24 被推翻了 —— journal 在 rootfs 上,
硬断电后一行都没剩。现在是双写,见下面那一节。

**下次掉线的判据速查:**

| 现象 | 结论 |
|---|---|
| `rssi` 骤降 / `link=0` / `rx_rate` 掉档 | 空口/射频 |
| WiFi 各项正常但 `pc=MISS` 连续多行 | 上游(AP/PC)问题,板子自己还活着 |
| 健康行整段断裂,下一行就是开机 | 板子重启了(掉电或 panic;panic 看 `/sys/fs/pstore`) |
| 断裂前 `/battery` 告警在下探 | 掉压 |
| `temp` 接近 95 度 | 降频。不是掉线,但会把一切拖慢 |

🔑 **它上线一分钟内就抓到一件事**:空载电压已经 9.79–9.92 V,而固件闭锁线是 9.60 V,
只剩 0.2 V 余量(当天下午还是 10.6–10.9 V)。**这种状态下做任何带载标定都会拟合出垃圾。**

### 2026-08-24 掉线定案:带载塌压把 USB 网卡打掉了

这三层第一次真的派上用场,而且暴露了它们自己的一个漏洞。

**时间线**(取自 `/userdata/x5/logs/nodes/*_diffcar_control.txt`):

```
09:55:25  PC reachable again
10:26:12  battery sagged to 9.97 V (now 9.85)   <- 正在跑 360° 闭合标定
10:26:14  battery sagged to 9.86 V (now 9.92)
10:26:16  battery sagged to 9.96 V (now 9.96)
10:26:18  battery sagged to 9.95 V (now 10.14)
10:26:27  PC unreachable -- LED goes cyan       <- 9 秒后 WiFi 断
10:34:35  battery sagged to 9.99 V (now 10.12)  <- 网断了但节点还在写日志
10:35:01  battery back to 10.67 V
10:36:17  人工断电重上
```

🔑 **WiFi、ESP32、Looper、电机全部由同一个电池供电**(已确认)。电机电流把这条共用轨拉到
9.85 V,9 秒后 USB 网卡(`0bda:b711`, RTL8710BU)就掉了。固件的低压闭锁线是 9.60,只差 0.25 V。

⚠️ **9.85 V 是固件内部跟踪的最小值,不是 `/battery_voltage` 话题上的数** —— 那个话题 0.5 Hz,
当时报的最低只有 10.30。固件采样快得多,抓到了话题漏掉的瞬时凹坑。**光看 web 上的电压不够。**

**青灯一个人就把故障定位了。**触发条件是 `!g_pcOk && (now - g_pcStamp < 8000ms)`,也就是
ESP32 在最近 8 秒内收到过 Looper 发来的 `K 0`;而**橙灯(UART1 静默 5 s)优先级压过青灯**,
当时没亮橙。两条合起来:

| 环节 | 状态 | 依据 |
|---|---|---|
| X5 板子 | 活着 | 不然发不出 `K 0` |
| ROS app / `diffcar_control` | 活着 | 它就是发 `K 0` 的进程,且必须在 8 s 内发过 |
| 串口 `/dev/ttyS3` | 通 | 不然会亮橙,橙压青 |
| **WiFi 到 PC** | **断** | `diffcar_control` 自己的 ICMP 探测连丢 3 次 |

边缘欠压能打死一个 USB 外设而不重启主控,这很典型;而掉电的 USB 设备**不重新枚举就一直是
死的**,所以它没自己恢复。`/sys/fs/pstore` 是空的,排除 panic。

### 🔴 journal 没活过断电 —— 「装了没生效」的第三次

`board_health` 的健康行写的是 stdout → journald → `/var/log/journal`,而重启后:

```
journalctl --list-boots           ->  只有当前这一次开机
/var/log/journal/<id>/            ->  只有一个本次开机创建的 system.journal，没有归档文件
journalctl -u board-health | wc -l ->  5 行，全是重启之后的
```

**唯一能分开「网卡掉电」和「射频问题」的那份 rssi/link 时间序列,正好是丢掉的那一份。**
上面那条时间线是从 `/userdata` 上的**节点日志**拿的 —— 那个分区连 8 月 21 号的日志都还在。

rootfs 本身是持久的(8 月 21 号装的脚本全在,`/var/backups` 的 mtime 还是 8 月 18),所以不是
整个 `/var` 易失。最可能是 journald 对持久存储的 `SyncIntervalSec` **默认 5 分钟**、而且用
mmap 写 —— 硬断电时脏页还没回写就全丢。**已在 `/var/log/PERSIST_TEST` 留了标记文件**,
下次重启一看便知:还在 = `/var/log` 持久、问题在 journald 的同步策略;没了 = `/var/log` 本身易失。

🔑 **教训:凡是要活过断电的东西,一律写 `/userdata`,别信 rootfs。**

**已修**(`board_health.py`):

- 双写 —— 除 stdout 外再追加 `/userdata/x5/logs/board_health.log`,8 MB 单代轮转
- 新增两个字段,专门用来分开这次分不出的那两种故障:

```
2026-08-24 10:43:58 up=461 load=1.97 memavail=1391M temp=82.0/81.9
                    rssi=-55 qual=86 link=1 igi=0x1c fa=512 rx_rate=MCS7
                    carrier=1 usb=1 pc=153
```

| 下次掉线看到 | 结论 |
|---|---|
| **`usb=0`** | 网卡从 USB 总线上掉了 → **供电塌陷** |
| `usb=1` 但 `carrier=0` | 网卡在、链路断 → 空口或 AP 那头 |
| `usb=1 carrier=1` 但 `pc=MISS` 连续多行 | 上游网络,板子和网卡都好 |
| `rssi` 先塌下去 | 射频(这块网卡余量本来就薄:−6 dB、速率掉到 MCS4) |

**预防**:①电池是首要变量,静置 11 V 的 3S 电池只剩三分之一,带载塌 1 V 以上说明内阻已经不小;
②`max_yaw` 从 0.8 降到 0.6 —— 0.8 rad/s 原地转是这台车电流最大的动作,正是它把电池拉塌的,
而且 VIO 在那一档本来就已经在跳变(见 `diffcar.md`)。

### ⚠️ 两个读数曾经在骗人,已修

**① `/battery` 是百分比契约,不能往里发电压。** 它原本是四足 `unitree_control` 的字段,
前端直接 `'${battery}%'` 渲染并在 <20 时标红。把伏特发进去,9.98 V 会显示成 **"10%"** ——
而 3S 锂电 9.98 V 的真实余量**恰好**就是 5~10%,**错数长得像对的,比明显错的更危险**。
现在电压走 **`/battery_voltage`** → 状态里的 **`batteryVolts`**,前端显示 `10.60 V`,
阈值按固件闭锁线定(<10.2 红 / <10.8 橙),因为人要的是"离 9.6 V 还有多远"而不是一个猜出来的百分比。

**② `cpu_per_core` 和 `cpu_percent` 曾来自两个不同的时间窗。** 合计用
`psutil.cpu_percent(interval=0.2)`(真实 0.2 s 窗口),每核用 `interval=0.0`(**距这个进程上次
percpu 调用的增量**,也就是距上次刷新面板的整段时间);而后端刚起时的第一次调用返回的是
**开机至今的平均值** —— 和 `ps -o pcpu` 同一个坑。现在一次阻塞采样同时给出两者,
合计 = 每核均值,严格一致。

### 🔑 `load` 和 `CPU%` 不是一回事,别混着读

实测(5 s `/proc/stat` 增量,scheme a + planning 在跑):

```
合计 busy=61.3%  idle=38.7%  iowait=0.0%   -> 折合 4.90 / 8 核
loadavg=7.27     procs_blocked(D)=0
procs_running(R): 均值 8.8, 峰值 15
```

- **不是 I/O 阻塞**:`iowait=0.0%`、D 态恒为 0。(曾把 load 高归因于 IO 等待,那是错的。)
- 真因:**瞬时可运行任务均值 8.8、峰值 15,而只有 8 个核**,但总 CPU 只吃 4.9 核 →
  这些线程**成串同时醒来、各自只干一点点活**(每来一帧图像几十个 ROS 回调同时唤醒)。
  `loadavg` 是对瞬时 R 做指数平均,抓到的是突发峰;CPU% 是时间平均,峰被摊薄了。
- 🔑 **实用结论:8 核 61% 忙 = 吞吐有余量,但运行队列经常长于核数 → 受损的是延迟不是吞吐。**
  这正是轨迹发布 p50 1.17 s 的来源 —— 不是算不完,是排队。
