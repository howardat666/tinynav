# Looper WiFi：2 Mbps 的有效吞吐是怎么定案的 —— 2026-08-25

## 结论先行

| 项 | 实测 |
|---|---|
| 物理层协商速率 | **72.2 Mbit/s**（MCS7 / 20 MHz / short-GI，`iw` 报告） |
| 实际可用吞吐 | **约 2 Mbit/s**（0.165–0.249 MB/s，多次） |
| 有效率 | **2.8%** |
| 信号 / 噪声 | −50 dBm、qual 95–99、`Total False Alarm` 310–650 —— **都正常** |
| 网卡 | RTL8710BU（`0bda:b711`），802.11n 单流，**硬件无 5 GHz** |
| 根因 | **2.4 GHz 空口拥塞**（29 个 AP 挤信道 1/6/11，各 11/10/8 个），叠加可能的 AP 侧限速 |
| 可行的根治 | 只剩「买双频 USB 网卡 + 交叉编译驱动」一条 |

**周期性掉线（15–69 秒）是独立于吞吐的另一个现象**，不要混为一谈。netheal 的第 1 级 `link-bounce` 通常 189 ms 就能恢复。

## 七个假设，逐一排除

| 假设 | 判据 | 结果 |
|---|---|---|
| CPU 饥饿导致降速 | 停 board-app 前后吞吐 0.165–0.247 MB/s，无变化 | ❌ |
| ssh 的 AES 加密开销 | 不加密的 Python socket 2.09 Mbps vs ssh 1.8 Mbps | ❌ |
| 信号弱 | rssi −50 dBm，且 `iw` 报 72.2 Mbit/s 满速 | ❌ |
| 射频噪声 | `Total False Alarm = 310`（低） | ❌ |
| TKIP 禁用 HT 速率 | `sec_info` 的 `enc_alg=0x4` 是 AES/CCMP | ❌ |
| 发射速率锁死在 CCK | 1400 B ping min RTT **9.555 ms** < 1 Mbps 下的理论下限 22.4 ms | ❌ |
| 同板第二个 2.4G 电台干扰 | `8852bs`/`8188gu` 引用计数为 0，`/sys/class/bluetooth/` 为空 | ❌ |

**拥塞的指纹是「延迟低 + 吞吐低」同时成立**：小包能插进空隙（RTT 6–54 ms），持续传输抢不到空口时间。

## ⚠️ `tx_rate` 字段不可信，我据此推了三轮错结论

`/proc/net/rtl8710bu/<dev>/sta_tp_info` 的 `tx_rate : CCK_1M(L)` **恒定不变** —— 空闲时是它，2 MB 真实传输全程是它，停 app 是它，`ifconfig down/up` 重新关联之后还是它。同一文件的 `TP {Tx,Rx,Total}` 全程为 0，同样没在工作。

判它死刑的是一条算术：1400 字节 = 11200 bit，1 Mbps 下单程 11.2 ms、往返至少 22.4 ms，而实测 min RTT **9.555 ms**，物理上不可能。后来 `iw dev <dev> link` 直接给出 `tx bitrate: 72.2 MBit/s`，证实前者是假象（多半只反映管理帧那一档）。

**判据**：
- 测吞吐必须**真的传数据并计时**，不要读驱动的速率字段
- 零流量下读任何速率统计都无意义 —— 先确认 `TP` 非零
- 交叉验证用 `ping -s 1400` 对比 `ping -s 64`：ping 走内核不加密，**不受板子 CPU 影响**，而 ssh/scp 混了 AES 在里面
- **`iw` 走 nl80211 标准接口，比这个 out-of-tree 驱动自己写的 proc 文件可信**

## 硬件底牌：为什么三条出路全堵死

**USB 只有一个可用口，且被 WiFi 网卡占着**
- 网卡路径 `.../35100000.usb3/.../usb1/1-1`，全板 USB 设备只有它一个
- 两个 role-switch：`35100000`=host（C 口，网卡在此）、`35300000`=device（gadget，`usb0` 有 IP 但 device 链接为空，物理口没引出）
- ⚠️ **把 35100000 切回 device 会让网卡掉电 = 远程失联**，只能在能物理接触板子时做

**板载没有 WiFi 芯片**
- SDIO 上只有 `mmc0:0001`，是 58.3 GiB 的 eMMC（`mmcblk0`）
- `8852bs` 要的是 `sdio:c07v024CdB852*`，硬件没焊；蓝牙同理，模块加载但无 hci 设备

**内核里没有任何双频 USB WiFi 驱动**
- USB 的只有 `8723du.ko`（RTL8723DU，**仍是单频 2.4G**）
- `aic8800_*.ko` 支持 11nac/5GHz，但 alias 全是 `sdio:`，且 `/lib/firmware` 无 aic 固件
- 当前驱动 `8188gu` 标着 `(OE)`，是外部编译塞进去的
- 板上无 gcc/make，但有 `/lib/modules/$(uname -r)/build` → 换卡必须在 PC 上交叉编译

**eth0 不可用**：operstate=down，`Supported link modes: 10baseT/Full`，物理口存疑。

## ⚠️ 切换 SSID 会把板子搞丢，不要再试

两次尝试切到 `DM_Stream_test` / `DEEP`，**两次都失联**（第一次归因于回退定时器重复执行，修掉后第二次照样发生）。这块网卡在反复 `down/up` + 重新关联之后就是容易起不来，netheal 要花好几分钟甚至救不回来，最后都得断电重启。

两个 SSID 均**关联失败**（40 秒超时），原因未定（密码/认证方式/超时）。注意「关联失败」≠「穿透不了」—— 前者是链路层没连上，后者是连上之后设备间不通，我们从没测到第二步。

如果将来一定要测，**必须**：无条件定时回退（不能依赖任何探测判断 —— 最危险的情况是「连上了但被隔离」，那时板子自己觉得网络正常，netheal 不会触发），且回退进程里**不要自己写 `pkill -f` 模式**（模式串出现在自己的命令行里会把自己杀掉），直接调 `wifi-connect.sh` 让它内部清理。

## 负载与掉线的相关性（未定案，但有数据）

掉线次数按小时（2026-08-25）：

| 时段 | 次数 | 当时在做 |
|---|---|---|
| 10 时 | 16 | vlad 建图（BPU 满载） |
| 11 时 | 15 | vlad 建图 + 导航 |
| 12 时 | 5 | WiFi 切换实验 |
| 13 时 | 3 | 同上 |
| 前一天 14/16/17 时 | 6/9/2 | ORB 建图与导航 |

**BPU 满载的两小时掉了 31 次**，而大量折腾 WiFi 的两小时只掉了 8 次 —— 高峰跟着 BPU 负载走，不跟着操作走。

推测机制是**温度**：实测 `temp=90.7/91.4`，降频线 95 °C。射频前端在此温度下功放效率与噪声都会劣化，而 `rssi`/`qual` 看着正常正是因为它们量的是接收，劣化的是发射侧。

⚠️ 注意区分：先前「CPU 负载不影响吞吐」的结论仍然成立；**「负载影响掉线」是另一件事，从未被排除**。

## 不换硬件的应对

**导航本身几乎不吃带宽** —— ROS 话题全走板内回环（`ROS_LOCALHOST_ONLY=1`），WiFi 上只有 web 的 HTTP/WebSocket，其中图像预览是绝对大头，指令和状态只有几 KB。所以 2 Mbps 够用，被卡住的只有实时画面和日志下载。

候选改动：预览改「按需单帧」、日志下载走 gzip、netheal 门限 90→60 秒。

---

# 2026-08-27：关联死亡是一类独立故障，软件救不回来

## 结论先行

🔴 **在这块 rtl8710bu / 8188gu 上，`wpa_supplicant` 一被杀，关联就再也起不来，重启之前任何手段都无效。**

这是和「日常的慢和抖（拥塞）」「掉线时 fa 阶跃（干扰）」并列的**第三类故障**，判据和修法都不同。

| 故障 | 判据 | 修法 |
|---|---|---|
| 日常慢/抖 | 指标全正常但 RTT 中位数 70 ms+ | 换双频网卡（拥塞） |
| 掉线，association 没断 | `link=1 carrier=1` 全程在，`fa` 阶跃 >1000 | 踢接口（netheal 第 1–2 级） |
| **关联死亡** | **`link=0` + `association failed after 20s`** | **只有重启** |

## 证据：四次实验，两天，测前状态都很好

| 时间 | 做法 | 结果 |
|---|---|---|
| 08-27 14:25 | shell 版：`pkill` 后直接起 supplicant（**缺 `ip link up`**） | 三个 AP 全失败；netheal 四级梯度跑两遍全败；躺一小时后人工上电 |
| 08-27 15:04 | Python 版：**补上 `busybox ip link set up`** + 完全照固件顺序 + **等满 20 s** | `20s 内没关联上` |
| 08-27 15:05 | 回落固件自己的 `wifi-connect.sh` | `association failed after 20s` |
| 08-27 15:05 | 再来一次 | `association failed after 20s` |

15:04 那次的测前状态：`rssi=-51 qual=93 igi=0x36 fa=390 rx_rate=MCS7` —— **信号好得不能再好**。

🔴 **「缺 `ip link set up`」这个诊断是错的，已推翻。**补上之后照样失败，所以那一行不是根因。

**已验证无效的手段全集：**`busybox ip link up` + 新 supplicant、固件 `wifi-connect.sh`（×4）、
`ifconfig down/up`、`sh wifi-connect.sh`、`/sys/bus/usb/devices/1-1/authorized` 重新枚举。
**与信号强度、`fa`、时间点全都无关。**

## 推论：信道 A/B 在当前固件下做不了

换 BSSID 只有两条路，而它们互锁：

| 路 | 阻塞 |
|---|---|
| 重启 `wpa_supplicant` 换配置 | = 自杀，见上 |
| `wpa_cli set_network 0 bssid X` | 需要 `ctrl_interface`，而它**只能在启动 supplicant 时给** → 回到上一条 |

**唯一出路**：让 `wifi-connect.sh` 在**开机时**就写入 `ctrl_interface=/var/run/wpa_supplicant`，
重启一次，之后全程只用 `wpa_cli` + `reassociate`，**永不杀 supplicant**。
⚠️ 前提未验证：`reassociate` 在这个驱动上到底能不能换 AP。**改完先只测这一件事**，成了再跑完整 A/B。

## netheal 的重启级已打开（2026-08-27）

`board-netheal.service` 的 `Environment=NETHEAL_ALLOW_REBOOT=1` 已启用。
判据：启动行 `allow_reboot=True`，且 `/proc/<MainPID>/environ` 里有这个变量。

理由是上面的实测：**四级梯度对关联死亡完全无效，重启是唯一手段。**
对着 14:25 那次算：链路 14:25:49 死，梯度跑完 14:31:30 → 开了开关会在**故障后 5 分 41 秒**
自己重启恢复，而实际躺了 50 多分钟。

🔑 **不会重启循环。**`armed` 只在探测成功那一支置 True，`if not armed: continue` 跳过整条梯度 ——
所以「开机起来一直连不上」（AP/网关那头挂了）**永不触发重启**，只有「有过连接又丢了」才会。

⚠️ 残留场景：AP 以约 6 分钟周期反复闪断，会导致反复重启。目前不处理。

## 做危险的网络实验必须先有兜底

🔴 **不能把恢复挂在被测脚本自己的 `trap` 上。**14:25 那次 `trap restore EXIT INT TERM`
**执行了**，但 restore 自己失败了 —— 兜底必须既不依赖脚本存活，也不依赖它成功。

已验证可用的做法（`tool/x5_board/wifi_channel_ab.py`）：

```sh
systemd-run --on-active=12min --unit=wifi-ab-watchdog --collect \
  /bin/sh -c '[ -f /userdata/x5/logs/wifi_ab_done ] && exit 0; sync; systemctl reboot'
```

实测有效：15:04:36 布防 → 15:06:16 脚本判定恢复失败并**故意保留定时器** → 15:16:36 触发 →
15:18 板子自己回来，**全程无人介入**。

🔴 **撤除定时器时绝不能无条件 `systemctl stop`** —— 恢复失败时那会变成「既不写完成标记也不重启」，
板子就再躺一次。和 `trap` 那个 bug 同一形状。

## 两条出路彻底排掉（设备树/驱动级定案）

| 路 | 证据 | 结论 |
|---|---|---|
| 板载 SDIO 上 5 GHz | `8852bs`（RTL8852BS，**双频 WiFi 6**）驱动**确实加载着**，但 `/sys/bus/sdio/devices/` 空；设备树 `sdhci@35020000`（=`mmc1`）带 **`no-sdio` + `no-mmc`**，配的是 `cap-sd-highspeed`/`sd-uhs-sdr104`/`power-gpios` 的普通 SD 卡槽；全设备树无 wifi/wlan/pwrseq/realtek 节点 | 🔴 故意关闭。**驱动加载 ≠ 硬件存在** |
| 走有线 | `hobot_gmac ... eth0: no phy founded` → `eth_netdev_open, init phy error`；MDIO 总线上只有控制器无 PHY | 🔴 MAC 后面没接芯片。`Supported link modes: 10baseT/Full` 是驱动兜底显示，**不是真实能力** |

## ⚠️ 板上没有 `ip`，也没有 `ping`

`ip` 只存在于 busybox 里（`busybox ip ...`），裸 `ip` 是 command-not-found；`ping` 完全没有。
`PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin`，全都找不到 `ip`。

**取 IP 用 `ioctl(SIOCGIFADDR)` 或 `ifconfig`；测延迟用原始套接字自己发 ICMP。**

🔴 **由此发现固件的一个 bug：`/etc/init.d/looper/wifi-connect.sh` 第 9 步用裸 `ip -4 addr show`
→ 变量恒空 → 每次都报 `ERROR: no IP assigned` 并 `exit 1`，即使完全成功。**

```
[15:39:17] Requesting IP via udhcpc
[15:39:17] ERROR: no IP assigned      ← 而板子这一刻已经拿到 192.168.19.218 并一直在线
```

**所以它的返回码和最后那行日志都不能当判据**（要看有没有真拿到 IP）。
netheal 靠复验探测判定而不看返回码，**所以它没受影响**。
修法是一个词：`ip -4 addr` → `busybox ip -4 addr`。
⚠️ 而第 7 步的 `association failed after 20s` 是**真错误**（在第 8 步之前），别和这个混淆。

## 吞吐现状（2026-08-27 15:2x，2 MB，接收端计时）

| 方向 | 本次 | 08-18 | 08-20 | 08-21 | 驱动天花板 |
|---|---|---|---|---|---|
| 板→PC（发） | **2.080** | 1.33–1.97 | 0.874 | 2.477 | ~2.5 |
| PC→板（收） | **7.356** | 4.13–9.99 | 2.043 | 13.111 | ~14.5 |

射频：`rssi=-55 qual=91 igi=0x3d fa=559 rx_rate=MCS7`。

发方向贴着驱动天花板，正常。**收方向只有 08-21 峰值的一半**，与 `igi=0x3d`（驱动主动降低
接收灵敏度）自洽 —— 08-20「慢」那次也是 0x3d，08-21「快」那次是 0x38。
⚠️ **别用 `rssi` 解释**：−55 和 08-21 的 −53 差不多而吞吐差一倍，这个错已经犯过一次。

预览预算：infra1 @5 fps 需 0.86、depth q50 需 0.26 —— 都装得下；**`color` 需 3.59，仍然超。**

## 「不换硬件的应对」的状态更新

| 候选 | 状态 |
|---|---|
| 预览 base64 → 二进制（省 33%） | ✅ 已完成 |
| 预览降采样/降质量/限帧率 | ✅ 已完成：320 px 长边 + JPEG q50 + 5 fps（慢话题 2 fps） |
| 前端 bundle gzip | ✅ 已完成：3.04 MB → 0.89 MB |
| 驱动省电参数 | 🔴 无余量：`rtw_power_mgnt`/`ips_mode`/`lps_level`/`adaptivity_en` 全为 0，`iw` 报 `Power save: off` |
| netheal 门限 90 → 60 秒 | ❌ **作废**。90 秒是**有意**保守的（第 2 级的拆重建会把「关联着但过不了流量」打成「连都连不上」）。真正缺的是重启级，已打开 |
| 换双频 USB 网卡 | 仍是唯一根治。必须在能物理接触板子时做（C 口是唯一 USB 口，驱动起不来就彻底失联） |
