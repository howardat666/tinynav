# X5 的 USB 能不能做主机（host）—— 2026-08-14 实测

> ## 🔴 结论：**能，但需要一次运行时的角色切换**
>
> 此前 `wheel_odometry.md` 和 [[x5-nav-project-scope]] 里写的「**X5 不能做 USB host**」是**错的**。
> 观察是对的（`/sys/bus/usb/devices/` 为空、第二个 udc `not attached`），
> 推论是错的 —— 空的原因是**两个控制器都被绑成了 gadget（设备）**，不是硬件不支持。

测试机：早期标准版 Looper（eMMC 15 GB），固件在跑，`root@169.254.10.1`。

## 1. 实测到的拓扑

```sh
ls /sys/class/udc/                    # 35100000.usb  35300000.usb
ls /sys/bus/usb/devices/              # 空 —— 没有任何 host root hub
ls /sys/bus/platform/drivers/ | grep -E "dwc3|xhci"
                                      # dwc3  dwc3-of-simple  xhci-hcd
for f in $(find /sys/firmware/devicetree/base -name dr_mode); do cat $f; done
                                      # otg   otg
```

| 控制器 | 速率 | 设备树 `dr_mode` | 当前状态 |
|---|---|---|---|
| `35100000.usb` | super-speed | **otg** | 🔴 跑着 gadget `g_comp_usb3.0`，`state=configured` |
| `35300000.usb` | high-speed | **otg** | 🟢 **`state=not attached`，空闲** |

**`35100000` 就是 `usb0`（169.254.10.1）那条 NCM 链路，也是 SSH 的唯一通路 —— 不要动它。**

## 2. 角色开关是存在的，而且是纯运行时

```sh
cat /sys/class/usb_role/35100000.usb-role-switch/role   # device
cat /sys/class/usb_role/35300000.usb-role-switch/role   # device
```

切换只需要一次 sysfs 写：

```sh
echo host > /sys/class/usb_role/35300000.usb-role-switch/role
```

🟢 **不改设备树、不改固件、不动任何系统文件，重启自动恢复** —— 在「别开壳、别改相机系统配置」的红线之内。

`xhci-hcd` 驱动已编进内核（`/sys/bus/platform/drivers/xhci-hcd/` 存在，只是没有绑定任何设备）。

## 3. 还没验证的三件事（都要实物）

| # | 未知 | 为什么重要 |
|---|---|---|
| 1 | **USB2 那个口有没有引到壳外？** | 引不出来就只能开壳，等于不可用。⚠️ `servo_bus.md` 记过「2026-08-06 试过另一个机壳外接口，三个 UART 全部环回无回声」—— 那个口有可能就是它 |
| 2 | **host 模式下有没有 VBUS（5 V）？** | 设备树里搜 `vbus` **零命中**，很可能没有 VBUS 开关 → 必须用**外部供电的 USB hub** |
| 3 | ID 脚状态 | `extcon1` 报 `USB=1 / USB-HOST=0`。用 OTG 线（ID 接地）也许能自动切，不用手写 sysfs |

## 4. 驱动现状 —— 决定该买什么网卡

```sh
uname -r                              # 6.1.83-DR-PL5.2_V1.1.0
find /lib/modules -name "*.ko*" | wc -l   # 142
```

| | 状态 |
|---|---|
| `cfg80211.ko` | 🟢 **在**（无线核心层）|
| `iw` / `wpa_supplicant` / `hostapd` / `udhcpc` / `lsusb` | 🟢 **全在** —— 厂商本来就打算支持无线 |
| `usbnet.ko` / `cdc_ether.ko` / `asix.ko` | 🟢 **在** —— USB 转以太网开箱可用 |
| `mac80211.ko`（软 MAC 栈）| 🔴 **没有** |
| 任何 WiFi 芯片驱动（`8188/8192/8821/8812/mt7/rtw/ath`）| 🔴 **零命中** |
| 板上内核头文件 | 🔴 `/lib/modules/$(uname -r)/build` 是**断链**，指向厂商编译机路径 |

### 选型判据

| 方案 | 驱动工作量 | 说明 |
|---|---|---|
| **USB 转以太网 + 迷你无线路由器（client 桥接）** | 🟢 **零** | `asix`/`cdc_ether` 现成。代价：多一个盒子和一路供电 |
| **USB WiFi 网卡** | 🔴 **大** | 必须是 **full-MAC 且只依赖 cfg80211** 的芯片（Realtek 厂商驱动 rtl8188eu / rtl8821cu / rtl8812au 属这类）；要向 D-Robotics 要内核源码在 PC 上交叉编译 `.ko`。in-tree 的 `ath9k_htc` / `mt76` **用不了**（缺 mac80211）|

**建议先走第一条**：它把「host 模式起不起得来」和「WiFi 驱动编不编得出来」两个未知**解耦**。host 起不来的话两条路都是死的，先用几十块的 asix 网卡把便宜的那个未知消掉。

## 5. 其它相关事实

- `eth0` 在 SoC 里存在（驱动 `hobot_gmac`），但 `operstate=down`，设备树里 `gmac-tsn` / `hobot_tsn` 节点标 `disabled`。**壳外有没有 RJ45 要看实物**；有的话连 USB 都不用折腾。
- `mmc1` 是 `horizon,x5-dwcmshc-sd`（SD 卡控制器，空），**不是 SDIO WiFi 插槽**。
- 带宽完全不是约束：实测 `/ws/planning` 2D **29.6 kB/s** / 3D 139.8 kB/s，加相机预览也远低于 5 Mbit/s。
- 🔴 **真正的风险是遥控看门狗**：`wheel_odometry_node` 的 `cmd_timeout_s = 0.5`，WiFi 连续丢包超过 0.6 s 轮子就停。上线前必须做 30 分钟 soak（`ping -i 0.1`，判据是连续丢包不超过 5 个）。

## 6. 方法论教训

> **`/sys/bus/usb/devices/` 为空只说明「现在没有 host」，不说明「不能有 host」。**
> 判断能力要看 `dr_mode`、`usb_role` 开关和 `xhci-hcd` 驱动在不在，
> 而不是看当前有没有枚举出设备。这和 `servo_bus.md` 里那条
> 「一个在所有条件下都不变的量不能算已排除，只能算没测过」是同一类错误。
