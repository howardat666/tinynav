# 修复 64GB Looper 的 MIPI 故障 —— 可执行流程

> # ✅ 已于 2026-08-03 修复（用 Plan B，只改了一个 JSON 字段）
>
> ```
> /userdata/install/share/insight_full/config/user_params.json
> -  "stereo_sensor_name": "sc132gs-1088x1280-20fps-2lane"
> +  "stereo_sensor_name": "sc132gs-1088x1280-20fps-1lane"
> ```
> 原文件备份在设备 `/userdata/fixbak/user_params.json.orig`（md5 `a628cf82…`）。**没有替换任何固件二进制。**
>
> **修复后实测**：
>
> | 项 | 修复前 | 修复后 |
> |---|---|---|
> | `insight_full` | SIGABRT，systemd 重试 5 次放弃 | 🟢 active，20+ 分钟无崩溃 |
> | `sif2` / `sif3`（立体） | 0 / 0 | 🟢 **20 / 20 fps** |
> | `sif0`（RGB imx415） | **从未初始化** | 🟢 **30 fps** |
> | `PHY_STOPSTATE` host2/host3 | `0x1000d` / `0x10001` | 🟢 **`0xd` / `0x1`**（bit16=0 → HS 时钟在跑） |
> | `N_LANES` | `0x1`（2 lane，与 sensor 矛盾） | 🟢 `0x0`（1 lane，与 sensor 一致） |
> | MIPI 错误计数 | 全 0 | 🟢 全 0 |
> | `infra1` / `infra2` 话题 | 无 | 🟢 19.9 / 20.0 Hz · `544x640 mono8` · mean 97 |
> | `depth` 话题 | 无 | 🟢 12.8 Hz · `544x640 mono16` · 67–65535 mm，中位 315 |
> | **`color` 话题（RGB）** | **无** | 🟢 **30.1 Hz · JPEG ~76 KB** |
> | `imu` / `vio_100hz` / `vio_image` | 无 | 🟢 382 Hz / 99.0 Hz / 20.0 Hz |
> | 内存 | — | `MemAvailable` 919 MB，`insight_full` RSS 218 MB / 178% CPU / 61 线程 |
> | BPU / 温度 | — | 85% / **69 °C**（此前 48 °C 是相机没跑时测的）|
>
> ⭐ **附带解决了 T-21**：RGB（imx415）第一次出图 —— `vin sensor2: sensor_start imx415 flow2 start done`，`mipi0` 4 lane 1944x1081 30 fps。之前 `insight_full` 死在第一路立体相机上，RGB 根本没走到。
>
> 🔴 **遗留风险（见 § 10）**：`user_params.json` 是 OTA 从包里安装的文件，**下次刷 OTA 会把它覆盖回 2lane，修复会被撤销**。

> 故障诊断全文见 [`x5.md § 2.2`](x5.md)。本文只讲**怎么修**。
> 目标机：64GB 改装版，`soc_uid = 0x328152560a164e920256c08a00120040`
> 参照机：桌面 B，`soc_uid = 0x328152560a164b9402a4408a00120040`（三路相机全正常）

---

## 0. 一句话结论

**`/usr/hobot/lib/sensor/libsc132gs.so.1.0.0` 是 2026-04 旧 OTA 包的残留，它对所有模式只写同一张 1-lane 寄存器表；而 CSI host 被配成 2 lane，于是永远等不到 D1 进入 LP-11。**

⬤ **2026-08-03 在故障机上已定案**：`strings … | grep -c 2lane` = **0**，md5 `f171ab12…`，46768 B，库里的模式名只有 `_30fps_setting_slave` / `_60fps_setting_master`，**一个 `_2lane_` 都没有**。

### ⭐ 两种修法，**推荐先试第二种**

| | Plan A：换库 | **Plan B：改配置（推荐先试）** |
|---|---|---|
| 做什么 | 把文件换成正常机那份（55848 B / `357bfde5…`）| `user_params.json` 里 `stereo_sensor_name` 从 `sc132gs-1088x1280-20fps-2lane` 改成 **`-1lane`** |
| 改动范围 | 覆盖一个固件二进制 | **一个 JSON 字段** |
| 可回滚 | 需要备份原文件 | 🟢 改回来就行 |
| 结果 | 恢复原厂 2-lane 20 fps | 1-lane 20 fps（**带宽绰绰有余**，见下） |

**Plan B 为什么几乎一定能成**（依据 LooperHub 源码，2026-08-03 核对）：

1. `sc132gs-1088x1280-20fps-1lane` 是**已注册的一等模式**（`tros_ws/src/insight_full/sensor/camera/vp_sensors.c:33`），不是 hack
2. **代码里编译进去的默认值本来就是 1lane** —— `insight_full_node.cpp:566` `userParamStr("stereo_sensor_name", "sc132gs-1088x1280-20fps-1lane")`，`insight_full/readme.md:52` 的示例也写的是 1lane。**是这台的 `user_params.json` 把它覆盖成了 2lane**
3. 两个模式的定义**只差两个字段**：
   ```c
   // sc132gs_linear_1088x1280_raw10_20fps_1lane.c        _2lane.c
   .lane = 1,                                          .lane = 2,
   .config_index = 0, //1：2lane 0：1lane               .config_index = 1,
   ```
   `config_index` 就是**选 BSP 库里哪张寄存器表**。坏库只有 1-lane 那一张 → 请求 `config_index=1` 拿不到 2-lane 表 → **host 配 2 lane 而 sensor 被写成 1 lane** → 正是我们看到的 `PHY_STOPSTATE=0x1000d`
4. 改成 1lane 后 host 也走 1 lane、`config_index=0`，**和坏库唯一拥有的那张表完全对上**
5. 已有实测支撑：`multi_isp_vflow -s 1`（1 lane）下两路立体相机都跑满 **~62 fps、MIPI 错误计数器全 0**，而我们只要 20 fps

> ⚠️ 注意：**寄存器表不在 LooperHub 里**（`grep 0x3018 tros_ws/src/insight_full/sensor/` 为空，全仓库无 `libsc132gs` 字样）。LooperHub 只管 host 侧的模式配置，寄存器表属于 D-Robotics BSP 的 `.so`。所以 Plan A 那份正确的库只能来自正常机或整包镜像，**不能靠重编 LooperHub 得到**。

### 🟢 修复不会被 OTA 撤销（已核实）

| 检查 | 结果 |
|---|---|
| 待装 `update_v2_1_6cmsibv830.2.bundle` 里有 `libsc132gs` 吗 | **0 命中** |
| 全盘还有别的同 md5 副本吗 | 只有目标文件自己（`deps/` 那个是不同 md5 且不会被加载）|
| 有开机脚本往 `/usr/hobot` 拷文件吗 | **无** |
| `libsc132gs.so` / `.so.1` 软链接 | **May 21 2026 的 BSP 原件**，只有 `.so.1.0.0` 目标被换过 |

> ⚠️ 别被 mtime 骗了：那个文件显示 `Aug 26 23:26`，看着像"本次开机写的"，但 **RTC 每次开机都固定回到 `2025-08-26 23:23`**，所以任何一次开机后 3 分钟写的文件都长这样。加上权限是 `0644`（同目录其他 `.so` 全是 `0775`）→ **一次性手工拷贝，不是每次开机刷。**

```
sensor 被写成 1 lane          host 配成 2 lane
0x3018=0x12 (lane_num-1=0)  ← 不匹配 →  N_LANES=0x1
0x3019=0x0e (禁用 D1/D2/D3)            等 D1 stop state
                                        ↓
                        PHY_STOPSTATE=0x1000d，init error: -1
                        insight_full → CreateAndRunVflow failed, ret=-10 → SIGABRT
```

### 🔴 为什么 LooperHub 的 OTA 修不了这个

**OTA 2.1.2（2026-07-17）把 `libsc132gs.so.1.0.0` 从 payload 里删掉了**（`Bin 46768 -> 0 bytes`，`sc132gs_tuning.json` 同时删）。
而 `/userdata/postinst` 只做 `cp -a "$INSTALL_DIR/usr/hobot/"* /usr/hobot/` —— **从不删文件**。

> 所以：**装任何 ≥2.1.2 的 OTA 都不会覆盖这个文件，旧库永远留着。**
> LooperHub 只有两种做法能修：
> 1. **给一个整包烧写镜像**（重写 `mmcblk0p13` = `/usr/hobot`）—— 最干净，推荐向他们要这个
> 2. 让他们确认那份 55848 B 的库是 20260717 包的原件，然后**手工替换**（就是下面的流程）
>
> ⚠️ 另外 `/userdata/ota_packet/setting/` 里躺着 `update_v2_1_6cmsibv830.2.bundle` + `downloadfinish`，但已装版本写的是 2.1.2。**装它之前必须先确认它的 payload 里有没有 `libsc132gs.so`** —— 如果它把旧版又刷回来，修复会被撤销（步骤 1 有检查命令）。

---

## 1. 只读定案（先跑，不改任何东西）

```bash
D=root@169.254.10.1          # 密码 looper@0731

# ① 确认是哪台机器（必须是 ...0a164e920256c08a00120040）
sshpass -p looper@0731 ssh $D 'cat /sys/class/socinfo/soc_uid; df -h /userdata | tail -1'

# ② 🔑 决定性判据：这个库里有没有 2lane 表
sshpass -p looper@0731 ssh $D \
  'F=/usr/hobot/lib/sensor/libsc132gs.so.1.0.0; ls -l $F; md5sum $F; strings $F | grep -c 2lane'
#   期望（故障）：46768 字节 / f171ab12… / 2lane 计数 = 0   → 100% 定案
#   若为        ：55848 字节 / 357bfde5… / 2lane 计数 = 7   → 库是对的，故障另有原因，停下重新诊断

# ③ 确认实际被 mmap 的就是这个文件（而不是 deps 里那个）
sshpass -p looper@0731 ssh $D \
  'grep -o "/[^ ]*libsc132gs[^ ]*" /proc/$(pidof insight_full)/maps 2>/dev/null | sort -u'
#   insight_full 可能已经崩掉，那就跳过这步（x5.md 已在正常机上证实过加载路径）

# ④ 待装的 2.1.6 bundle 里有没有这个库（决定修完会不会被撤销）
sshpass -p looper@0731 ssh $D \
  'ls -la /userdata/ota_packet/setting/; strings /userdata/ota_packet/setting/*.bundle | grep -c libsc132gs'

# ⑤ 把故障库拿回 PC 做符号级 diff（闭合最后一个证据缺口）
sshpass -p looper@0731 scp $D:/usr/hobot/lib/sensor/libsc132gs.so.1.0.0 \
  /home/dm/looper/looper_device_files/libsc132gs.so.1.0.0.BAD_from_64gb
readelf -sW /home/dm/looper/looper_device_files/libsc132gs.so.1.0.0.BAD_from_64gb | grep -i setting
#   期望：符号表里只有 1lane / 不带 2lane 的表
```

PC 上参照库的实测（已完成）：

```
$ readelf -sW firmware_ref_from_working_device/libsc132gs.so.1.0.0 | grep -i setting
sc132gs_linear_init_1088x1280_30fps_1lane_setting_slave    1040
sc132gs_linear_init_1088x1280_30fps_2lane_setting_slave    1040   ← 故障库应缺这些
sc132gs_linear_init_1088x1280_60fps_2lane_setting_slave    1016   ←
sc132gs_linear_init_1088x1280_60fps_setting_master_2lane    960   ←
sc132gs_linear_init_1088x1280_60fps_setting_master          952
...
$ strings ... | grep -c 2lane
7
```

---

## 2. 备份（必做，这是唯一的回滚路径）

```bash
sshpass -p looper@0731 ssh $D 'set -e
  mkdir -p /userdata/fixbak
  cp -a /usr/hobot/lib/sensor/libsc132gs.so.1.0.0 \
        /userdata/fixbak/libsc132gs.so.1.0.0.orig
  ls -l /userdata/fixbak/; md5sum /userdata/fixbak/*'
```

`/userdata` 有 44 G 空余，放得下。**备份确认成功后再做第 3 步。**

---

## 3. 安装正确的库

```bash
# 传上去（先落到 /userdata，不直接覆盖）
sshpass -p looper@0731 scp \
  /home/dm/looper/looper_device_files/firmware_ref_from_working_device/libsc132gs.so.1.0.0 \
  $D:/userdata/fixbak/libsc132gs.so.1.0.0.good

# 校验传输 + 原子替换 + 恢复正确权限（正常机上该目录所有 .so 都是 0775）
sshpass -p looper@0731 ssh $D 'set -e
  cd /userdata/fixbak
  md5sum libsc132gs.so.1.0.0.good           # 必须 = 357bfde527a303fa097898fad02e6fbe
  install -m 0775 -o root -g root libsc132gs.so.1.0.0.good \
          /usr/hobot/lib/sensor/libsc132gs.so.1.0.0
  ls -l /usr/hobot/lib/sensor/libsc132gs.so.1.0.0
  md5sum /usr/hobot/lib/sensor/libsc132gs.so.1.0.0
  sync'
```

## 4. 重启相机固件

```bash
sshpass -p looper@0731 ssh $D '/etc/init.d/ota_project/scripts/insight-ctl restart' \
  || sshpass -p looper@0731 ssh $D 'systemctl restart S99all_run.service'
```

systemd 之前因 `Start request repeated too quickly` 放弃过，可能需要先 `systemctl reset-failed S99all_run.service`。

## 5. 验证（四条，逐级递进）

```bash
# ① 内核日志里 lane 报错应当消失，且 sensor_start 成功
sshpass -p looper@0731 ssh $D 'dmesg | grep -E "mipi[23]|sensor_start|lane state" | tail -20'
#   修好：vin mipi2: 2 lane 1088x1280 20fps ... / sensor_start sc132gs flow0 start done
#   没修好：仍然 lane state of host phy is error: 0x1000d

# ② PHY stop state：0x1000d → 0xf / 0x3
sshpass -p looper@0731 ssh $D \
  'for h in 2 3; do echo "--- host$h"; cat /sys/class/vps/mipi_host$h/status/regs | grep -i "stopstate\|N_LANES"; done'

# ③ 活体读 sensor 寄存器：应命中 2lane 表
sshpass -p looper@0731 ssh $D \
  'i2ctransfer -y -f 4 w2@0x32 0x30 0x18 r1; i2ctransfer -y -f 4 w2@0x32 0x30 0x1f r1'
#   期望 0x3018 = 0x22 或 0x32（≠0x12）· 0x301f = 0x9c（2lane 表）

# ④ 真实帧率 + ROS 话题
sshpass -p looper@0731 ssh $D 'cat /sys/kernel/debug/sif2/fps /sys/kernel/debug/sif3/fps'
sshpass -p looper@0731 ssh $D \
  'source /etc/init.d/looper/setting/ros2_env.conf; ros2 topic hz /camera/camera/infra1/image_rect_raw'
#   ⚠️ 这台机器上 ros2 CLI 报 PackageNotFoundError: ros2cli（x5.md § 12），
#      话题验证改在 PC 端做：ros2 topic hz（PC 与相机同网段）
```

## 6. 回滚

```bash
sshpass -p looper@0731 ssh $D \
  'install -m 0775 /userdata/fixbak/libsc132gs.so.1.0.0.orig \
      /usr/hobot/lib/sensor/libsc132gs.so.1.0.0 && sync && \
   /etc/init.d/ota_project/scripts/insight-ctl restart'
```

---

## 7. 如果换库没修好 —— 下一个假设

根因排序里的 2️⃣：**这台的立体模组本身就是 1-lane 变体**（改装时换了模组），而配置写的是 2-lane。

那就不改库，改配置 —— 把 host 也配成 1 lane：

```bash
# /userdata/install/share/insight_full/config/user_params.json
#   stereo_sensor_name: "sc132gs-1088x1280-20fps-2lane"
#                    → "sc132gs-1088x1280-20fps-1lane"    ← 若库里有这个模式名
```

**这条路完全可行**：`multi_isp_vflow -s 1` 已实测 **1 lane 下两路都跑满 ~62 fps、MIPI 错误计数器全 0**，而我们只要 20 fps —— **1 lane 的带宽绰绰有余**。改配置比改库更保守，**如果你想先试风险最低的一条，就先试这个。**

改之前同样先备份 `user_params.json`，并确认库里存在对应的模式名：

```bash
sshpass -p looper@0731 ssh $D 'strings /usr/hobot/lib/sensor/libsc132gs.so.1.0.0 | grep -i "1088x1280.*lane"'
```

---

## 8. ⚠️ 必须一起处理：RTC 纽扣电池

`hwclock -r` 返回 1970，系统时间每次开机固定回到 **2025-08-26 23:23**。后果：

- `insight_full_crash_YYYYMMDD_HHMMSS.log` 文件名反复撞车互相覆盖 → **崩溃证据被持续销毁**
- `journalctl` 只有 boot 0，无持久 journal，`/app/startup.sh` 每次开机还 `rm -rf /var/log/*`
- **很可能让 OTA 的版本/时间判断失效，每次开机重装旧包**

**修 RTC 电池应该和换库一起做**，否则我们既看不到失败日志，也不能保证修复不被 OTA 撤销。
（⚠️ 但这需要开壳 —— 之前的约定是这台不开壳。要开的话得你先同意。）

---

## 9. 修完之后紧接着要做的（都还没验证过）

| # | 事项 | 为什么现在做 |
|---|---|---|
| 1 | **RGB（imx415）出图** | 之前 `insight_full` 死在第一路立体相机上，RGB 根本没走到（`mipi_host0` 全 `--------`）。修好后第一次有机会验证 |
| 2 | `ros2` CLI 坏掉（`PackageNotFoundError: ros2cli`）| 这台机器上 `source ros2_env.conf` 那招不管用，板上验证话题要靠它 |
| 3 | **T-20：让 Looper 减小 ion 预留** | 物理 DRAM 3.9 GiB，~2.5 GiB 被 ion 占着，`MemTotal` 只剩 1307 MB。**这是分配决策而非硬件上限** —— 现在有 LooperHub 权限了，这是最该提的一个需求，收益比任何算法优化都大 |
| 4 | 遥控器 IR 测试 | 决定夜间能否用红外补光，1 分钟（见 `x5.md § 10.3`）|

---

## 10. 🔴 遗留风险：下次 OTA 会撤销这个修复

Plan B 改的是 `/userdata/install/share/insight_full/config/user_params.json`，而**这个文件是 OTA 从包里安装的**（LooperHub `tros_ws/src/insight_full/config/user_params.json` → 装到该路径）。`/userdata/postinst` 会重新铺一遍 `/userdata/install/`，**于是 `stereo_sensor_name` 被刷回 `2lane`，相机再次起不来。**

（对比：Plan A 换的 `/usr/hobot/lib/sensor/` 那个文件反而**不会**被撤销 —— OTA 2.1.2 已经把它从 payload 删掉了。两种修法的持久性正好相反。）

### 解法（三选一，推荐第 2 个）

| # | 做法 | 评价 |
|---|---|---|
| 1 | 每次 OTA 后手动再改一遍 | 🔴 会忘 |
| 2 | **在 LooperHub 里把 `config/user_params.json` 的 `stereo_sensor_name` 改成 1lane（或直接删掉这个 key，让它回落到编译进去的默认值 —— 默认本来就是 1lane）**，随下一个包一起发 | 🟢 **一劳永逸**。而且我们本来就要为「depth 降到 5 Hz」重编一版，**顺手一起改，不额外增加工作** |
| 3 | 走 Plan A 换库，恢复原厂 2lane | 🟡 也持久，但要覆盖固件二进制，且这台机器 1 lane 已够用（20 fps 只需 1 lane 的一小部分带宽）|

⚠️ **所以「重编固件」这件事必须把这个修复一起带进去，否则新包一刷就退回故障状态。**
