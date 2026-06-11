# TinyNav RTK Bridge

这个脚本把 RTK 第一阶段需要的功能合到一个 ROS 2 节点里：

- 连接 NTRIP caster，接收 RTCM 差分数据
- 把 RTCM 写入 RTK 接收机串口
- 从同一个串口读取 NMEA
- 发布 `/fix`、`/heading`、`/vel`、`/time_reference`
- 同时发布 TinyNav 方便使用的 `/rtk/odom`、`/rtk/path`、`/rtk/status`

默认串口：`/dev/ttyCH341USB0`，默认波特率：`115200`。

## 运行

不要再单独运行 `str2str` 和 `nmea_navsat_driver`，避免多个进程同时打开同一个串口。

先在 Jetson Docker 容器里配置 NTRIP 账号和密码，避免把凭据写进代码：

```bash
export TINYNAV_NTRIP_USER="your-user"
export TINYNAV_NTRIP_PASSWORD="your-password"
```

caster host、端口、mountpoint 和 initial GGA 有默认值；如果现场要换，也可以继续用环境变量覆盖：

```bash
export TINYNAV_NTRIP_HOST="your-caster-host"
export TINYNAV_NTRIP_PORT="2101"
export TINYNAV_NTRIP_MOUNTPOINT="your-mountpoint"
export TINYNAV_NTRIP_INITIAL_GGA="your-gga-sentence"
```

```bash
sudo uv run python /tinynav/rtk/rtk_bridge_node.py
```

如果串口权限已经配置好，也可以不用 `sudo`：

```bash
uv run python /tinynav/rtk/rtk_bridge_node.py
```

## 查看原始 NMEA

脚本默认创建一个伪串口镜像：

```bash
cat /tmp/rtk_nmea
```

也可以用 ROS topic 看：

```bash
ros2 topic echo /rtk/nmea_sentence
```

## 主要输出

底层兼容话题：

- `/fix`: `sensor_msgs/NavSatFix`，经纬高和定位状态
- `/heading`: `geometry_msgs/QuaternionStamped`，由 NMEA `HDT/THS` 转成 ENU yaw
- `/vel`: `geometry_msgs/TwistStamped`，由 NMEA `RMC` 的 speed/course 转成 ENU 速度
- `/time_reference`: `sensor_msgs/TimeReference`，GNSS UTC 时间

TinyNav 调试/融合话题：

- `/rtk/odom`: `nav_msgs/Odometry`，本地 ENU 坐标系下的位置、朝向、速度
- `/rtk/path`: `nav_msgs/Path`，RTK 轨迹，方便 RViz 显示
- `/rtk/status`: `std_msgs/String`，JSON 状态，包含 fix 是否 accepted、heading/velocity 是否 ready、ENU 位置等

默认 ENU 坐标：

- `x`: East
- `y`: North
- `z`: Up

默认第一次有效 `/fix` 会作为 ENU 原点。

## 常用参数

固定 ENU 原点：

```bash
sudo uv run python /tinynav/rtk/rtk_bridge_node.py --ros-args \
  -p origin_lat:=22.78156952 \
  -p origin_lon:=113.5139155455 \
  -p origin_alt:=-1.1718
```

换串口：

```bash
sudo uv run python /tinynav/rtk/rtk_bridge_node.py --ros-args \
  -p serial_port:=/dev/ttyCH341USB0 \
  -p baud:=115200
```

只解析串口，不连接 NTRIP：

```bash
sudo uv run python /tinynav/rtk/rtk_bridge_node.py --ros-args -p ntrip_enabled:=false
```

也可以用 ROS 参数临时传入 NTRIP 信息。账号和密码建议每次手动传，其他参数不传就使用默认值：

```bash
sudo uv run python /tinynav/rtk/rtk_bridge_node.py --ros-args \
  -p ntrip_user:=your-user \
  -p ntrip_password:=your-password
```

只做 NMEA/RTK 话题发布，不使用 heading 进 `/rtk/odom`：

```bash
sudo uv run python /tinynav/rtk/rtk_bridge_node.py --ros-args -p use_heading:=false
```

## 验证

```bash
ros2 topic echo /fix
ros2 topic echo /rtk/status
ros2 topic echo /rtk/odom
cat /tmp/rtk_nmea
```

如果 `/rtk/status` 里 `accepted=true`，且 `/rtk/odom` 的位置在推车移动时以米为单位变化，就说明第一阶段链路正常。
