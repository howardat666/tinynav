# 图像数据流：17.3 MB/s 的回环流量，信息量只有 4.3 MB/s

2026-08-24 实测。app 在跑、nav 未开、板子 load 8.01/8 核。

## 测到的

```
lo 回环          收 17.30 MB/s   发 17.30 MB/s
/dev/shm 总量    5.1 MB，最大段 549408 B ≈ 536 KB
UDP              2663330 收包，23157 收包错误 = 0.87%
depth            4.305 Hz   640×544 mono16 = 680 KB/帧
infra1          20.235 Hz   640×544 mono8  = 340 KB/帧
```

账对得上：

| 项 | 计算 | MB/s |
|---|---|---|
| depth × **2 个订阅者** | 680 KB × 4.305 × 2 | 5.72 |
| infra1 × 1 | 340 KB × 20.235 | 6.72 |
| `/slam/depth` 转发 | 680 KB × 4.305 | 2.86 |
| 小计 | | **15.30** |
| 实测 | | **17.30**（差额是 100 Hz 位姿 ×3、camera_info、tf、app 状态推送） |

**真实信息量**只有 depth 4.3 Hz + infra1 配得上的那 4.3 Hz = **4.3 MB/s。开销 4 倍。**

## 🔴 深度图不可能走共享内存

`/dev/shm` 里最大的段是 **536 KB**，深度图是 **680 KB**。所以它只能退回 UDP 回环，在那里被拆成
**11 个数据包**（64 KB MTU）。丢任何一个，整帧就废：

```
P(整帧丢失) = 1 - (1 - 0.0087)^11 ≈ 9.2%
```

这和消息内容、和我们的代码都无关 —— 纯粹是"消息比段大"这一个事实的后果。

## 三处可定位的浪费

### 1. depth 被订阅了两次

实测 `ros2 topic info` → `Subscription count: 2`。代码里是 `looper_bridge_node.py:111`
（直接订阅，发 `/slam/depth`）和 `:117`（给 message_filters 再订阅一遍）。

**两个 QoS 不同**（`fast_depth_qos` vs `sync_depth_qos`），所以不能简单删一个。做法是留一条订阅，
在回调里用 `SimpleFilter.signalMessage` 手动喂给同步器。

### 2. infra1 的 78% 是收进来就扔

infra1 发 20.2 Hz，depth 只有 4.3 Hz。精确时间戳同步器只能配上有 depth 的那些帧，所以
约 **15.9 Hz × 340 KB = 5.3 MB/s** 被收进来、反序列化、然后丢掉。

**这是最大的一块，而且最该在固件侧解决** —— 不发比发了再扔便宜。固件参数我们改得动
（`vio_enabled` / `depth_frame_skip` 都改过，见 `looperhub-is-the-firmware-source`）。

### 3. 关键帧话题没人订阅

nav 未开时 `/slam/keyframe_image`、`/slam/keyframe_depth` 实测 `Subscription count: 0`。
发布端已经按 `get_subscription_count() > 0` 门控了，所以这一条**已经不花钱**。

## ⚠️ 两个已经做完 / 差点误判的

- **计划文件里的 B2 和 B4 已经做完了** —— `sync_callback` 里的 `want_keyframe_depth` 和
  keyframe-only 重构都在代码里了。别重复做。
- **`sync_callback` 那条日志是按秒节流的**（`stamp_s - self._last_sync_log_stamp >= 1.0`）。
  386 行日志 / 425 秒 **不等于** 0.91 Hz —— 它其实跟着 depth 跑 4.3 Hz。我差点据此断定
  "精确时间戳同步只配上了 21%"，查了代码才发现是节流。

## 为什么不能靠"合并进程"一步解决

**rclpy 没有 intra-process 零拷贝。** rclcpp 的 component 可以在同进程内传指针，rclpy 每次
publish 都要过中间件。而 `sensor_msgs/Image` 含变长数组，也用不了 loaned message 零拷贝
（那要求固定大小的消息类型）。

所以"把 bridge 和 planning 合成一个进程"在 Python 下**只有真的绕开 ROS**（直接传 numpy 数组）
才有收益，代价是两者不能再独立重启。

## 按性价比排序的下一步

| # | 动作 | 预期收益 | 风险 |
|---|---|---|---|
| 1 | 加 FastDDS profile，把 SHM `segment_size` 提到 4 MB | 深度图离开内核网络栈；9.2% 的分片丢帧应归零 | 一个 XML + 一个环境变量，零代码。**是假设，判据是 lo 流量该掉** |
| 2 | 固件侧把 infra1 降到 5 Hz | −5.3 MB/s（最大一块） | 预览会变成 5 Hz；VIO 在固件内自己的流上，不受影响 |
| 3 | 合并重复的 depth 订阅 | −2.86 MB/s + 每帧少一次 680 KB 反序列化 | 要用 `signalMessage`，改动在 bridge 里 |

1 应该先做：零代码，而且如果假设对，它同时解决流量和丢帧两件事。判据很干净 ——
**lo 的 MB/s 应该掉下来**，`/dev/shm` 应该出现大段。
