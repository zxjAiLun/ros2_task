# 多传感器融合与状态监控系统（ROS 2 Jazzy, rclpy）

## 环境与依赖

- ROS 2 Jazzy
- Python 3
- 已安装 `rclpy`、`message_filters` 等 ROS 2 依赖（通过 `rosdep install --from-paths src -y` 推荐）

## 工作空间构成

- `multisensor_interfaces`：自定义消息与服务（`VisionMsg` / `AudioMsg` / `LidarMsg` / `FusionMsg` / `NodeStatusMsg` / `AlertMsg`）
- `multisensor_sensors`：Python 版 `vision_node` / `audio_node` / `lidar_node`，发布：
  - `/vision/detection` (`VisionMsg`)
  - `/audio/source_angle` (`AudioMsg`)
  - `/lidar/scan` (`LidarMsg`)
- `multisensor_fusion`：`fusion_node`，订阅上述三个话题并输出 `/fusion/result` (`FusionMsg`)
- `multisensor_monitor`：`status_monitor`，订阅 `/vision/detection`、`/audio/source_angle`、`/lidar/scan`、`/fusion/result`，发布：
  - `/system/status` (`NodeStatusMsg`)
  - `/system/alert` (`AlertMsg`)
- `multisensor_bringup`：Launch 与 YAML 参数配置
- `multisensor_sensors_cpp`：C++ 版 `vision_node_cpp` / `audio_node_cpp` / `lidar_node_cpp`，与 Python 一致地发布 `/vision/detection`、`/audio/source_angle`、`/lidar/scan`
- `multisensor_fusion_cpp`：C++ 版 `fusion_node_cpp`，订阅 `/vision/detection`、`/audio/source_angle`、`/lidar/scan` 并输出 `/fusion/result`
- `multisensor_monitor_cpp`：C++ 版 `status_monitor_cpp`，监控上述话题并发布 `/system/status_cpp` / `/system/alert_cpp`

## 构建

```bash
cd ros2_task_ws
colcon build
source install/setup.bash
```

## 运行

- 一键启动全系统：

```bash
ros2 launch multisensor_bringup bringup_all.launch.py        # Python 版本
ros2 launch multisensor_bringup bringup_all_cpp.launch.py    # C++ 版本
```

- 单独启动示例（需先 `source install/setup.bash`）：

```bash
# Python 版本
ros2 launch multisensor_bringup sensors.launch.py
ros2 launch multisensor_bringup fusion.launch.py
ros2 launch multisensor_bringup monitor.launch.py

# C++ 版本
ros2 launch multisensor_bringup sensors_cpp.launch.py
ros2 launch multisensor_bringup fusion_cpp.launch.py
ros2 launch multisensor_bringup monitor_cpp.launch.py
```

## Python 与 C++ 行为对比建议

- 只启动 Python 版或 C++ 版其中一套，避免同一话题上多源干扰。
- 使用 `ros2 topic echo` / `rqt_plot` 对比：
  - Python `/vision/data` 与 C++ `/vision_cpp/data` 的检测概率、角度与置信度分布是否接近。
  - Python `/fusion/result` 与 C++ `/fusion/result_cpp` 的 `fusion_status`、`fusion_confidence` 在典型场景下是否一致。
- 在 C++ 版本中同样可通过停止某个 C++ 传感器节点 ≥ 3 秒，观察 `/system/status_cpp` 与 `/system/alert_cpp` 行为是否符合预期。

## QoS 策略与共享内存

- 传感器话题（`/vision/detection`、`/audio/source_angle`、`/lidar/scan`）：使用 BestEffort + KeepLast，depth 默认 10，侧重实时性。
- 融合结果与系统状态（`/fusion/result`、`/system/status`、`/system/alert` 以及 C++ 版 `/system/status_cpp`、`/system/alert_cpp`）：使用 Reliable + KeepLast，depth 默认 10，避免关键结果丢失。
- 可在 `multisensor_bringup/config/*.yaml` 中修改 QoS 相关参数（如 `qos.reliability`、`qos.depth`）。

**共享内存说明**：

- 建议使用 CycloneDDS 或 FastDDS，并在对应 XML 配置中启用共享内存传输。
- 通过环境变量选择 RMW 实现，例如：

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 launch multisensor_bringup bringup_all.launch.py
```

## Fusion 决策规则（示例）

| Vision.detected | Audio.voice_detected | |angle_v - angle_a| ≤ 阈值 | fusion_status | 说明 |
|-----------------|----------------------|-----------------------------|---------------|------|
| False           | False                | -                           | none          | 无目标 |
| True            | False                | -                           | vision_only   | 仅视觉检测到 |
| False           | True                 | -                           | audio_only    | 仅音频检测到 |
| True            | True                 | True                        | both          | 方向一致，多源确认 |
| True            | True                 | False                       | conflict      | 方向冲突 |

## 测试与可视化建议

- 使用 `rqt_graph` 检查话题连通性。
- 使用 `rqt_plot` 观察 `/fusion/result/fusion_confidence`、`/fusion/result/target_angle` 与 `/system/status/last_update_age`。
- 模拟断线测试：在全系统运行时停止任一传感器节点 ≥ 3 秒，观察 `/system/alert` 是否产生 WARN 告警。

## 调整监控参数的 Action 接口

- Action 类型：`multisensor_interfaces/action/AdjustNodeParams`
- 话题名：`/system/adjust_node_params`
- 作用：替代原来的 `SetMonitorParam.srv`，用于在运行时调整 `status_monitor` / `status_monitor_cpp` 的监控参数，并返回执行进度与结果。

**Goal 字段**：

- `string key`：参数名（支持 `timeout_sec`、`status_publish_period`、`alert_cooldown_sec`）
- `float32 value`：对应的新数值

**Result 字段**：

- `bool success`：是否更新成功
- `string message`：结果说明（例如 `Updated timeout_sec` 或错误原因）

**Feedback 字段**：

- `float32 progress`：执行进度 0.0–1.0
- `string current_state`：当前状态（如 `validating` / `applying` / `done` / `failed`）

### 使用示例（Python / C++ 通用）

在任意终端中先进入工作空间并 source 安装环境：

```bash
cd ros2_task_ws
source install/setup.bash
```

调整监控超时时间为 5 秒：

```bash
ros2 action send_goal /system/adjust_node_params \
  multisensor_interfaces/action/AdjustNodeParams \
  "{key: 'timeout_sec', value: 5.0}"
```

调整状态发布周期为 0.5 秒：

```bash
ros2 action send_goal /system/adjust_node_params \
  multisensor_interfaces/action/AdjustNodeParams \
  "{key: 'status_publish_period', value: 0.5}"
```

在 Python 版和 C++ 版 bringup（`bringup_all.launch.py` / `bringup_all_cpp.launch.py`）运行时均可使用该 Action，`status_monitor` 与 `status_monitor_cpp` 会分别作为 Action Server 响应请求。

## 通信延迟测量示例

- 可以在订阅端计算 `now - msg.header.stamp` 的时间差，估计端到端传输延迟，并在日志或自定义 Topic 中统计。
- 可在不同 QoS 和 RMW 配置下重复测量，记录典型延迟数据作为实验结果。

