# 多传感器融合与状态监控系统（ROS 2 Jazzy, rclpy）

## 环境与依赖

- ROS 2 Jazzy
- Python 3
- 已安装 `rclpy`、`message_filters` 等 ROS 2 依赖（通过 `rosdep install --from-paths src -y` 推荐）

## 工作空间构成

- `multisensor_interfaces`：自定义消息与服务
- `multisensor_sensors`：`vision_node` / `audio_node` / `lidar_node` 模拟传感器
- `multisensor_fusion`：`fusion_node`，基于 `message_filters` 时间同步与规则融合
- `multisensor_monitor`：`status_monitor` 状态监控与告警
- `multisensor_bringup`：Launch 与 YAML 参数配置
- `multisensor_sensors_cpp`：C++ 版 `vision_node_cpp` / `audio_node_cpp` / `lidar_node_cpp`
- `multisensor_fusion_cpp`：C++ 版 `fusion_node_cpp`，与 C++ 传感器对接输出 `/fusion/result_cpp`
- `multisensor_monitor_cpp`：C++ 版 `status_monitor_cpp`，监控 C++ 话题并发布 `/system/status_cpp` / `/system/alert_cpp`

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

- 传感器话题（`/vision/data`、`/audio/data`、`/lidar/data`）：使用 BestEffort + KeepLast，depth 默认 10，侧重实时性。
- 融合结果与系统状态（`/fusion/result`、`/system/status`、`/system/alert`）：使用 Reliable + KeepLast，depth 默认 10，避免关键结果丢失。
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

## 通信延迟测量示例

- 可以在订阅端计算 `now - msg.header.stamp` 的时间差，估计端到端传输延迟，并在日志或自定义 Topic 中统计。
- 可在不同 QoS 和 RMW 配置下重复测量，记录典型延迟数据作为实验结果。

