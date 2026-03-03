---
name: ros2_multisensor_cpp_nodes
overview: 在现有基于 rclpy 的多传感器融合与状态监控系统上，新增一组使用 rclcpp 的 C++ 节点包，实现与 Python 版本等价或互通的功能逻辑，并与当前工作空间共存构建。
todos:
  - id: cpp-create-packages
    content: 在 src 下创建 C++ 包 multisensor_sensors_cpp、multisensor_fusion_cpp、multisensor_monitor_cpp，并配置 package.xml 与 CMakeLists.txt 使其依赖 multisensor_interfaces 与 rclcpp
    status: pending
  - id: cpp-sensor-nodes
    content: 在 multisensor_sensors_cpp 中实现 vision/audio/lidar C++ 节点，逻辑与 Python 版一致（模拟数据 + 参数 + QoS），发布到独立或共用的话题
    status: pending
  - id: cpp-fusion-node
    content: 在 multisensor_fusion_cpp 中使用 message_filters C++ 版对多路 Topic 时间同步，实现与 Python 版等价的融合规则并发布 FusionResult
    status: pending
  - id: cpp-monitor-node
    content: 在 multisensor_monitor_cpp 中实现 C++ 状态监控节点，监控各 Topic 更新时间并通过 NodeStatus/SystemAlert 与 GetStatus/SetMonitorParam 提供相同接口
    status: pending
  - id: cpp-launch-config
    content: 在 multisensor_bringup 中新增 *_cpp.launch.py 与对应 YAML，支持 C++ 版本的单节点和一键启动
    status: pending
  - id: cpp-docs-tests
    content: 在 README 中补充 C++ 版本说明和运行示例，并设计简单对比测试（Python 与 C++ 节点输出与行为的一致性）
    status: pending
isProject: false
---

## 目标与思路

- **目标**：在当前 `ros2_task_ws` 中，保留现有 Python 节点不变，新增一套 **C++ 版本** 的传感器节点 / 融合节点 / 状态监控节点，实现相同 Topic 接口与大致相同的逻辑，支持与现有 msg/srv 共用，并能通过 Launch 单独或混合启动。
- **关键点**：
  - 继续复用现有 `multisensor_interfaces` 中的消息与服务定义（C++/Python 都可用）。
  - 新建 1~2 个 C++ 包（例如 `multisensor_sensors_cpp`、`multisensor_fusion_cpp`、`multisensor_monitor_cpp`），使用 `ament_cmake` 与 `rclcpp`。
  - QoS、参数、融合规则与 Python 版本保持一致或尽量相似，便于对比。

## 包结构规划

- 在现有 `src/` 目录下新增 C++ 包：
  - `[ros2_task_ws/src/multisensor_sensors_cpp/]`：C++ 版本 `vision_node_cpp` / `audio_node_cpp` / `lidar_node_cpp`，发布与 Python 相同消息与话题。
  - `[ros2_task_ws/src/multisensor_fusion_cpp/]`：C++ 版本 `fusion_node_cpp`，订阅三路传感器消息并输出 `/fusion/result_cpp` 或沿用 `/fusion/result`（根据需要决定是否区分）。
  - `[ros2_task_ws/src/multisensor_monitor_cpp/]`：C++ 版本 `status_monitor_cpp`，监控 Topic 更新时间并发布 `/system/status_cpp` 与 `/system/alert_cpp` 或复用原 Topic。
- 继续沿用现有 `multisensor_bringup` 包，新增或扩展 Launch 文件以支持 C++ 节点的启动：
  - `sensors_cpp.launch.py`、`fusion_cpp.launch.py`、`monitor_cpp.launch.py`、`bringup_all_cpp.launch.py`。

## 接口与逻辑对齐

- **接口复用**：
  - C++ 节点直接使用 `multisensor_interfaces::msg::Vision` / `Audio` / `Lidar` / `FusionResult`，以及 `NodeStatus` / `SystemAlert`，服务使用 `GetStatus` / `SetMonitorParam`。
- **话题命名策略**（建议）：
  - 若希望 C++与 Python 节点**不互相干扰**，可以为 C++ 版本添加后缀：
    - 传感器：`/vision_cpp/data`、`/audio_cpp/data`、`/lidar_cpp/data`。
    - 融合：`/fusion/result_cpp`。
    - 状态监控：`/system/status_cpp`、`/system/alert_cpp`。
  - 若希望 C++ 节点与 Python 节点**互操作测试**，也可以使用相同 Topic 名，但不建议同时启动同类型的两套节点，以免混淆。
- **参数与 QoS**：
  - 与 Python 版本一致：
    - 发布频率参数 `publish_rate`。
    - QoS 参数（`reliability`、`history`、`depth`），在 C++ 节点中通过 `rclcpp::QoS` 与 `declare_parameter` 加载。
  - 状态监控节点同样使用超时时间 `timeout_sec`、状态发布周期 `status_publish_period`、告警冷却 `alert_cooldown_sec`。

## C++ 节点设计细节

### 1. `multisensor_sensors_cpp` 包

- **CMake 与 package.xml**：
  - `CMakeLists.txt`：
    - `find_package(rclcpp REQUIRED)`
    - `find_package(multisensor_interfaces REQUIRED)`
    - 为每个节点添加可执行文件：`add_executable(vision_node_cpp src/vision_node_cpp.cpp)` 等，并 `ament_target_dependencies`。
  - `package.xml`：声明依赖 `rclcpp` 与 `multisensor_interfaces`。
- **节点逻辑**：
  - `vision_node_cpp`：
    - Node 名：`vision_node_cpp`。
    - 参数：`publish_rate`、模拟检测概率、角度均值/方差、置信度范围等（与 Python 版本对齐）。
    - 使用 `rclcpp::TimerBase` 定时发布 `Vision` 消息，填充 `header.stamp` 与 `frame_id`，随机模拟 `detected` / `angle` / `confidence`。
  - `audio_node_cpp`：
    - 类似地模拟 `voice_detected` / `direction` / `energy`，使用角度归一化函数实现 (-180°, 180°]) 区间。
  - `lidar_node_cpp`：
    - 随机模拟障碍物与距离（米），保持与 Python `Lidar` 逻辑相似。

### 2. `multisensor_fusion_cpp` 包

- **订阅与同步**：
  - 使用 `message_filters` 的 C++ 版本（或 `message_filters::TimeSynchronizer` / `message_filters::Synchronizer` + 策略）对 `Vision` / `Audio` / `Lidar` 三路 Topic 进行时间同步。
- **融合逻辑**：
  - 与 Python 中 `compute_fusion` 一致：
    - 全 False → `none`；仅视觉 → `vision_only`；仅音频 → `audio_only`；角度差超阈值 → `conflict`；角度差在阈值内 → `both`。
    - 使用参数 `angle_tolerance_deg`、`vision_weight`、`audio_weight` 进行角度与置信度加权平均。
- **输出**：
  - 发布 `FusionResult` 到 `/fusion/result_cpp` 或 `/fusion/result`。
  - QoS 使用 `Reliable` + `KeepLast(10)`，与 Python 版本对齐。

### 3. `multisensor_monitor_cpp` 包

- **监控目标**：
  - 订阅 C++ 版的传感器与融合话题（如：`/vision_cpp/data` 等），记录最后一次更新时间。
  - 定时检查 `now - last_update > timeout_sec` 的节点，判定为掉线并发布告警。
- **状态话题与服务**：
  - 发布 `NodeStatus` 到 `/system/status_cpp`（或共用 `/system/status`）。
  - 发布 `SystemAlert` 到 `/system/alert_cpp`（或共用原 Topic）。
  - 提供 `GetStatus` 与 `SetMonitorParam` 服务，与 Python 版接口一致，方便复用测试工具或 `ros2 service call`。

## Launch 与配置计划

- 在 `multisensor_bringup` 中新增 Launch：
  - `[multisensor_bringup/launch/sensors_cpp.launch.py]`：启动 3 个 C++ 传感器节点，并加载 `config/sensors_cpp.yaml` 或复用已有 `sensors.yaml`（若参数名相同）。
  - `[multisensor_bringup/launch/fusion_cpp.launch.py]`：启动 C++ 融合节点并加载 `fusion_cpp.yaml`。
  - `[multisensor_bringup/launch/monitor_cpp.launch.py]`：启动 C++ 状态监控节点并加载 `monitor_cpp.yaml`。
  - `[multisensor_bringup/launch/bringup_all_cpp.launch.py]`：一键启动 C++ 版本全系统。
- 可选：在已有 `bringup_all.launch.py` 中增加参数开关，选择启动 Python 版本、C++ 版本或两者之一。

## 测试与对比计划

- **功能对比**：
  - 通过 `ros2 topic echo` 与 `rqt_plot` 对比 Python 版与 C++ 版传感器与融合输出的一致性（统计角度分布、置信度范围等）。
  - 测试 C++版 `status_monitor_cpp` 对 C++ 节点断线 3 秒后的告警行为。
- **混合场景**：
  - 例如：Python 传感器 + C++融合 + Python 状态监控；或 C++ 传感器 + Python 融合等，用于验证接口跨语言兼容性。

## 文档补充

- 在现有 `README.md` 中新增一节：
  - 说明新增 C++ 包的结构与主要节点名称。
  - 给出构建与运行示例：`colcon build --packages-select multisensor_sensors_cpp ...` 与 `ros2 launch multisensor_bringup bringup_all_cpp.launch.py`。
  - 简要对比 Python 与 C++ 实现的差异（如性能、语言特性、代码结构）。

