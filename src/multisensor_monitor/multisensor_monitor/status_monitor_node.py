from typing import Dict, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.time import Time
from rclpy.action import ActionServer, CancelResponse, GoalResponse

from multisensor_interfaces.msg import (
    NodeStatusMsg,
    AlertMsg,
    VisionMsg,
    AudioMsg,
    LidarMsg,
    FusionMsg,
)
from multisensor_interfaces.srv import GetStatus
from multisensor_interfaces.action import AdjustNodeParams


class StatusMonitorNode(Node):
    def __init__(self) -> None:
        super().__init__('status_monitor')

        self.timeout_sec = self.declare_parameter('timeout_sec', 3.0).get_parameter_value().double_value
        self.status_publish_period = self.declare_parameter('status_publish_period', 1.0).get_parameter_value().double_value
        self.alert_cooldown_sec = self.declare_parameter('alert_cooldown_sec', 5.0).get_parameter_value().double_value

        self.qos_status = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self._subscriptions = []
        self.last_update: Dict[str, Optional[Time]] = {
            'vision_node': None,
            'audio_node': None,
            'lidar_node': None,
            'fusion_node': None,
        }
        self.last_alert_time: Dict[str, Optional[Time]] = {name: None for name in self.last_update.keys()}

        self._subscriptions.append(
            self.create_subscription(VisionMsg, '/vision/detection', self._make_cb('vision_node'), qos_sensor)
        )
        self._subscriptions.append(
            self.create_subscription(AudioMsg, '/audio/source_angle', self._make_cb('audio_node'), qos_sensor)
        )
        self._subscriptions.append(
            self.create_subscription(LidarMsg, '/lidar/scan', self._make_cb('lidar_node'), qos_sensor)
        )
        self._subscriptions.append(
            self.create_subscription(FusionMsg, '/fusion/result', self._make_cb('fusion_node'), qos_sensor)
        )

        self.pub_status = self.create_publisher(NodeStatusMsg, '/system/status', self.qos_status)
        self.pub_alert = self.create_publisher(AlertMsg, '/system/alert', self.qos_status)

        self.timer = self.create_timer(self.status_publish_period, self.timer_callback)

        self.srv_get_status = self.create_service(GetStatus, '/system/get_status', self.handle_get_status)

        self.adjust_action_server = ActionServer(
            self,
            AdjustNodeParams,
            '/system/adjust_node_params',
            execute_callback=self.execute_adjust_node_params,
            goal_callback=self.handle_adjust_goal,
            cancel_callback=self.handle_adjust_cancel,
        )

        self.get_logger().info(
            f'status_monitor started with timeout={self.timeout_sec}s, '
            f'status_period={self.status_publish_period}s'
        )

    def _make_cb(self, node_name: str):
        def cb(_msg) -> None:
            self.last_update[node_name] = self.get_clock().now()
        return cb

    def timer_callback(self) -> None:
        now = self.get_clock().now()

        for node_name, last_time in self.last_update.items():
            age = float('inf')
            alive = False
            if last_time is not None:
                age = (now - last_time).nanoseconds / 1e9
                alive = age <= self.timeout_sec

            status_msg = NodeStatusMsg()
            status_msg.header.stamp = now.to_msg()
            status_msg.header.frame_id = ''
            status_msg.node_name = node_name
            status_msg.is_online = alive
            status_msg.last_update_time = float(age if age != float('inf') else -1.0)
            self.pub_status.publish(status_msg)

            if not alive:
                self.maybe_publish_alert(node_name, now, age)

    def maybe_publish_alert(self, node_name: str, now: Time, age: float) -> None:
        last_alert = self.last_alert_time.get(node_name)
        if last_alert is not None:
            dt = (now - last_alert).nanoseconds / 1e9
            if dt < self.alert_cooldown_sec:
                return

        alert = AlertMsg()
        alert.header.stamp = now.to_msg()
        alert.header.frame_id = ''
        alert.message = f'WARN [{node_name}]: no update for {age:.2f}s (> {self.timeout_sec}s)'

        self.pub_alert.publish(alert)
        self.last_alert_time[node_name] = now

    def handle_get_status(self, request: GetStatus.Request, response: GetStatus.Response) -> GetStatus.Response:
        now = self.get_clock().now()
        target = request.node_name.strip()
        names = [target] if target else list(self.last_update.keys())

        for node_name in names:
            if node_name not in self.last_update:
                continue
            last_time = self.last_update[node_name]
            age = float('inf')
            alive = False
            if last_time is not None:
                age = (now - last_time).nanoseconds / 1e9
                alive = age <= self.timeout_sec

            status_msg = NodeStatusMsg()
            status_msg.header.stamp = now.to_msg()
            status_msg.header.frame_id = ''
            status_msg.node_name = node_name
            status_msg.is_online = alive
            status_msg.last_update_time = float(age if age != float('inf') else -1.0)
            response.statuses.append(status_msg)

        return response

    def handle_adjust_goal(self, goal_request: AdjustNodeParams.Goal) -> GoalResponse:
        key = goal_request.key.strip()
        if key in ('timeout_sec', 'status_publish_period', 'alert_cooldown_sec'):
            return GoalResponse.ACCEPT
        self.get_logger().warn(f'Reject AdjustNodeParams goal with unknown key: {key}')
        return GoalResponse.REJECT

    def handle_adjust_cancel(self, _goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    async def execute_adjust_node_params(self, goal_handle) -> AdjustNodeParams.Result:
        goal = goal_handle.request
        key = goal.key.strip()
        value = goal.value

        feedback = AdjustNodeParams.Feedback()
        feedback.progress = 0.0
        feedback.current_state = 'validating'
        goal_handle.publish_feedback(feedback)

        success = True
        message = ''

        try:
            if key == 'timeout_sec':
                self.timeout_sec = float(value)
                message = f'Updated timeout_sec to {value}'
            elif key == 'status_publish_period':
                self.status_publish_period = float(value)
                self.timer.cancel()
                self.timer = self.create_timer(self.status_publish_period, self.timer_callback)
                message = f'Updated status_publish_period to {value}'
            elif key == 'alert_cooldown_sec':
                self.alert_cooldown_sec = float(value)
                message = f'Updated alert_cooldown_sec to {value}'
            else:
                success = False
                message = f'Unknown parameter key: {key}'
        except Exception as exc:  # noqa: BLE001
            success = False
            message = f'Exception while updating param {key}: {exc}'
            self.get_logger().error(message)

        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            result = AdjustNodeParams.Result()
            result.success = False
            result.message = 'Goal canceled'
            return result

        feedback.progress = 1.0
        feedback.current_state = 'done' if success else 'failed'
        goal_handle.publish_feedback(feedback)

        result = AdjustNodeParams.Result()
        result.success = success
        result.message = message

        if success:
            goal_handle.succeed()
        else:
            goal_handle.abort()

        return result


def main(args=None) -> None:
    rclpy.init(args=args)
    node = StatusMonitorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

