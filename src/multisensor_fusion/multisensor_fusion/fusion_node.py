from typing import Optional

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from message_filters import ApproximateTimeSynchronizer, Subscriber

from multisensor_interfaces.msg import Vision, Audio, Lidar, FusionResult


def create_fusion_qos(node: Node) -> QoSProfile:
    reliability_str = node.declare_parameter('fusion_qos.reliability', 'reliable').get_parameter_value().string_value
    history_str = node.declare_parameter('fusion_qos.history', 'keep_last').get_parameter_value().string_value
    depth = node.declare_parameter('fusion_qos.depth', 10).get_parameter_value().integer_value

    reliability = ReliabilityPolicy.RELIABLE if reliability_str.lower() == 'reliable' else ReliabilityPolicy.BEST_EFFORT
    history = HistoryPolicy.KEEP_LAST if history_str.lower() == 'keep_last' else HistoryPolicy.KEEP_ALL

    return QoSProfile(
        reliability=reliability,
        history=history,
        depth=depth,
    )


class FusionNode(Node):
    def __init__(self) -> None:
        super().__init__('fusion_node')

        self.angle_tolerance_deg = self.declare_parameter('angle_tolerance_deg', 30.0).get_parameter_value().double_value
        self.vision_weight = self.declare_parameter('vision_weight', 0.6).get_parameter_value().double_value
        self.audio_weight = self.declare_parameter('audio_weight', 0.4).get_parameter_value().double_value

        self.queue_size = self.declare_parameter('sync_queue_size', 10).get_parameter_value().integer_value
        self.slop = self.declare_parameter('sync_slop', 0.1).get_parameter_value().double_value

        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.sub_vision = Subscriber(self, Vision, '/vision/data', qos_profile=qos_sensor)
        self.sub_audio = Subscriber(self, Audio, '/audio/data', qos_profile=qos_sensor)
        self.sub_lidar = Subscriber(self, Lidar, '/lidar/data', qos_profile=qos_sensor)

        self.sync = ApproximateTimeSynchronizer(
            [self.sub_vision, self.sub_audio, self.sub_lidar],
            queue_size=self.queue_size,
            slop=self.slop,
        )
        self.sync.registerCallback(self.sync_callback)

        self.qos_fusion = create_fusion_qos(self)
        self.pub_fusion = self.create_publisher(FusionResult, '/fusion/result', self.qos_fusion)

        self.get_logger().info(
            f'fusion_node started with angle_tolerance={self.angle_tolerance_deg}deg, '
            f'queue_size={self.queue_size}, slop={self.slop}'
        )

    def sync_callback(self, vision: Vision, audio: Audio, lidar: Lidar) -> None:
        result = FusionResult()
        result.header.stamp = vision.header.stamp
        result.header.frame_id = 'fusion_base'

        status, confidence, target_angle = self.compute_fusion(vision, audio, lidar)
        result.fusion_status = status
        result.fusion_confidence = confidence
        result.target_angle = target_angle if target_angle is not None else float('nan')

        self.pub_fusion.publish(result)

    def compute_fusion(
        self,
        vision: Vision,
        audio: Audio,
        lidar: Lidar,
    ) -> tuple[str, float, Optional[float]]:
        v = vision.detected
        a = audio.voice_detected

        if not v and not a:
            return 'none', 0.0, None
        if v and not a:
            return 'vision_only', max(vision.confidence, 0.0), vision.angle
        if a and not v:
            return 'audio_only', max(audio.energy, 0.0), audio.direction

        if math.isnan(vision.angle) or math.isnan(audio.direction):
            return 'conflict', 0.0, None

        diff = abs(self.normalize_angle_deg(vision.angle - audio.direction))
        if diff <= self.angle_tolerance_deg:
            v_conf = max(vision.confidence, 0.0)
            a_conf = max(audio.energy, 0.0)
            norm = v_conf + a_conf
            if norm > 0.0:
                v_w = self.vision_weight * v_conf / norm
                a_w = self.audio_weight * a_conf / norm
            else:
                v_w = self.vision_weight
                a_w = self.audio_weight
            fused_angle = v_w * vision.angle + a_w * audio.direction
            fused_conf = min(1.0, (v_conf * self.vision_weight + a_conf * self.audio_weight))
            return 'both', fused_conf, fused_angle

        return 'conflict', 0.0, None

    @staticmethod
    def normalize_angle_deg(angle: float) -> float:
        value = math.fmod(angle, 360.0)
        if value <= -180.0:
            value += 360.0
        elif value > 180.0:
            value -= 360.0
        return value


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FusionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

