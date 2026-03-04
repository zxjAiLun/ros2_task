from typing import Optional

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from message_filters import ApproximateTimeSynchronizer, Subscriber

from multisensor_interfaces.msg import VisionMsg, AudioMsg, LidarMsg, FusionMsg


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
        self.sub_vision = Subscriber(self, VisionMsg, '/vision/detection', qos_profile=qos_sensor)
        self.sub_audio = Subscriber(self, AudioMsg, '/audio/source_angle', qos_profile=qos_sensor)
        self.sub_lidar = Subscriber(self, LidarMsg, '/lidar/scan', qos_profile=qos_sensor)

        self.sync = ApproximateTimeSynchronizer(
            [self.sub_vision, self.sub_audio, self.sub_lidar],
            queue_size=self.queue_size,
            slop=self.slop,
        )
        self.sync.registerCallback(self.sync_callback)

        self.qos_fusion = create_fusion_qos(self)
        self.pub_fusion = self.create_publisher(FusionMsg, '/fusion/result', self.qos_fusion)

        self.get_logger().info(
            f'fusion_node started with angle_tolerance={self.angle_tolerance_deg}deg, '
            f'queue_size={self.queue_size}, slop={self.slop}'
        )

    def sync_callback(self, vision: VisionMsg, audio: AudioMsg, lidar: LidarMsg) -> None:
        result = FusionMsg()
        result.header.stamp = vision.header.stamp
        result.header.frame_id = 'fusion_base'

        status, vision_conf, audio_angle, lidar_distance = self.compute_fusion(vision, audio, lidar)
        result.vision_person_detected = vision.person_detected
        result.vision_confidence = vision_conf
        result.audio_source_angle = audio_angle
        result.lidar_distance = lidar_distance
        result.fusion_status = status

        self.pub_fusion.publish(result)

    def compute_fusion(
        self,
        vision: VisionMsg,
        audio: AudioMsg,
        lidar: LidarMsg,
    ) -> tuple[str, float, float, float]:
        v = vision.person_detected
        a_angle = audio.audio_source_angle

        # lidar_distance 始终提供（用于规则或上层使用）
        lidar_distance = lidar.lidar_distance

        # 简化后的规则：只看视觉是否有人 + 音频角是否有效
        if not v and math.isnan(a_angle):
            return 'none', 0.0, float('nan'), lidar_distance
        if v and math.isnan(a_angle):
            return 'vision_only', max(vision.vision_confidence, 0.0), float('nan'), lidar_distance
        if (not v) and not math.isnan(a_angle):
            return 'audio_only', 0.0, a_angle, lidar_distance

        # 视觉有人，音频角有效：认为多源一致，输出 both
        return 'both', max(vision.vision_confidence, 0.0), a_angle, lidar_distance

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

