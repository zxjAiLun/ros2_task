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
            f'fusion_node started for raw multimodal fusion, '
            f'queue_size={self.queue_size}, slop={self.slop}'
        )

    def sync_callback(self, vision: VisionMsg, audio: AudioMsg, lidar: LidarMsg) -> None:
        result = FusionMsg()
        result.header.stamp = vision.header.stamp
        result.header.frame_id = 'fusion_base'
        result.frame_id = 'fusion_base'

        # 直接将对齐后的原始多模态数据打包输出，供下游大模型使用
        result.vision = vision
        result.audio = audio
        result.lidar = lidar
        result.is_aligned = True

        self.pub_fusion.publish(result)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FusionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

