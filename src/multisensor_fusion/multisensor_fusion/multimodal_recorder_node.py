from typing import Optional

import rclpy
from rclpy.node import Node
from message_filters import Subscriber, ApproximateTimeSynchronizer

from multisensor_interfaces.msg import (
    VisionMsg,
    AudioMsg,
    LidarMsg,
    FusionMsg,
    MultimodalSample,
)


class MultimodalRecorderNode(Node):
    def __init__(self) -> None:
        super().__init__('multimodal_recorder')

        self.sync_queue_size = self.declare_parameter('sync_queue_size', 10).get_parameter_value().integer_value
        self.sync_slop = self.declare_parameter('sync_slop', 0.1).get_parameter_value().double_value
        self.source_profile = self.declare_parameter('source_profile', 'optimized').get_parameter_value().string_value

        self.pub_sample = self.create_publisher(MultimodalSample, '/multimodal/sample', 10)

        # message_filters 订阅轻量摘要话题
        self.sub_vision = Subscriber(self, VisionMsg, '/vision/detection')
        self.sub_audio = Subscriber(self, AudioMsg, '/audio/source_angle')
        self.sub_lidar = Subscriber(self, LidarMsg, '/lidar/scan')
        self.sub_fusion = Subscriber(self, FusionMsg, '/fusion/result')

        self.sync = ApproximateTimeSynchronizer(
            [self.sub_vision, self.sub_audio, self.sub_lidar, self.sub_fusion],
            queue_size=self.sync_queue_size,
            slop=self.sync_slop,
        )
        self.sync.registerCallback(self.sync_callback)

        self.get_logger().info(
            f'multimodal_recorder started: sync_queue_size={self.sync_queue_size}, '
            f'slop={self.sync_slop}, source_profile={self.source_profile}'
        )

    def sync_callback(
        self,
        vision: VisionMsg,
        audio: AudioMsg,
        lidar: LidarMsg,
        fusion: FusionMsg,
    ) -> None:
        sample = MultimodalSample()
        sample.header = fusion.header  # 以融合结果时间为准
        sample.vision_summary = vision
        sample.audio_summary = audio
        sample.lidar_summary = lidar
        sample.fusion_result = fusion
        sample.source_profile = self.source_profile
        self.pub_sample.publish(sample)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = MultimodalRecorderNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

