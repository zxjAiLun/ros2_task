import random

import rclpy
from rclpy.node import Node

from multisensor_interfaces.msg import AudioMsg
from .common import (
    create_qos_from_params,
    get_publish_rate_hz,
    angle_deg_normalized,
    random_bool_with_probability,
    load_simulation_params,
)


class AudioNode(Node):
    def __init__(self) -> None:
        super().__init__('audio_node')

        self.frame_id = self.declare_parameter('frame_id', 'mic_link').get_parameter_value().string_value
        self.qos_profile = create_qos_from_params(self, 'qos')
        self.publish_rate_hz = get_publish_rate_hz(self, 10.0)

        defaults = {
            'voice_probability': 0.5,
            'angle_mean_deg': 0.0,
            'angle_sigma_deg': 45.0,
            'energy_min': 0.2,
            'energy_max': 1.0,
        }
        self.sim_params = load_simulation_params(self, defaults)

        self.publisher_ = self.create_publisher(AudioMsg, '/audio/source_angle', self.qos_profile)
        timer_period = 1.0 / self.publish_rate_hz if self.publish_rate_hz > 0.0 else 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info(
            f'audio_node started: rate={self.publish_rate_hz}Hz, '
            f'frame_id={self.frame_id}, qos={self.qos_profile}'
        )

    def timer_callback(self) -> None:
        msg = AudioMsg()
        now = self.get_clock().now().to_msg()
        msg.header.stamp = now
        msg.header.frame_id = self.frame_id
        msg.format = 'pcm16'

        # 简单生成一段固定长度的一维音频波形（正弦 + 噪声），直接序列化为 int16/uint8
        sample_count = 160  # 例如 10ms@16kHz
        amplitude = 20000
        freq = 440.0
        samples = []
        for i in range(sample_count):
            t = float(i) / 16000.0
            value = int(
                amplitude
                * random_bool_with_probability(self.sim_params['voice_probability'])
                * __import__('math').sin(2.0 * __import__('math').pi * freq * t)
            )
            samples.append(value & 0xFFFF)
        # 小端 int16 打包
        msg.data = b''.join(int.to_bytes(s, length=2, byteorder='little', signed=False) for s in samples)

        self.publisher_.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AudioNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

