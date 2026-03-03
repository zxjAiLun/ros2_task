import random

import rclpy
from rclpy.node import Node

from multisensor_interfaces.msg import Audio
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

        self.publisher_ = self.create_publisher(Audio, '/audio/data', self.qos_profile)
        timer_period = 1.0 / self.publish_rate_hz if self.publish_rate_hz > 0.0 else 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info(
            f'audio_node started: rate={self.publish_rate_hz}Hz, '
            f'frame_id={self.frame_id}, qos={self.qos_profile}'
        )

    def timer_callback(self) -> None:
        msg = Audio()
        now = self.get_clock().now().to_msg()
        msg.header.stamp = now
        msg.header.frame_id = self.frame_id

        voice_detected = random_bool_with_probability(self.sim_params['voice_probability'])
        msg.voice_detected = voice_detected

        if voice_detected:
            angle = random.gauss(self.sim_params['angle_mean_deg'], self.sim_params['angle_sigma_deg'])
            msg.direction = float(angle_deg_normalized(angle))
            msg.energy = random.uniform(self.sim_params['energy_min'], self.sim_params['energy_max'])
        else:
            msg.direction = float('nan')
            msg.energy = 0.0

        self.publisher_.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AudioNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

