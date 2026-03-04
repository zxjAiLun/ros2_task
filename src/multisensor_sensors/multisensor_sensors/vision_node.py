import random

import rclpy
from rclpy.node import Node

from multisensor_interfaces.msg import VisionMsg
from .common import (
    create_qos_from_params,
    get_publish_rate_hz,
    random_bool_with_probability,
    load_simulation_params,
)


class VisionNode(Node):
    def __init__(self) -> None:
        super().__init__('vision_node')

        self.frame_id = self.declare_parameter('frame_id', 'camera_link').get_parameter_value().string_value
        self.qos_profile = create_qos_from_params(self, 'qos')
        self.publish_rate_hz = get_publish_rate_hz(self, 10.0)

        defaults = {
            'detect_probability': 0.6,
            'confidence_min': 0.5,
            'confidence_max': 1.0,
        }
        self.sim_params = load_simulation_params(self, defaults)

        self.publisher_ = self.create_publisher(VisionMsg, '/vision/detection', self.qos_profile)
        timer_period = 1.0 / self.publish_rate_hz if self.publish_rate_hz > 0.0 else 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info(
            f'vision_node started: rate={self.publish_rate_hz}Hz, '
            f'frame_id={self.frame_id}, qos={self.qos_profile}'
        )

    def timer_callback(self) -> None:
        msg = VisionMsg()
        now = self.get_clock().now().to_msg()
        msg.header.stamp = now
        msg.header.frame_id = self.frame_id

        detected = random_bool_with_probability(self.sim_params['detect_probability'])
        msg.person_detected = detected

        if detected:
            msg.vision_confidence = random.uniform(
                self.sim_params['confidence_min'],
                self.sim_params['confidence_max'],
            )
        else:
            msg.vision_confidence = 0.0

        self.publisher_.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VisionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

