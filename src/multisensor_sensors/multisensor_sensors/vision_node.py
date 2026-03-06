import random

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

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
        self.image_width = int(self.declare_parameter('image_width', 320).get_parameter_value().integer_value)
        self.image_height = int(self.declare_parameter('image_height', 240).get_parameter_value().integer_value)
        self.image_rate_hz = float(
            self.declare_parameter('image_rate', float(self.publish_rate_hz)).get_parameter_value().double_value
        )

        defaults = {
            'detect_probability': 0.6,
            'confidence_min': 0.5,
            'confidence_max': 1.0,
        }
        self.sim_params = load_simulation_params(self, defaults)

        self.publisher_ = self.create_publisher(VisionMsg, '/vision/detection', self.qos_profile)
        self.image_publisher_ = self.create_publisher(Image, '/vision/image_raw', self.qos_profile)

        timer_period = 1.0 / self.publish_rate_hz if self.publish_rate_hz > 0.0 else 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)

        image_timer_period = 1.0 / self.image_rate_hz if self.image_rate_hz > 0.0 else timer_period
        self.image_timer = self.create_timer(image_timer_period, self.image_timer_callback)

        self.get_logger().info(
            f'vision_node started: rate={self.publish_rate_hz}Hz, '
            f'frame_id={self.frame_id}, qos={self.qos_profile}, '
            f'image: {self.image_width}x{self.image_height}@{self.image_rate_hz}Hz'
        )

    def timer_callback(self) -> None:
        msg = VisionMsg()
        now = self.get_clock().now().to_msg()
        msg.header.stamp = now
        msg.header.frame_id = self.frame_id
        msg.encoding = 'mono8'

        # 用与图像类似的方式生成一帧灰度像素数据，直接存入 VisionMsg.data
        base_level = 30
        target_level = 200
        detected = random_bool_with_probability(self.sim_params['detect_probability'])
        level = target_level if detected else base_level
        msg.data = bytes([level] * (self.image_width * self.image_height))

        self.publisher_.publish(msg)

    def image_timer_callback(self) -> None:
        # 简单生成一张灰度图：根据 person_detected 概率模拟亮度
        img = Image()
        now = self.get_clock().now().to_msg()
        img.header.stamp = now
        img.header.frame_id = self.frame_id
        img.height = self.image_height
        img.width = self.image_width
        img.encoding = 'mono8'
        img.is_bigendian = 0
        img.step = self.image_width

        # 亮度模式：大部分时间为低亮度，模拟环境背景；在检测概率内使用较高亮度模拟目标区域
        base_level = 30
        target_level = 200
        detected = random_bool_with_probability(self.sim_params['detect_probability'])
        level = target_level if detected else base_level
        img.data = bytes([level] * (self.image_width * self.image_height))

        self.image_publisher_.publish(img)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VisionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

