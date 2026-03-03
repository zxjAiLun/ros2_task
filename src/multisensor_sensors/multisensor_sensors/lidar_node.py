import random

import rclpy
from rclpy.node import Node

from multisensor_interfaces.msg import Lidar
from .common import (
    create_qos_from_params,
    get_publish_rate_hz,
    load_simulation_params,
)


class LidarNode(Node):
    def __init__(self) -> None:
        super().__init__('lidar_node')

        self.frame_id = self.declare_parameter('frame_id', 'lidar_link').get_parameter_value().string_value
        self.qos_profile = create_qos_from_params(self, 'qos')
        self.publish_rate_hz = get_publish_rate_hz(self, 5.0)

        defaults = {
            'obstacle_probability': 0.4,
            'distance_min_m': 0.3,
            'distance_max_m': 5.0,
            'no_obstacle_distance_m': 10.0,
        }
        self.sim_params = load_simulation_params(self, defaults)

        self.publisher_ = self.create_publisher(Lidar, '/lidar/data', self.qos_profile)
        timer_period = 1.0 / self.publish_rate_hz if self.publish_rate_hz > 0.0 else 0.2
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info(
            f'lidar_node started: rate={self.publish_rate_hz}Hz, '
            f'frame_id={self.frame_id}, qos={self.qos_profile}'
        )

    def timer_callback(self) -> None:
        msg = Lidar()
        now = self.get_clock().now().to_msg()
        msg.header.stamp = now
        msg.header.frame_id = self.frame_id

        has_obstacle = random.random() < self.sim_params['obstacle_probability']
        msg.obstacle_detected = has_obstacle

        if has_obstacle:
            msg.min_distance = random.uniform(self.sim_params['distance_min_m'], self.sim_params['distance_max_m'])
        else:
            msg.min_distance = float(self.sim_params['no_obstacle_distance_m'])

        self.publisher_.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LidarNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

