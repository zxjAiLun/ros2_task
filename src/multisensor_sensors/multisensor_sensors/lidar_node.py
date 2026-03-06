import random

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import struct

from multisensor_interfaces.msg import LidarMsg
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
        self.points_per_scan = int(
            self.declare_parameter('points_per_scan', 360).get_parameter_value().integer_value
        )

        defaults = {
            'obstacle_probability': 0.4,
            'distance_min_m': 0.3,
            'distance_max_m': 5.0,
            'no_obstacle_distance_m': 10.0,
        }
        self.sim_params = load_simulation_params(self, defaults)

        self.publisher_ = self.create_publisher(LidarMsg, '/lidar/scan', self.qos_profile)
        self.points_publisher_ = self.create_publisher(PointCloud2, '/lidar/points', self.qos_profile)

        timer_period = 1.0 / self.publish_rate_hz if self.publish_rate_hz > 0.0 else 0.2
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info(
            f'lidar_node started: rate={self.publish_rate_hz}Hz, '
            f'frame_id={self.frame_id}, qos={self.qos_profile}, '
            f'points_per_scan={self.points_per_scan}'
        )

    def timer_callback(self) -> None:
        msg = LidarMsg()
        now = self.get_clock().now().to_msg()
        msg.header.stamp = now
        msg.header.frame_id = self.frame_id
        msg.type = '2d_circle'

        has_obstacle = random.random() < self.sim_params['obstacle_probability']

        if has_obstacle:
            radius = random.uniform(
                self.sim_params['distance_min_m'],
                self.sim_params['distance_max_m'],
            )
        else:
            radius = float(self.sim_params['no_obstacle_distance_m'])

        points = []
        for i in range(self.points_per_scan):
            angle = 2.0 * 3.1415926 * float(i) / float(self.points_per_scan)
            x = radius * float(__import__('math').cos(angle))
            y = radius * float(__import__('math').sin(angle))
            z = 0.0
            points.extend([float(x), float(y), float(z)])

        msg.point_cloud = points
        msg.point_count = len(points) // 3

        self.publisher_.publish(msg)

        # 同步生成一个简单环状 PointCloud2
        header = Header()
        header.stamp = now
        header.frame_id = self.frame_id
        cloud = PointCloud2()
        cloud.header = header
        cloud.height = 1
        cloud.width = self.points_per_scan
        cloud.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        cloud.is_bigendian = False
        cloud.point_step = 12
        cloud.row_step = cloud.point_step * cloud.width
        cloud.is_dense = True

        # 以同样的半径，在平面上生成 points_per_scan 个点
        radius = radius
        points_bin = []
        for i in range(self.points_per_scan):
            angle = 2.0 * 3.1415926 * float(i) / float(self.points_per_scan)
            x = radius * float(__import__('math').cos(angle))
            y = radius * float(__import__('math').sin(angle))
            z = 0.0
            points_bin.append(struct.pack('fff', x, y, z))
        cloud.data = b''.join(points_bin)

        self.points_publisher_.publish(cloud)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LidarNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

