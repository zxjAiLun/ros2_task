from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory('multisensor_bringup')
    sensors_params = [
        os.path.join(pkg_share, 'config', 'sensors.yaml'),
    ]

    return LaunchDescription(
        [
            Node(
                package='multisensor_sensors',
                executable='vision_node',
                name='vision_node',
                parameters=sensors_params,
                output='screen',
            ),
            Node(
                package='multisensor_sensors',
                executable='audio_node',
                name='audio_node',
                parameters=sensors_params,
                output='screen',
            ),
            Node(
                package='multisensor_sensors',
                executable='lidar_node',
                name='lidar_node',
                parameters=sensors_params,
                output='screen',
            ),
        ]
    )

