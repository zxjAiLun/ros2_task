from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory('multisensor_bringup')
    fusion_params = [
        os.path.join(pkg_share, 'config', 'fusion.yaml'),
    ]

    return LaunchDescription(
        [
            Node(
                package='multisensor_fusion',
                executable='fusion_node',
                name='fusion_node',
                parameters=fusion_params,
                output='screen',
            ),
        ]
    )

