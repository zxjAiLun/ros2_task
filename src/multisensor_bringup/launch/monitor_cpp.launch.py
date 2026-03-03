from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory('multisensor_bringup')
    monitor_params = [
        os.path.join(pkg_share, 'config', 'monitor.yaml'),
    ]

    return LaunchDescription(
        [
            Node(
                package='multisensor_monitor_cpp',
                executable='status_monitor_node_cpp',
                name='status_monitor_cpp',
                parameters=monitor_params,
                output='screen',
            ),
        ]
    )

