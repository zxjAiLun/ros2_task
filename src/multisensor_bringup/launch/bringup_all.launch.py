from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from pathlib import Path


def generate_launch_description() -> LaunchDescription:
    pkg_share = Path(__file__).resolve().parent.parent

    sensors_launch = PythonLaunchDescriptionSource(str(pkg_share / 'launch' / 'sensors.launch.py'))
    fusion_launch = PythonLaunchDescriptionSource(str(pkg_share / 'launch' / 'fusion.launch.py'))
    monitor_launch = PythonLaunchDescriptionSource(str(pkg_share / 'launch' / 'monitor.launch.py'))

    return LaunchDescription(
        [
            IncludeLaunchDescription(sensors_launch),
            IncludeLaunchDescription(fusion_launch),
            IncludeLaunchDescription(monitor_launch),
        ]
    )

