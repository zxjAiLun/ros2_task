from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory('multisensor_bringup')

    sensors_launch = PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'sensors.launch.py'))
    fusion_launch = PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'fusion.launch.py'))
    monitor_launch = PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'monitor.launch.py'))

    return LaunchDescription(
        [
            IncludeLaunchDescription(sensors_launch),
            IncludeLaunchDescription(fusion_launch),
            IncludeLaunchDescription(monitor_launch),
        ]
    )

