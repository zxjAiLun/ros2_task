from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory('multisensor_bringup')

    sensors_cpp_launch = PythonLaunchDescriptionSource(
        os.path.join(pkg_share, 'launch', 'sensors_cpp.launch.py')
    )
    fusion_cpp_launch = PythonLaunchDescriptionSource(
        os.path.join(pkg_share, 'launch', 'fusion_cpp.launch.py')
    )
    monitor_cpp_launch = PythonLaunchDescriptionSource(
        os.path.join(pkg_share, 'launch', 'monitor_cpp.launch.py')
    )

    return LaunchDescription(
        [
            IncludeLaunchDescription(sensors_cpp_launch),
            IncludeLaunchDescription(fusion_cpp_launch),
            IncludeLaunchDescription(monitor_cpp_launch),
        ]
    )

