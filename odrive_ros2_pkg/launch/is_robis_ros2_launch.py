import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_lidar = get_package_share_directory("ydlidar_ros2_driver")
    pkg_odrive = get_package_share_directory("odrive_ros2_pkg")

    cmd_lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_lidar, "launch", "ydlidar_launch.py"),
        ),
        launch_arguments={
            "params_file": os.path.join(pkg_odrive, "params", "ydlidar.yaml")
        }.items(),
    )

    cmd_odrive = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_odrive, "launch", "odrive_launch.py")
        ),
    )

    return LaunchDescription(
        [
            cmd_lidar,
            cmd_odrive,
        ]
    )
