import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    bringup_dir = get_package_share_directory('drivelink_bringup')
    sllidar_dir = get_package_share_directory('sllidar_ros2')
    slam_dir = get_package_share_directory('slam_toolbox')

    slam_config = os.path.join(bringup_dir, 'config', 'slam_toolbox.yaml')

    # 1. LiDAR driver launch
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sllidar_dir, 'launch', 'sllidar_c1_launch.py')
        )
    )

    # 2. SLAM Toolbox (automatically configures & activates the node!)
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_dir, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'slam_params_file': slam_config,
            'use_sim_time': 'false',
        }.items(),
    )

    return LaunchDescription([
        lidar_launch,
        slam_launch,
    ])

