from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    """
    Launches OpenVINS with the baby_k_sewer configuration:
      - cam0: forward-facing  (main VIO)
      - cam1: downward-facing (sewer entrance detection)
    """

    from ament_index_python.packages import get_package_share_directory
    import os
    config_path = os.path.join(
        get_package_share_directory('ov_msckf'),
        'config',
        'baby_k_sewer',
        'estimator_config.yaml'
    )

    ov_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ov_msckf'),
                'launch',
                'subscribe.launch.py'
            ])
        ]),
        launch_arguments={
            'config_path': config_path,
            'use_stereo': 'false',
            'max_cameras': '2',
            'rviz_enable': 'false',
        }.items()
    )

    return LaunchDescription([
        ov_launch
    ])
