import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('stereo_camera'),
        'config',
        'stereo_camera.yaml',
    )

    # see stereo_lidar_record.launch.py / README "Recording to a bag" for why
    fastdds_profile = SetEnvironmentVariable(
        'FASTRTPS_DEFAULT_PROFILES_FILE',
        os.path.join(get_package_share_directory('stereo_camera'), 'config', 'fastdds_large_images.xml'),
    )

    return LaunchDescription([
        fastdds_profile,
        Node(
            package='stereo_camera',
            executable='stereo_camera_node',
            name='stereo_camera_node',
            output='screen',
            parameters=[config],
        ),
    ])
