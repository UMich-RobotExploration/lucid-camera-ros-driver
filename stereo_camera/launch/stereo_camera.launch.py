import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('stereo_camera'),
        'config',
        'stereo_camera.yaml',
    )

    return LaunchDescription([
        Node(
            package='stereo_camera',
            executable='stereo_camera_node',
            name='stereo_camera_node',
            output='screen',
            parameters=[config],
        ),
    ])
