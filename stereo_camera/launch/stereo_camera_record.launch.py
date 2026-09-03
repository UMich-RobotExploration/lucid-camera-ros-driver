import os
from datetime import datetime

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('stereo_camera'),
        'config',
        'stereo_camera.yaml',
    )

    default_bag_path = os.path.join(
        os.path.expanduser('~'),
        'bags',
        'stereo_camera_' + datetime.now().strftime('%Y%m%d_%H%M%S'),
    )

    bag_path_arg = DeclareLaunchArgument(
        'bag_path',
        default_value=default_bag_path,
        description='Output directory for the recorded ros2 bag',
    )

    camera_node = Node(
        package='stereo_camera',
        executable='stereo_camera_node',
        name='stereo_camera_node',
        output='screen',
        parameters=[config],
    )

    bag_record = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record',
            '-o', LaunchConfiguration('bag_path'),
            '/camera/left/image_raw',
            '/camera/right/image_raw',
        ],
        output='screen',
    )

    return LaunchDescription([
        bag_path_arg,
        camera_node,
        bag_record,
    ])
