import os
from datetime import datetime

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    stereo_config = os.path.join(
        get_package_share_directory('stereo_camera'),
        'config',
        'stereo_camera.yaml',
    )

    ouster_share_dir = get_package_share_directory('ouster_ros')
    ouster_driver_launch = os.path.join(ouster_share_dir, 'launch', 'driver.launch.py')
    default_ouster_params = os.path.join(ouster_share_dir, 'config', 'driver_params.yaml')

    vectornav_share_dir = get_package_share_directory('vectornav')
    vectornav_launch = os.path.join(vectornav_share_dir, 'launch', 'vectornav.launch.py')

    default_bag_path = os.path.join(
        os.path.expanduser('~'),
        'bags',
        'stereo_lidar_' + datetime.now().strftime('%Y%m%d_%H%M%S'),
    )

    bag_path_arg = DeclareLaunchArgument(
        'bag_path',
        default_value=default_bag_path,
        description='Output directory for the recorded ros2 bag',
    )
    ouster_ns_arg = DeclareLaunchArgument(
        'ouster_ns',
        default_value='ouster',
        description='Namespace the ouster os_driver node publishes under',
    )
    ouster_params_file_arg = DeclareLaunchArgument(
        'ouster_params_file',
        default_value=default_ouster_params,
        description='params file for the ouster os_driver node '
                     '(set timestamp_mode: TIME_FROM_PTP_1588 in there for PTP sync)',
    )
    # named "enable_rviz", not "viz" -- ouster_ros's driver.launch.py also
    # declares an argument literally named "viz"; passing 'viz' into that
    # nested IncludeLaunchDescription overwrites any launch configuration
    # of the same name for the rest of this launch, which silently forced
    # our own rviz condition to false regardless of what was passed on the
    # command line.
    enable_rviz_arg = DeclareLaunchArgument(
        'enable_rviz',
        default_value='true',
        description='launch a combined rviz view (stereo images + ouster point cloud)',
    )
    enable_cameras_arg = DeclareLaunchArgument(
        'enable_cameras',
        default_value='true',
        description='launch the stereo_camera_node; set to false to run the '
                     'lidar alone (e.g. while debugging the lidar without the cameras attached)',
    )
    enable_imu_arg = DeclareLaunchArgument(
        'enable_imu',
        default_value='true',
        description='launch the vectornav IMU driver',
    )
    show_cameras_arg = DeclareLaunchArgument(
        'show_cameras',
        default_value='true',
        description='include the camera Image displays in rviz; set to false to view '
                     'only the lidar point cloud (e.g. while debugging the lidar alone)',
    )
    stereo_camera_share_dir = get_package_share_directory('stereo_camera')
    full_rviz_config = os.path.join(stereo_camera_share_dir, 'config', 'stereo_lidar.rviz')
    lidar_only_rviz_config = os.path.join(stereo_camera_share_dir, 'config', 'lidar_only.rviz')
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=PythonExpression([
            "'", full_rviz_config, "' if '",
            LaunchConfiguration('show_cameras'), "' == 'true' else '",
            lidar_only_rviz_config, "'",
        ]),
        description='rviz config file to load; defaults to the point cloud + both camera '
                     'images, or just the point cloud when show_cameras:=false',
    )

    camera_node = Node(
        package='stereo_camera',
        executable='stereo_camera_node',
        name='stereo_camera_node',
        output='screen',
        parameters=[stereo_config],
        condition=IfCondition(LaunchConfiguration('enable_cameras')),
    )

    ouster_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ouster_driver_launch),
        launch_arguments={
            'params_file': LaunchConfiguration('ouster_params_file'),
            'ouster_ns': LaunchConfiguration('ouster_ns'),
            # ouster_ros's own viz only shows its own topics; our rviz node
            # below shows the point cloud together with both camera images.
            'viz': 'false',
        }.items(),
    )

    vectornav_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(vectornav_launch),
        condition=IfCondition(LaunchConfiguration('enable_imu')),
    )

    # Strip VSCode's snap confinement env vars for this one process: when
    # this launch file is run from VSCode's integrated terminal (itself
    # installed as a snap), those vars leak into child processes and make
    # rviz2 (Qt-based) dynamically link against an incompatible bundled
    # libpthread from /snap/core20, crashing immediately with a
    # "symbol lookup error". Confirmed empirically on real hardware: rviz2
    # never even appeared as a running process until this was stripped.
    rviz_env = os.environ.copy()
    for var in (
        'SNAP', 'SNAP_LIBRARY_PATH', 'SNAP_NAME', 'SNAP_REVISION',
        'GTK_PATH', 'GTK_EXE_PREFIX', 'GDK_PIXBUF_MODULEDIR',
        'GDK_PIXBUF_MODULE_FILE', 'GIO_MODULE_DIR', 'LOCPATH',
    ):
        rviz_env.pop(var, None)

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        condition=IfCondition(LaunchConfiguration('enable_rviz')),
        output='screen',
        env=rviz_env,
    )

    ouster_ns = LaunchConfiguration('ouster_ns')

    bag_record = ExecuteProcess( #recorded topics in the bag file
        cmd=[
            'ros2', 'bag', 'record',
            '-o', LaunchConfiguration('bag_path'),
            # the camera topics are raw, uncompressed bgr8 at ~20fps from two
            # cameras -- easily hundreds of MB/s uncompressed, so compress on
            # write or short recordings balloon into tens/hundreds of GB.
            '--compression-mode', 'message',
            '--compression-format', 'zstd',
            '/camera/left/image_raw',
            '/camera/right/image_raw',
            ['/', ouster_ns, '/lidar_packets'],
            ['/', ouster_ns, '/points'],
            ['/', ouster_ns, '/imu_packets'],
            ['/', ouster_ns, '/metadata'],
            '/vectornav/imu',
        ],
        output='screen',
    )

    return LaunchDescription([
        bag_path_arg,
        ouster_ns_arg,
        ouster_params_file_arg,
        enable_rviz_arg,
        enable_cameras_arg,
        enable_imu_arg,
        show_cameras_arg,
        rviz_config_arg,
        camera_node,
        ouster_driver,
        vectornav_driver,
        rviz_node,
        bag_record,
    ])
