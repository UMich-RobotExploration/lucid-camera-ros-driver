import os
from datetime import datetime

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.actions import LogInfo, RegisterEventHandler, Shutdown
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
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
        description='launch the stereo_camera_node; set to false to preview the lidar alone '
                     '(e.g. while debugging it without the cameras attached) -- nothing is '
                     'recorded then, since every bag must contain both cameras and the lidar',
    )
    sensor_start_timeout_arg = DeclareLaunchArgument(
        'sensor_start_timeout',
        default_value='180',
        description='seconds to wait for both cameras (PTP-synced, publishing) and the lidar '
                    'before giving up without recording',
    )
    enable_imu_arg = DeclareLaunchArgument(
        'enable_imu',
        default_value='true',
        description='launch the vectornav IMU driver',
    )
    show_cameras_arg = DeclareLaunchArgument(
        'show_cameras',
        default_value='true',
        description='include the camera Image displays in rviz (raw Bayer renders as a '
                     'grayscale mosaic); set to false to view only the lidar point cloud, '
                     'which also removes a full-resolution image subscriber during recording',
    )
    stereo_camera_share_dir = get_package_share_directory('stereo_camera')
    # Fast DDS shared-memory profile with a 64 MB segment: with the default
    # segment the 5.4 MB images are fragmented through a few 64 KB slots, the
    # reliable protocol keeps repairing dropped fragments (the left topic arrived
    # 30-50 ms late all run long) and the last ~0.4 s of frames were lost when
    # the recorder stopped. Applies to every process this launch starts.
    fastdds_profile = SetEnvironmentVariable(
        'FASTRTPS_DEFAULT_PROFILES_FILE',
        os.path.join(stereo_camera_share_dir, 'config', 'fastdds_large_images.xml'),
    )
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
            # the camera topics are raw Bayer (bayer_rggb8, ~5.4 MB/frame) at
            # ~20fps from two cameras -- ~210 MB/s uncompressed, so compress on
            # write or short recordings balloon into tens of GB. Demosaic at
            # playback (see README "Published topics").
            '--compression-mode', 'message', #other mode is file, which compresses the entire bag file at once
            '--compression-format', 'zstd',
            # default compression-queue-size is 1 -- with message-mode
            # compression thFt means a single momentary stall in keeping up
            # with two ~20fps uncompressed camera streams causes the next
            # message to be dropped outright instead of just queued. Give it
            # real headroom (system has 14 cores / 46GB RAM to spare) and
            # make sure it's actually using multiple threads.
            '--compression-queue-size', '150',
            '--compression-threads', '8',
            # the recorder now starts after the driver has already published its
            # one latched metadata message; subscribe transient_local so the bag
            # still gets it (lidar_packets are unusable without it)
            '--qos-profile-overrides-path',
            os.path.join(ouster_share_dir, 'config', 'metadata-qos-override.yaml'),

            '/camera/left/image_raw',
            '/camera/right/image_raw',
            ['/', ouster_ns, '/lidar_packets'],
           #['/', ouster_ns, '/points'],
            ['/', ouster_ns, '/imu_packets'],
            ['/', ouster_ns, '/metadata'],
            '/vectornav/imu',
        ],
        output='screen',
    )

    # Recording is gated: sensor_gate (mode=wait) exits 0 only once both cameras
    # are PTP-synced and publishing paired frames AND lidar packets are flowing.
    # Only then do the recorder and the watchdog start. Any required sensor
    # failing to come up, or dropping out mid-recording, shuts the whole launch
    # down, and the recorder closes the bag cleanly on the way out.
    sensor_gate = Node(
        package='stereo_camera',
        executable='sensor_gate',
        name='sensor_gate',
        output='screen',
        parameters=[{'mode': 'wait', 'ouster_ns': ouster_ns,
                     # float() so "60" isn't handed to a double parameter as an int
                     'start_timeout': PythonExpression(
                         ['float(', LaunchConfiguration('sensor_start_timeout'), ')'])}],
        condition=IfCondition(LaunchConfiguration('enable_cameras')),
    )
    sensor_watchdog = Node(
        package='stereo_camera',
        executable='sensor_gate',
        name='sensor_watchdog',
        output='screen',
        parameters=[{'mode': 'watch', 'ouster_ns': ouster_ns}],
    )

    def shutdown_unless_already(reason):
        # during a normal Ctrl-C every process exits, and a second Shutdown while
        # one is already in progress makes launch_ros log spurious errors
        def handler(event, context):
            if not context.is_shutdown:
                return [Shutdown(reason=reason)]
        return handler

    def start_recording_when_ready(event, context):
        if context.is_shutdown:
            return None
        if event.returncode == 0:
            return [bag_record, sensor_watchdog]
        return [Shutdown(reason=f'sensor_gate exited with code {event.returncode}: both cameras '
                                'and the lidar were not all streaming; nothing was recorded')]

    recording_handlers = [
        RegisterEventHandler(OnProcessExit(target_action=sensor_gate,
                                           on_exit=start_recording_when_ready)),
        RegisterEventHandler(OnProcessExit(
            target_action=sensor_watchdog,
            on_exit=shutdown_unless_already('sensor watchdog: a required sensor stopped streaming'))),
        RegisterEventHandler(OnProcessExit(
            target_action=bag_record,
            on_exit=shutdown_unless_already('ros2 bag record exited'))),
        RegisterEventHandler(OnProcessExit(
            target_action=camera_node,
            on_exit=shutdown_unless_already('stereo_camera_node exited'))),
    ]
    no_recording_notice = LogInfo(
        msg='enable_cameras:=false -- lidar-only preview, NOT recording '
            '(every bag must contain both cameras and the lidar)',
        condition=UnlessCondition(LaunchConfiguration('enable_cameras')),
    )

    return LaunchDescription([
        fastdds_profile,
        bag_path_arg,
        ouster_ns_arg,
        ouster_params_file_arg,
        enable_rviz_arg,
        enable_cameras_arg,
        sensor_start_timeout_arg,
        enable_imu_arg,
        show_cameras_arg,
        rviz_config_arg,
        camera_node,
        ouster_driver,
        vectornav_driver,
        rviz_node,
        *recording_handlers,
        sensor_gate,
        no_recording_notice,
    ])
