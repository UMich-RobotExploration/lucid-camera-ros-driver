"""Recording gate / watchdog for stereo_lidar_record.launch.py.

mode=wait:  exit 0 once both cameras report streaming (PTP-paired frames being
            published) AND lidar packets have been arriving steadily; exit 1 if
            that does not happen within start_timeout. The launch file only
            starts `ros2 bag record` after a 0 exit, so a bag never begins
            before the cameras have synchronized or without the lidar.
mode=watch: runs alongside the recorder; exits 1 as soon as either sensor goes
            quiet, and the launch file shuts everything down (the recorder
            closes the bag cleanly on SIGINT) so a bag never continues without
            the cameras or the lidar.

Camera state comes from stereo_camera_node's small /stereo_camera/streaming
status topic rather than the image topics themselves, so this adds no extra
subscriber to the 5.4 MB images. Lidar packets are subscribed raw (no
deserialization); the callback only records the arrival time.
"""
import sys
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from ouster_sensor_msgs.msg import PacketMsg
from std_msgs.msg import Bool

EXIT_READY = 0
EXIT_FAILED = 1
EXIT_INTERRUPTED = 130


class SensorGate(Node):
    def __init__(self):
        super().__init__('sensor_gate')
        self.declare_parameter('mode', 'wait')
        self.declare_parameter('ouster_ns', 'ouster')
        self.declare_parameter('camera_status_topic', '/stereo_camera/streaming')
        self.declare_parameter('start_timeout', 180.0)   # s, wait mode
        self.declare_parameter('lidar_steady', 1.0)      # s of continuous packets before "ready"
        self.declare_parameter('lidar_timeout', 2.0)     # s without a packet = lidar gone
        self.declare_parameter('camera_timeout', 5.0)    # s without streaming=True = cameras gone
        # watch mode: a freshly started node needs time for DDS discovery before its
        # first message arrives (took >2 s with rviz2 loading the CPU), so the strict
        # timeouts only start once each sensor has been heard from at least once
        self.declare_parameter('watch_startup_grace', 15.0)

        p = lambda name: self.get_parameter(name).value
        self.mode = p('mode')
        if self.mode not in ('wait', 'watch'):
            raise ValueError(f"mode must be 'wait' or 'watch', got {self.mode!r}")
        self.lidar_topic = f"/{p('ouster_ns').strip('/')}/lidar_packets"
        self.start_timeout = float(p('start_timeout'))
        self.lidar_steady = float(p('lidar_steady'))
        self.lidar_timeout = float(p('lidar_timeout'))
        self.camera_timeout = float(p('camera_timeout'))
        self.watch_startup_grace = float(p('watch_startup_grace'))

        self.exit_code = None
        now = time.monotonic()
        self._t0 = now
        self._last_progress = now
        self._lidar_first = None
        self._lidar_last = None
        self._camera_ok_last = None
        self._watch_armed = False

        # best-effort matches the driver's sensor-data QoS and is also compatible
        # with a reliable publisher (use_system_default_qos: true)
        self.create_subscription(PacketMsg, self.lidar_topic, self._on_lidar,
                                 qos_profile_sensor_data, raw=True)
        self.create_subscription(Bool, p('camera_status_topic'), self._on_camera_status,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                            durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_timer(0.2, self._check)
        if self.mode == 'wait':
            self.get_logger().info(
                f"Waiting up to {self.start_timeout:.0f}s for both cameras (PTP-synced and publishing) "
                f"and the lidar ({self.lidar_topic}) before recording starts")
        else:
            self.get_logger().info(
                f"Watching sensors while recording: stops the launch if {self.lidar_topic} is silent "
                f"for {self.lidar_timeout:.1f}s or the cameras stop streaming for {self.camera_timeout:.1f}s")

    def _on_lidar(self, _raw):
        now = time.monotonic()
        if self._lidar_last is None or now - self._lidar_last > self.lidar_timeout:
            self._lidar_first = now   # (re)start the steady-flow window after a gap
        self._lidar_last = now

    def _on_camera_status(self, msg):
        if msg.data:
            self._camera_ok_last = time.monotonic()

    def _lidar_ok(self, now):
        return self._lidar_last is not None and now - self._lidar_last <= self.lidar_timeout

    def _camera_ok(self, now):
        return self._camera_ok_last is not None and now - self._camera_ok_last <= self.camera_timeout

    def _check(self):
        if self.exit_code is not None:
            return
        now = time.monotonic()
        if self.mode == 'wait':
            self._check_wait(now)
        else:
            self._check_watch(now)

    def _check_wait(self, now):
        lidar_ready = self._lidar_ok(now) and now - self._lidar_first >= self.lidar_steady
        camera_ready = self._camera_ok(now)
        if lidar_ready and camera_ready:
            self.get_logger().info(
                f"All required sensors streaming after {now - self._t0:.1f}s — starting the recorder")
            self.exit_code = EXIT_READY
            return
        missing = [name for name, ok in (('cameras', camera_ready), ('lidar', lidar_ready)) if not ok]
        if now - self._t0 > self.start_timeout:
            self.get_logger().error(
                f"Gave up after {self.start_timeout:.0f}s waiting for: {', '.join(missing)} — "
                f"nothing was recorded")
            self.exit_code = EXIT_FAILED
        elif now - self._last_progress >= 5.0:
            self._last_progress = now
            self.get_logger().info(f"Still waiting for: {', '.join(missing)} ({now - self._t0:.0f}s)")

    def _check_watch(self, now):
        in_grace = now - self._t0 <= self.watch_startup_grace
        problems = []
        for name, last, timeout in (('lidar packets', self._lidar_last, self.lidar_timeout),
                                    ('camera streaming status', self._camera_ok_last,
                                     self.camera_timeout)):
            if last is None:
                if not in_grace:
                    problems.append(f"no {name} received in the first "
                                    f"{self.watch_startup_grace:.0f}s of recording")
            elif now - last > timeout:
                problems.append(f"no {name} for {now - last:.1f}s")
        if not self._watch_armed and self._lidar_last is not None and self._camera_ok_last is not None:
            self._watch_armed = True
            self.get_logger().info(f"Receiving both sensors after {now - self._t0:.1f}s — watchdog armed")
        if problems:
            self.get_logger().error(
                f"Required sensor lost ({'; '.join(problems)}) — stopping the recording")
            self.exit_code = EXIT_FAILED


def main(args=None):
    rclpy.init(args=args)
    node = SensorGate()
    code = None
    try:
        while rclpy.ok() and node.exit_code is None:
            rclpy.spin_once(node, timeout_sec=0.1)
        code = node.exit_code
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if code is None:
            # Interrupted (e.g. Ctrl-C). In wait mode this must be non-zero so the
            # launch file never starts a recorder during shutdown.
            code = EXIT_INTERRUPTED if node.mode == 'wait' else EXIT_READY
        node.destroy_node()
        rclpy.try_shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
