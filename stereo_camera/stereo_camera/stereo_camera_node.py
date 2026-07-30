import rclpy
from rclpy.node import Node

from rclpy.executors import ExternalShutdownException # captures events that shutdown this particular node
from rclpy._rclpy_pybind11 import RCLError
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from cv_bridge import CvBridge

import time
import threading
import cv2
import numpy as np
import ctypes
from arena_api.system import system

class stereo_camera_node(Node):
    def __init__(self):
        super().__init__('stereo_camera_node')

        # parameters (see config/stereo_camera.yaml for the launch-time overrides)
        self.declare_parameter('target_fps', 19.8)
        self.declare_parameter('duration_sec', 10.0)
        self.declare_parameter('exposure_us', 50429.312)
        self.declare_parameter('output_dir', '.')
        self.declare_parameter('action_device_key', 1)
        self.declare_parameter('action_group_key', 1)
        self.declare_parameter('action_group_mask', 1)
        self.declare_parameter('schedule_delta', 0.05)
        self.declare_parameter('buffer_timeout', 3000)
        self.declare_parameter('lut_enable', True)
        self.declare_parameter('lut_sigmoid_threshold', 0.005)
        self.declare_parameter('lut_sigmoid_strength', 20.0)
        self.declare_parameter('lut_sigmoid_dark_limit', 0)
        self.declare_parameter('lut_sigmoid_bright_limit', 4095)
        self.declare_parameter('sync_tolerance_us', 5000.0)

        self.target_fps        = self.get_parameter('target_fps').value
        self.duration_sec      = self.get_parameter('duration_sec').value
        self.exposure_us       = self.get_parameter('exposure_us').value
        self.output_dir        = self.get_parameter('output_dir').value
        self.action_device_key = self.get_parameter('action_device_key').value
        self.action_group_key  = self.get_parameter('action_group_key').value
        self.action_group_mask = self.get_parameter('action_group_mask').value
        self.schedule_delta    = self.get_parameter('schedule_delta').value
        self.buffer_timeout    = self.get_parameter('buffer_timeout').value
        self.lut_enable               = self.get_parameter('lut_enable').value
        self.lut_sigmoid_threshold    = self.get_parameter('lut_sigmoid_threshold').value
        self.lut_sigmoid_strength     = self.get_parameter('lut_sigmoid_strength').value
        self.lut_sigmoid_dark_limit   = self.get_parameter('lut_sigmoid_dark_limit').value
        self.lut_sigmoid_bright_limit = self.get_parameter('lut_sigmoid_bright_limit').value
        self.sync_tolerance_us        = self.get_parameter('sync_tolerance_us').value

        # create a publisher for each cam
        self.pub_left = self.create_publisher(Image, '/camera/left/image_raw',10)
        self.pub_right = self.create_publisher(Image, '/camera/right/image_raw',10)
        self.bridge = CvBridge() #CvBridge is a ROS library that provides an interface between ROS and OpenCV

        #setup cameras 
        self.devices = self._discover_devices()
        self.master_cam = self.devices[0]
        self.pixel_formats = []

        #setup the transport layer and configure ptp
        for d in self.devices:
            self._configure_transport_layer(d)
        self._configure_ptp(self.devices)

        for d in self.devices:
            self._configure_camera(d) # set the camera with the desired settings
            self.pixel_formats.append(d.nodemap['PixelFormat'].value)
        
        self.get_logger().info(f"Pixel formats: {self.pixel_formats}")

        for d in self.devices:
            d.start_stream()

        self.get_logger().info(f"Streams started :) - Publishing at {self.target_fps} fps")

        self.frame_count = 0
        self.drop_count = 0

        # Triggering and retrieval run independently so the camera's internal
        # acquisition pipeline stays full (exposure N+1 overlaps transfer of N).
        # A synchronous fire->wait->publish->fire loop caps throughput at
        # ~1/(per-frame latency) instead of the sensor's real max fps.
        # Each camera is fetched by its own dedicated thread (NewestOnly means
        # they can drift onto different trigger cycles independently), and
        # _try_publish_pair only publishes when both timestamps land within
        # sync_tolerance_us of each other, discarding anything that doesn't
        # pair up — this preserves the hardware-sync guarantee under decoupled
        # retrieval.
        self.timer = self.create_timer(1.0 / self.target_fps, self._fire_trigger_tick)

        self._pair_lock = threading.Lock()
        self._pending = [None, None]

        self._stop_event = threading.Event()
        self._retrieval_threads = [
            threading.Thread(target=self._camera_retrieval_loop, args=(idx,), daemon=True)
            for idx in range(len(self.devices))
        ]
        for t in self._retrieval_threads:
            t.start()


    def _discover_devices(self):
        infos = system.device_infos
        if len(infos) < 2:
            raise RuntimeError(f"Expected 2 cameras, found {len(infos)}")
        devices = system.create_device(infos[:2])
        self.get_logger().info(f"Found {len(devices)} cameras")
        return devices


    def _configure_transport_layer(self,device):
        tl = device.tl_stream_nodemap
        tl['StreamBufferHandlingMode'].value    = 'NewestOnly'
        tl['StreamAutoNegotiatePacketSize'].value = True
        tl['StreamPacketResendEnable'].value    = True


    def _configure_ptp(self, devices):
        self.get_logger().info("Configuring PTP...")
        for d in devices:
            is_master = (d == devices[0])
            d.nodemap['PtpEnable'].value    = True
            d.nodemap['PtpSlaveOnly'].value = not is_master
            #role = "Master" if is_master else "Slave"
            #print(f"  Camera {devices.index(d)}: PtpSlaveOnly={not is_master} → expecting {role}")

        self.get_logger().info("Waiting for PTP convergence. . . ")
        for d in devices:
            is_master  = (d == devices[0])
            target     = "Master" if is_master else "Slave"
            while d.nodemap['PtpStatus'].value != target:
                time.sleep(0.5)
            self.get_logger().info(f"Camrea {devices.index(d)}: {target}")
        
        self.get_logger().info("Waiting 3 extra seconds for clocks to fully stabilize...")
        time.sleep(3)


    def _configure_camera(self,device):
        nm = device.nodemap

        # Set pixel format
        nm['PixelFormat'].value = 'BayerRG8'

        # action command keys
        nm['ActionUnconditionalMode'].value = 'On'
        nm['ActionSelector'].value          = 0
        nm['ActionDeviceKey'].value         = self.action_device_key
        nm['ActionGroupKey'].value          = self.action_group_key
        nm['ActionGroupMask'].value         = self.action_group_mask

        # trigger
        nm['TriggerSelector'].value = 'FrameStart'
        nm['TriggerMode'].value     = 'On'
        nm['TriggerSource'].value   = 'Action0'

        # acquisition
        nm['AcquisitionMode'].value            = 'Continuous'
        nm['AcquisitionFrameRateEnable'].value = False
        nm['ExposureAuto'].value               = 'Off'
        nm['ExposureTime'].value               = self.exposure_us

        # LUT tone-mapping (brightening curve for low-light scenes; see README)
        nm['LUTFunction'].value              = 'LUTFunctionSigmoid'
        nm['LUTSigmoidThreshold'].value      = self.lut_sigmoid_threshold
        nm['LUTSigmoidStrength'].value       = self.lut_sigmoid_strength
        nm['LUTSigmoidDarkLimit'].value      = self.lut_sigmoid_dark_limit
        nm['LUTSigmoidBrightLimit'].value    = self.lut_sigmoid_bright_limit
        nm['LUTFunctionGenerate'].execute()
        nm['LUTEnable'].value                = self.lut_enable


    def _fire_action_command(self):
        nm = self.master_cam.nodemap

        #DEBUGGING START
         # check PTP status before firing
        ptp_status_master = nm['PtpStatus'].value
        ptp_status_slave  = self.devices[1].nodemap['PtpStatus'].value
        #DEBUGGING END

        # latch and read master PTP clock
        nm['PtpDataSetLatch'].execute()
        curr_ptp   = nm['PtpDataSetLatchValue'].value
        target_ptp = curr_ptp + int(self.schedule_delta * 1e9)

        # set ALL three keys on the system nodemap — must match camera side
        tl = system.tl_system_nodemap
        tl['ActionCommandDeviceKey'].value  = self.action_device_key
        tl['ActionCommandGroupKey'].value   = self.action_group_key
        tl['ActionCommandGroupMask'].value  = self.action_group_mask
        tl['ActionCommandTargetIP'].value   = 0xFFFFFFFF  # broadcast
        tl['ActionCommandExecuteTime'].value = target_ptp
        tl['ActionCommandFireCommand'].execute()

        #DEBUGGING START
        self.get_logger().info(
            f"PTP master={ptp_status_master} slave={ptp_status_slave} | "
            f"curr={curr_ptp} target={target_ptp} delta={self.schedule_delta}s"
            )
        #DEBUGGING END

        return target_ptp

    def _buffer_to_frame(self, buffer, pixel_format):
        n_pixels    = buffer.width * buffer.height
        n_bytes     = buffer.buffer_size  # use actual size, not assumed

        # read exactly as many bytes as the buffer contains
        raw = np.frombuffer(
            (ctypes.c_ubyte * n_bytes).from_address(
                ctypes.addressof(buffer.pdata.contents)
            ),
            dtype=np.uint8
        )

        if n_bytes == n_pixels * 3:
            # 3 bytes per pixel — color format
            if pixel_format == 'BayerRG8':
                frame    = raw.reshape(buffer.height, buffer.width).copy()
                frame    = cv2.cvtColor(frame, cv2.COLOR_BayerRG2BGR)
                is_color = True
            else:
                # BGR8 or similar
                frame    = raw.reshape(buffer.height, buffer.width, 3).copy()
                is_color = True

        elif n_bytes == n_pixels:
            # 1 byte per pixel
            if 'Bayer' in str(pixel_format):
                frame    = raw.reshape(buffer.height, buffer.width).copy()
                frame    = cv2.cvtColor(frame, cv2.COLOR_BayerRG2BGR)
                is_color = True
            else:
                # Mono8
                frame    = raw.reshape(buffer.height, buffer.width).copy()
                is_color = False

      

        return frame, is_color

    def _fire_trigger_tick(self):
        # Runs on the ROS timer at target_fps. Only fires the broadcast Action
        # Command — retrieval/publish happens independently in _retrieval_loop
        # so this stays fast and keeps the camera's acquisition pipeline full.
        try:
            self._fire_action_command()
        except Exception as e:
            self.get_logger().error(f"Trigger error: {e}")

    def _camera_retrieval_loop(self, idx):
        device       = self.devices[idx]
        pixel_format = self.pixel_formats[idx]
        while not self._stop_event.is_set():
            try:
                buf = device.get_buffer(timeout=self.buffer_timeout)
                ts  = buf.timestamp_ns
                frame, is_color = self._buffer_to_frame(buf, pixel_format)
                device.requeue_buffer(buf)  # data already copied out; free it ASAP
            except Exception as e:
                if not self._stop_event.is_set():
                    self.get_logger().error(f"Capture error (cam {idx}): {e}")
                continue

            self._try_publish_pair(idx, ts, frame, is_color)

    def _try_publish_pair(self, idx, ts, frame, is_color):
        other_idx = 1 - idx
        with self._pair_lock:
            other = self._pending[other_idx]

            if other is None:
                self._pending[idx] = (ts, frame, is_color)
                return

            other_ts, other_frame, other_is_color = other
            delta_us = abs(ts - other_ts) / 1000

            if delta_us > self.sync_tolerance_us:
                # not a match — keep whichever timestamp is newer and wait for its partner
                self.drop_count += 1
                self.get_logger().warn(
                    f"Dropping unmatched frame (cam {idx if ts < other_ts else other_idx}), "
                    f"delta={delta_us:.0f}µs > tolerance={self.sync_tolerance_us:.0f}µs "
                    f"(total drops: {self.drop_count})",
                    throttle_duration_sec=5.0,
                )
                if ts >= other_ts:
                    self._pending[idx] = (ts, frame, is_color)
                    self._pending[other_idx] = None
                else:
                    self._pending[idx] = None
                return

            # matched pair — publish both and clear the slots
            self._pending[0] = None
            self._pending[1] = None

        if self._stop_event.is_set():
            return  # shutting down — avoid publishing into a torn-down context

        pair = {idx: (ts, frame, is_color), other_idx: (other_ts, other_frame, other_is_color)}
        self.frame_count += 1
        if self.frame_count % max(1, int(self.target_fps)) == 0:
            self.get_logger().info(
                f"frame {self.frame_count} | sync delta={delta_us:.2f} µs"
            )

        publishers = [self.pub_left, self.pub_right]
        for i in (0, 1):
            f_ts, f_frame, f_is_color = pair[i]
            header               = Header()
            header.stamp.sec     = int(f_ts // 1_000_000_000)
            header.stamp.nanosec = int(f_ts % 1_000_000_000)
            header.frame_id      = 'left' if i == 0 else 'right'

            encoding = 'bgr8' if f_is_color else 'mono8'
            msg      = self.bridge.cv2_to_imgmsg(f_frame, encoding=encoding)
            msg.header = header
            try:
                publishers[i].publish(msg)
            except RCLError:
                # a SIGINT can invalidate the rcl context between threads
                # faster than _stop_event can be observed; harmless at exit.
                return

    def destroy_node(self):
        self.get_logger().info("Shutting down. . .")
        self.timer.cancel()
        self._stop_event.set()
        for t in self._retrieval_threads:
            t.join(timeout=self.buffer_timeout / 1000 + 1.0)
        for d in self.devices:
            d.stop_stream()
        system.destroy_device(self.devices)
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = stereo_camera_node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
if __name__ == '__main__':
    main()