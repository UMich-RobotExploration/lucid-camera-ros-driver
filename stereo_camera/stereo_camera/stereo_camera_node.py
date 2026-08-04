import rclpy
from rclpy.node import Node

from rclpy.executors import ExternalShutdownException # captures events that shutdown this particular node
from rclpy._rclpy_pybind11 import RCLError
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from cv_bridge import CvBridge

import time
import threading
import multiprocessing as mp
import queue
import cv2
import numpy as np
import ctypes
from arena_api.system import system

# --- module-level helpers used inside the per-camera worker processes ---
# These run in a spawned child process (not this Node's __init__), so they
# take plain picklable arguments instead of `self` — a rclpy Node/CvBridge/etc.
# can't cross a multiprocessing.Process boundary, and each worker owns its own
# exclusive GenTL device handle (GigE devices only allow one open handle per
# process at a time — see README Troubleshooting).

def _buffer_to_frame(buffer, pixel_format):
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


def _configure_transport_layer(device):
    tl = device.tl_stream_nodemap
    tl['StreamBufferHandlingMode'].value    = 'NewestOnly'
    tl['StreamAutoNegotiatePacketSize'].value = True
    tl['StreamPacketResendEnable'].value    = True


def _configure_ptp_role_and_wait(idx, device, is_master, stop_evt):
    device.nodemap['PtpEnable'].value    = True
    device.nodemap['PtpSlaveOnly'].value = not is_master
    target = "Master" if is_master else "Slave"

    print(f"[cam {idx}] waiting for PTP convergence ({target})...", flush=True)
    while device.nodemap['PtpStatus'].value != target:
        if stop_evt.wait(timeout=0.5):
            return
    print(f"[cam {idx}] PTP converged: {target}", flush=True)

    # extra stabilization wait, mirrors the original shared 3s settle period
    stop_evt.wait(timeout=3.0)


# Per-camera transmission-start offset so simultaneous PTP-synced cameras don't
# burst onto the wire at the same instant — Lucid's bandwidth-sharing app note
# formula: base_delay = packet_size * 1e9 / DeviceLinkSpeed(bytes/s), +25%
# buffer. Confirmed empirically: this (via GevSCFTD alone, GevSCPD left at 0)
# is what actually fixed the trigger-drop problem — see README.
GEV_PACKET_SIZE = 1500


def _configure_camera(device, cfg, idx):
    nm = device.nodemap

    nm['PixelFormat'].value = cfg['pixel_format']

    # No Action-command keys or TriggerMode/TriggerSource here — under
    # AcquisitionStartMode=PTPSync the camera firmware generates its own
    # trigger internally (armed via AcquisitionStart), and setting these
    # legacy Action-Command nodes explicitly errors as "not writable" in
    # this mode. This was confirmed by testing: those nodes were never
    # touched in the standalone script that measured 100% delivery.

    # acquisition
    nm['AcquisitionMode'].value = 'Continuous'
    nm['ExposureAuto'].value    = 'Off'
    nm['ExposureTime'].value    = cfg['exposure_us']
    nm['GainAuto'].value        = 'Off'
    nm['Gain'].value            = cfg['gain_db']

    # transmission-start stagger (see note above GEV_PACKET_SIZE)
    link_speed_bps = nm['DeviceLinkSpeed'].value  # bytes/sec
    base_delay_ns  = int(GEV_PACKET_SIZE * 1e9 / link_speed_bps * 1.25)
    nm['GevSCPSPacketSize'].value = GEV_PACKET_SIZE
    nm['GevSCPD'].value           = 0
    nm['GevSCFTD'].value          = base_delay_ns * idx

    # native PTP-synced acquisition — the camera firmware schedules its own
    # periodic frame-start internally once armed via AcquisitionStart,
    # instead of the host broadcasting a per-frame Action Command.
    # AcquisitionFrameRateEnable becomes non-writable once this mode is set,
    # so it must not be touched here.
    nm['AcquisitionStartMode'].value = 'PTPSync'
    max_fps = nm['AcquisitionFrameRate'].max
    nm['AcquisitionFrameRate'].value = max_fps  # avoid capping PTPSyncFrameRate
    nm['PTPSyncFrameRate'].value     = min(cfg['target_fps'], max_fps - 0.01)
    nm['PTPSyncOffset'].value        = 0

    # LUT tone-mapping (brightening curve for low-light scenes; see README)
    nm['LUTFunction'].value              = 'LUTFunctionSigmoid'
    nm['LUTSigmoidThreshold'].value      = cfg['lut_sigmoid_threshold']
    nm['LUTSigmoidStrength'].value       = cfg['lut_sigmoid_strength']
    nm['LUTSigmoidDarkLimit'].value      = cfg['lut_sigmoid_dark_limit']
    nm['LUTSigmoidBrightLimit'].value    = cfg['lut_sigmoid_bright_limit']
    nm['LUTFunctionGenerate'].execute()
    nm['LUTEnable'].value                = cfg['lut_enable']


# Measured empirically on the real hardware: a long device.get_buffer(timeout=...)
# call adds tens of ms of latency even when a frame is already available —
# whatever internal wait/poll granularity the SDK uses scales with the
# requested timeout. Polling with a short timeout instead (and looping on
# TimeoutError) drops per-call latency back to a few ms on both cameras.
POLL_TIMEOUT_MS = 20


def _print_stream_stats(idx, device):
    tl = device.tl_stream_nodemap
    print(
        f"[cam {idx}][gige] delivered={tl['StreamDeliveredFrameCount'].value} "
        f"started={tl['StreamStartedFrameCount'].value} "
        f"lost={tl['StreamLostFrameCount'].value} "
        f"missed_img={tl['StreamMissedImageCount'].value} "
        f"missed_pkt={tl['StreamMissedPacketCount'].value} "
        f"cum_missed_img={tl['StreamCumulativeMissedImageCount'].value} "
        f"cum_incomplete_img={tl['StreamCumulativeIncompleteImageCount'].value}",
        flush=True,
    )


def _enqueue_latest(q, item):
    # mirrors the GenTL NewestOnly semantics the driver already relies on:
    # drop the stale frame rather than block the retrieval loop on a full queue
    try:
        q.put_nowait(item)
    except queue.Full:
        try:
            q.get_nowait()
        except queue.Empty:
            pass
        try:
            q.put_nowait(item)
        except queue.Full:
            pass


def _new_worker_stats():
    return {'count': 0, 'get_buffer': 0.0, 'demosaic': 0.0, 'ipc_put': 0.0,
            'max_get_buffer': 0.0, 'max_demosaic': 0.0, 'max_ipc_put': 0.0}


def _record_frame_timing(idx, cfg, stats, device, gb, dm, put_t):
    stats['count']         += 1
    stats['get_buffer']    += gb
    stats['demosaic']      += dm
    stats['ipc_put']       += put_t
    stats['max_get_buffer'] = max(stats['max_get_buffer'], gb)
    stats['max_demosaic']   = max(stats['max_demosaic'], dm)
    stats['max_ipc_put']    = max(stats['max_ipc_put'], put_t)

    if gb > 0.1 or dm > 0.1 or put_t > 0.1:
        print(f"[cam {idx}][timing] stall: get_buffer={gb*1000:.1f}ms "
              f"demosaic={dm*1000:.1f}ms ipc_put={put_t*1000:.1f}ms", flush=True)

    if stats['count'] >= max(1, int(cfg['target_fps'])):
        n = stats['count']
        print(f"[cam {idx}][timing] avg over {n} frames (ms): "
              f"get_buffer={stats['get_buffer']/n*1000:.1f} (max {stats['max_get_buffer']*1000:.1f}) "
              f"demosaic={stats['demosaic']/n*1000:.1f} (max {stats['max_demosaic']*1000:.1f}) "
              f"ipc_put={stats['ipc_put']/n*1000:.1f} (max {stats['max_ipc_put']*1000:.1f})",
              flush=True)
        _print_stream_stats(idx, device)
        return _new_worker_stats()
    return stats


def _retrieval_loop(idx, device, cfg, stop_evt, frame_queue):
    # Both cameras run the identical loop now — PTPSync mode means the camera
    # firmware generates its own periodic frame-start internally (armed once
    # via AcquisitionStart in _camera_worker_main), so there's no more
    # host-driven trigger-firing to interleave with retrieval.
    stats = _new_worker_stats()
    pixel_format = cfg['pixel_format']
    last_frame_at = time.monotonic()

    while not stop_evt.is_set():
        try:
            t0  = time.perf_counter()
            buf = device.get_buffer(timeout=POLL_TIMEOUT_MS)
            t1  = time.perf_counter()
            ts  = buf.timestamp_ns
            frame, is_color = _buffer_to_frame(buf, pixel_format)
            t2  = time.perf_counter()
            device.requeue_buffer(buf)  # data already copied out; free it ASAP
        except TimeoutError:
            stalled_ms = (time.monotonic() - last_frame_at) * 1000
            if stalled_ms > cfg['buffer_timeout']:
                print(f"[cam {idx}] no frame for {stalled_ms:.0f}ms", flush=True)
                last_frame_at = time.monotonic()  # avoid repeat spam
            continue
        except Exception as e:
            if not stop_evt.is_set():
                print(f"[cam {idx}] capture error: {e}", flush=True)
            continue

        last_frame_at = time.monotonic()
        _enqueue_latest(frame_queue, (ts, frame, is_color))
        t3 = time.perf_counter()
        stats = _record_frame_timing(idx, cfg, stats, device, t1 - t0, t2 - t1, t3 - t2)


def _camera_worker_main(idx, ip, is_master, cfg, ready_evt, start_evt, stop_evt, frame_queue):
    device = None
    try:
        infos = system.device_infos
        info  = next(i for i in infos if i['ip'] == ip)
        device = system.create_device([info])[0]

        _configure_transport_layer(device)
        _configure_ptp_role_and_wait(idx, device, is_master, stop_evt)
        if stop_evt.is_set():
            return
        _configure_camera(device, cfg, idx)
        device.start_stream()
        ready_evt.set()
        print(f"[cam {idx}] stream started", flush=True)

        start_evt.wait()
        device.nodemap['AcquisitionStart'].execute()
        print(f"[cam {idx}] AcquisitionStart executed (PTPSync armed)", flush=True)

        _retrieval_loop(idx, device, cfg, stop_evt, frame_queue)
    finally:
        if device is not None:
            try:
                device.stop_stream()
            except Exception:
                pass
            try:
                system.destroy_device([device])
            except Exception:
                pass
        print(f"[cam {idx}] worker exiting", flush=True)


class stereo_camera_node(Node):
    def __init__(self):
        super().__init__('stereo_camera_node')

        # parameters (see config/stereo_camera.yaml for the launch-time overrides)
        self.declare_parameter('target_fps', 19.8)
        self.declare_parameter('duration_sec', 10.0)
        self.declare_parameter('exposure_us', 30000.0)
        self.declare_parameter('gain_db', 18.0)
        self.declare_parameter('output_dir', '.')
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
        self.gain_db           = self.get_parameter('gain_db').value
        self.output_dir        = self.get_parameter('output_dir').value
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

        self.frame_count = 0
        self.drop_count = 0
        self._pair_lock = threading.Lock()
        self._pending = [None, None]
        self._publish_stats = {'count': 0, 'total': 0.0, 'max': 0.0}

        infos = system.device_infos
        if len(infos) < 2:
            raise RuntimeError(f"Expected 2 cameras, found {len(infos)}")
        infos = infos[:2]
        self.get_logger().info(f"Found {len(infos)} cameras: {[i['ip'] for i in infos]}")

        cfg = {
            'pixel_format':             'BayerRG8',
            'exposure_us':              self.exposure_us,
            'gain_db':                  self.gain_db,
            'lut_enable':               self.lut_enable,
            'lut_sigmoid_threshold':    self.lut_sigmoid_threshold,
            'lut_sigmoid_strength':     self.lut_sigmoid_strength,
            'lut_sigmoid_dark_limit':   self.lut_sigmoid_dark_limit,
            'lut_sigmoid_bright_limit': self.lut_sigmoid_bright_limit,
            'buffer_timeout':           self.buffer_timeout,
            'target_fps':               self.target_fps,
        }

        # Each camera's get_buffer()+demosaic loop runs in its own OS process
        # (not a thread) so the two cameras' blocking SDK calls run in true
        # parallel instead of serializing on the GIL — see README for the
        # measured before/after. Triggering itself is handled entirely by
        # each camera's own firmware (AcquisitionStartMode=PTPSync, armed via
        # start_evt below) — there's no host-driven per-frame trigger loop.
        ctx = mp.get_context('spawn')
        self._stop_event   = ctx.Event()
        self._start_event  = ctx.Event()
        self._ready_events = [ctx.Event(), ctx.Event()]
        # maxsize>1 (not the minimal 1) gives the parent's two reader threads
        # slack to survive a brief GIL stall (e.g. one thread mid-publish)
        # without a frame silently getting overwritten before it's read —
        # see README for the periodic one-period pairing-drop this fixed.
        self._frame_queues = [ctx.Queue(maxsize=4), ctx.Queue(maxsize=4)]

        self._workers = [
            ctx.Process(
                target=_camera_worker_main,
                args=(idx, infos[idx]['ip'], idx == 0, cfg,
                      self._ready_events[idx], self._start_event, self._stop_event,
                      self._frame_queues[idx]),
                daemon=True,
            )
            for idx in range(2)
        ]
        for w in self._workers:
            w.start()

        self.get_logger().info("Waiting for both cameras to configure and PTP-converge...")
        for idx, evt in enumerate(self._ready_events):
            if not evt.wait(timeout=30.0):
                raise RuntimeError(
                    f"Camera {idx} did not become ready within 30s (PTP convergence failed?)"
                )
        self.get_logger().info(f"Both cameras ready — publishing at {self.target_fps} fps")
        self._start_event.set()

        self._reader_threads = [
            threading.Thread(target=self._ipc_reader_loop, args=(idx,), daemon=True)
            for idx in range(2)
        ]
        for t in self._reader_threads:
            t.start()

    def _ipc_reader_loop(self, idx):
        q = self._frame_queues[idx]
        while not self._stop_event.is_set():
            try:
                ts, frame, is_color = q.get(timeout=1.0)
            except queue.Empty:
                continue
            except (EOFError, OSError):
                break
            self._try_publish_pair(idx, ts, frame, is_color)

    def _record_publish_timing(self, dt):
        s = self._publish_stats
        s['count'] += 1
        s['total'] += dt
        s['max']    = max(s['max'], dt)
        if s['count'] >= max(1, int(self.target_fps)):
            self.get_logger().info(
                f"[timing] publish avg over {s['count']} pairs: "
                f"{s['total']/s['count']*1000:.1f}ms (max {s['max']*1000:.1f}ms)"
            )
            self._publish_stats = {'count': 0, 'total': 0.0, 'max': 0.0}

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

        t_pub0 = time.perf_counter()
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
        self._record_publish_timing(time.perf_counter() - t_pub0)

    def destroy_node(self):
        self.get_logger().info("Shutting down. . .")
        self._stop_event.set()
        for w in self._workers:
            w.join(timeout=self.buffer_timeout / 1000 + 2.0)
            if w.is_alive():
                self.get_logger().warn(f"Worker for cam (pid {w.pid}) did not exit cleanly, terminating.")
                w.terminate()
                w.join(timeout=2.0)
        for t in self._reader_threads:
            t.join(timeout=2.0)
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
