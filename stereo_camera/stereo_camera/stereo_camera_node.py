import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Image
from std_msgs.msg import Bool

import array
import collections
import ctypes
import multiprocessing as mp
import os
import queue
import signal
import time
import traceback
from arena_api.system import system

# --- Architecture (see README "Throughput history", point 6) ------------------
# The parent node (stereo_camera_node) only coordinates: it discovers the two
# cameras, assigns the PTP master/slave roles, arms both cameras' PTPSync
# acquisition at the same instant, and logs one aggregated [health] report per
# second. Every camera has its own spawned OS process that owns the exclusive
# GenTL device handle (GigE devices only allow one open handle per process, see
# README Troubleshooting), retrieves buffers, and publishes them itself as raw
# Bayer sensor_msgs/Image. No pixel data ever crosses a process boundary and
# nothing is demosaiced on the host: the 2026-09-24 diagnosis showed the old
# demosaic->pickle->pipe->cv_bridge->publish path through the parent cost ~45 ms
# of a 50.76 ms frame period and silently discarded 14-28% of frames once the
# recorder, zstd and rviz2 shared this thermally throttled CPU.
#
# The workers take plain picklable arguments instead of `self`: an rclpy Node
# cannot cross a multiprocessing.Process boundary.

# GenICam PixelFormat -> (sensor_msgs encoding, bytes per pixel). Only formats
# that need no host-side conversion are listed; anything else is a config error.
ROS_ENCODING = {
    'BayerRG8': ('bayer_rggb8', 1),
    'BayerGR8': ('bayer_grbg8', 1),
    'BayerGB8': ('bayer_gbrg8', 1),
    'BayerBG8': ('bayer_bggr8', 1),
    'Mono8':    ('mono8', 1),
    'BGR8':     ('bgr8', 3),
    'RGB8':     ('rgb8', 3),
}


def _configure_transport_layer(device, cfg):
    tl = device.tl_stream_nodemap
    # OldestFirst (default) queues frames in the announced buffer pool instead
    # of overwriting them: a transient host stall costs latency, not frames, and
    # a real overflow shows up in StreamLostFrameCount instead of vanishing.
    tl['StreamBufferHandlingMode'].value      = cfg['buffer_handling_mode']
    tl['StreamAutoNegotiatePacketSize'].value = True
    tl['StreamPacketResendEnable'].value      = True


def _should_stop(stop_evt, parent_pid):
    # The parent owns shutdown via stop_evt, but if it was killed outright
    # (SIGKILL, crash) we must not sit here forever holding the camera.
    return stop_evt.is_set() or os.getppid() != parent_pid


def _configure_ptp_role_and_wait(idx, device, is_master, stop_evt, parent_pid, log):
    device.nodemap['PtpEnable'].value    = True
    device.nodemap['PtpSlaveOnly'].value = not is_master
    target = "Master" if is_master else "Slave"

    log.info(f"[cam {idx}] waiting for PTP convergence ({target})...")
    while device.nodemap['PtpStatus'].value != target:
        if stop_evt.wait(timeout=0.5) or _should_stop(stop_evt, parent_pid):
            return
    log.info(f"[cam {idx}] PTP converged: {target}")

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

STREAM_STAT_NODES = {
    'delivered':  'StreamDeliveredFrameCount',        # frames the stream engine completed since AcquisitionStart
    'lost':       'StreamLostFrameCount',             # buffer pool underrun: camera sent a frame, no free buffer
    'missed_img': 'StreamMissedImageCount',           # network: whole image missed
    'missed_pkt': 'StreamMissedPacketCount',          # network: packets missed (before resend)
    'incomplete': 'StreamCumulativeIncompleteImageCount',
}


def _stream_stats(device):
    tl = device.tl_stream_nodemap
    return {k: int(tl[node].value) for k, node in STREAM_STAT_NODES.items()}


def _new_counters():
    return {
        'retrieved':      0,  # buffers handed to us by get_buffer()
        'published':      0,  # messages handed to rclpy without error
        'incomplete':     0,  # buffers flagged is_incomplete (published anyway, see README)
        'bad_size':       0,  # buffer smaller than width*height*bpp -> not published
        'publish_failed': 0,  # rclpy publish raised
        'get_errors':     0,  # get_buffer raised something other than TimeoutError
        'gap_missed':     0,  # frames missing between consecutive PTP timestamps (any cause)
        'ts_backwards':   0,  # timestamp went backwards (should never happen)
    }


def _new_timing():
    return {k: [0.0, 0.0, 0] for k in ('get', 'build', 'publish')}  # sum, max, n


def _add_timing(tm, key, dt):
    t = tm[key]
    t[0] += dt
    t[1] = max(t[1], dt)
    t[2] += 1


def _buffer_to_msg(buf, encoding, bpp, frame_id):
    w, h = buf.width, buf.height
    n = w * h * bpp
    if buf.buffer_size < n:
        raise ValueError(f"buffer_size={buf.buffer_size} < {w}x{h}x{bpp}={n} bytes expected for {encoding}")
    src  = (ctypes.c_ubyte * n).from_address(ctypes.addressof(buf.pdata.contents))
    data = array.array('B')
    data.frombytes(memoryview(src))   # the one copy out of the SDK buffer (~0.5 ms for 5.4 MB)
    ts   = buf.timestamp_ns            # camera's PTP-synced hardware clock, not host time

    msg = Image()
    msg.header.stamp.sec     = int(ts // 1_000_000_000)
    msg.header.stamp.nanosec = int(ts % 1_000_000_000)
    msg.header.frame_id      = frame_id
    msg.height        = h
    msg.width         = w
    msg.encoding      = encoding
    msg.is_bigendian  = 0
    msg.step          = w * bpp
    msg.data          = data            # array('B') takes the setter's no-iteration path
    return msg, ts


def _publish_loop(idx, device, cfg, stop_evt, pub, log, stats_queue, period_ns):
    encoding, bpp = ROS_ENCODING[cfg['pixel_format']]
    frame_id   = cfg['frame_id']
    counters   = _new_counters()
    timing     = _new_timing()
    stamps     = []          # timestamps published since the last report, for the parent's sync monitor
    prev_ts    = None
    parent_pid = os.getppid()
    reports_dropped = 0
    now        = time.monotonic()
    last_frame_at = last_report = now

    def report():
        nonlocal timing, stamps
        item = {
            'idx':      idx,
            'counters': dict(counters),
            'stream':   _stream_stats(device),
            'timing':   {k: (v[0] / v[2] * 1000 if v[2] else 0.0, v[1] * 1000) for k, v in timing.items()},
            'stamps':   stamps,
        }
        try:
            stats_queue.put_nowait(item)   # stats only, never frames
        except queue.Full:
            nonlocal reports_dropped
            reports_dropped += 1           # the parent's sync monitor will miss these stamps
        timing = _new_timing()
        stamps = []

    while not stop_evt.is_set():
        now = time.monotonic()
        if now - last_report >= cfg['report_period']:
            report()
            last_report = now
            if os.getppid() != parent_pid:
                # Parent died without setting stop_evt (e.g. SIGKILL). Don't linger
                # holding the camera's exclusive handle — see README Troubleshooting.
                log.error(f"[cam {idx}] parent process is gone, stopping")
                break

        try:
            t0  = time.perf_counter()
            buf = device.get_buffer(timeout=POLL_TIMEOUT_MS)
        except TimeoutError:
            stalled_ms = (time.monotonic() - last_frame_at) * 1000
            if stalled_ms > cfg['buffer_timeout']:
                log.warning(f"[cam {idx}] no frame for {stalled_ms:.0f}ms")
                last_frame_at = time.monotonic()  # avoid repeat spam
            continue
        except Exception as e:
            if not stop_evt.is_set():
                counters['get_errors'] += 1
                log.error(f"[cam {idx}] get_buffer error: {e!r}", throttle_duration_sec=5.0)
            continue
        t1 = time.perf_counter()

        counters['retrieved'] += 1
        last_frame_at = time.monotonic()
        msg = None
        try:
            if buf.is_incomplete:
                counters['incomplete'] += 1
            msg, ts = _buffer_to_msg(buf, encoding, bpp, frame_id)
        except ValueError as e:
            counters['bad_size'] += 1
            log.error(f"[cam {idx}] dropping frame: {e}", throttle_duration_sec=5.0)
        device.requeue_buffer(buf)   # data already copied out (or unusable); free it ASAP
        if msg is None:
            continue
        t2 = time.perf_counter()

        if prev_ts is not None:
            missing = int(round((ts - prev_ts) / period_ns)) - 1
            if missing > 0:
                counters['gap_missed'] += missing
                log.warning(f"[cam {idx}] {missing} frame(s) missing before PTP stamp {ts} "
                            f"(gap {(ts - prev_ts) / 1e6:.1f} ms, total gap_missed={counters['gap_missed']})",
                            throttle_duration_sec=5.0)
            elif missing < 0:
                counters['ts_backwards'] += 1
        prev_ts = ts

        try:
            pub.publish(msg)
            counters['published'] += 1
            stamps.append(ts)
        except Exception as e:
            counters['publish_failed'] += 1
            if not stop_evt.is_set():
                log.error(f"[cam {idx}] publish failed: {e!r}", throttle_duration_sec=5.0)
        t3 = time.perf_counter()

        gb, bd, pb = t1 - t0, t2 - t1, t3 - t2
        _add_timing(timing, 'get', gb)
        _add_timing(timing, 'build', bd)
        _add_timing(timing, 'publish', pb)
        if bd > 0.05 or pb > 0.05:
            log.warning(f"[cam {idx}] stall: build={bd*1000:.1f}ms publish={pb*1000:.1f}ms",
                        throttle_duration_sec=1.0)

    report()
    counters['reports_dropped'] = reports_dropped
    return counters


def _camera_worker_main(idx, ip, is_master, cfg, ready_evt, start_evt, stop_evt, stats_queue):
    # The parent owns shutdown: it sets stop_evt and joins us. Ignore SIGINT so a
    # Ctrl-C delivered to the whole process group can't tear the stream down
    # underneath the parent, and keep rclpy from installing its own handlers.
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    # SIGTERM comes from Process.terminate() (a join that timed out) or from
    # multiprocessing's exit hook for daemon children. Turn it into SystemExit so
    # the finally below still runs stop_stream()/destroy_device() instead of the
    # process dying with the camera's control channel held open.
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(SystemExit(143)))
    # args=[]: do NOT inherit the parent's --ros-args (its __node:= remap and
    # --params-file would otherwise apply to this node too).
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node(cfg['node_name'])
    log  = node.get_logger()
    # Reliable, KEEP_LAST. The depth must cover a full backlog drain: after a
    # host stall OldestFirst hands us up to stream_buffer_count frames at once,
    # and a reliable writer only guarantees delivery of its last `depth`
    # samples, so a depth of 10 lost a frame to the recorder in testing.
    pub  = node.create_publisher(Image, cfg['topic'], cfg['publisher_history_depth'])
    encoding, _ = ROS_ENCODING[cfg['pixel_format']]

    device = None
    counters = None
    parent_pid = os.getppid()
    try:
        infos  = system.device_infos
        info   = next(i for i in infos if i['ip'] == ip)
        device = system.create_device([info])[0]

        _configure_transport_layer(device, cfg)
        _configure_ptp_role_and_wait(idx, device, is_master, stop_evt, parent_pid, log)
        if _should_stop(stop_evt, parent_pid):
            return
        _configure_camera(device, cfg, idx)
        fps       = device.nodemap['PTPSyncFrameRate'].value
        period_ns = int(round(1e9 / fps))
        ready_evt.set()
        log.info(f"[cam {idx}] configured: PTPSyncFrameRate={fps:.3f}, waiting for the joint start")

        # Do not start streaming until BOTH cameras are configured. start_stream()
        # already puts the camera into acquisition, and with OldestFirst a stream
        # nobody retrieves fills its buffer pool in stream_buffer_count frames and
        # the SDK discards everything after that (counted as missed_img/missed_pkt,
        # seen as 76 missed frames when one camera's PTP convergence took 5 s
        # longer than the other's). Starting here keeps both cameras within a few
        # ms of each other and the retrieval loop begins immediately.
        while not start_evt.wait(timeout=0.5):
            if _should_stop(stop_evt, parent_pid):
                return
        device.start_stream(cfg['stream_buffer_count'])
        device.nodemap['AcquisitionStart'].execute()
        log.info(f"[cam {idx}] stream started: {cfg['stream_buffer_count']} buffers, "
                 f"{cfg['buffer_handling_mode']}, PTPSync armed, "
                 f"publishing {cfg['topic']} as {encoding}")

        counters = _publish_loop(idx, device, cfg, stop_evt, pub, log, stats_queue, period_ns)
    except Exception:
        log.error(f"[cam {idx}] worker failed:\n{traceback.format_exc()}")
    finally:
        if device is not None:
            final = None
            try:
                final = _stream_stats(device)
            except Exception:
                pass
            try:
                device.stop_stream()
            except Exception:
                pass
            try:
                system.destroy_device([device])
            except Exception:
                pass
            if counters is not None and final is not None:
                loss = final['delivered'] - counters['published']
                log.info(f"[cam {idx}] final: delivered={final['delivered']} published={counters['published']} "
                         f"not_published={loss} ({100.0 * loss / max(1, final['delivered']):.2f}%) "
                         f"gap_missed={counters['gap_missed']} lost={final['lost']} "
                         f"missed_img={final['missed_img']} missed_pkt={final['missed_pkt']} "
                         f"incomplete={counters['incomplete']} bad_size={counters['bad_size']} "
                         f"publish_failed={counters['publish_failed']} "
                         f"stats_reports_dropped={counters.get('reports_dropped', 0)}")
        # give the recorder's reliable reader a moment to ack the last frames
        # before the writer disappears with them
        wait_acked = getattr(pub, 'wait_for_all_acked', None)
        if wait_acked is not None:
            try:
                wait_acked(timeout_sec=1.0)
            except Exception:
                pass
        log.info(f"[cam {idx}] worker exiting")
        node.destroy_node()
        rclpy.try_shutdown()


class _SyncMonitor:
    """Pairs left/right PTP timestamps reported by the workers (timestamps only,
    never pixel data) so the parent can log whether the cameras are still
    hardware-synced. Nothing here affects what gets published."""

    def __init__(self, tolerance_ns, maxlen=2000):
        self.tol   = tolerance_ns
        self.q     = [collections.deque(maxlen=maxlen), collections.deque(maxlen=maxlen)]
        self.matched   = 0
        self.unmatched = [0, 0]
        self._delta_sum = 0
        self._delta_max = 0
        self._delta_n   = 0

    def add(self, idx, stamps):
        q = self.q[idx]
        evicted = max(0, len(q) + len(stamps) - q.maxlen)
        if evicted:
            # the other camera has been silent for > maxlen frames; these can never pair
            self.unmatched[idx] += evicted
        q.extend(stamps)
        a, b = self.q
        while a and b:
            d = a[0] - b[0]
            if abs(d) <= self.tol:
                self.matched += 1
                self._delta_sum += abs(d)
                self._delta_max  = max(self._delta_max, abs(d))
                self._delta_n   += 1
                a.popleft()
                b.popleft()
            elif d < 0:
                self.unmatched[0] += 1   # left frame with no right partner
                a.popleft()
            else:
                self.unmatched[1] += 1   # right frame with no left partner
                b.popleft()

    def pop_delta_stats(self):
        avg = self._delta_sum / self._delta_n / 1000 if self._delta_n else 0.0
        mx  = self._delta_max / 1000
        self._delta_sum = self._delta_max = self._delta_n = 0
        return avg, mx


class stereo_camera_node(Node):
    def __init__(self):
        super().__init__('stereo_camera_node')

        # parameters (see config/stereo_camera.yaml for the launch-time overrides)
        self.declare_parameter('target_fps', 19.8)
        self.declare_parameter('duration_sec', 10.0)
        self.declare_parameter('exposure_us', 3000.0)
        self.declare_parameter('gain_db', 26.0)
        self.declare_parameter('output_dir', '.')
        self.declare_parameter('buffer_timeout', 3000)
        self.declare_parameter('lut_enable', True)
        self.declare_parameter('lut_sigmoid_threshold', 0.005)
        self.declare_parameter('lut_sigmoid_strength', 20.0)
        self.declare_parameter('lut_sigmoid_dark_limit', 0)
        self.declare_parameter('lut_sigmoid_bright_limit', 4095)
        self.declare_parameter('sync_tolerance_us', 5000.0)
        self.declare_parameter('pixel_format', 'BayerRG8')
        self.declare_parameter('buffer_handling_mode', 'OldestFirst')
        self.declare_parameter('stream_buffer_count', 20)
        self.declare_parameter('publisher_history_depth', 40)
        self.declare_parameter('report_period', 1.0)

        p = lambda name: self.get_parameter(name).value
        self.target_fps        = p('target_fps')
        self.buffer_timeout    = p('buffer_timeout')
        self.sync_tolerance_us = p('sync_tolerance_us')
        self.report_period     = p('report_period')
        pixel_format           = p('pixel_format')
        if pixel_format not in ROS_ENCODING:
            raise RuntimeError(f"pixel_format {pixel_format!r} has no direct sensor_msgs encoding; "
                               f"use one of {sorted(ROS_ENCODING)}")

        infos = system.device_infos
        if len(infos) < 2:
            raise RuntimeError(f"Expected 2 cameras, found {len(infos)}")
        if len(infos) > 2:
            self.get_logger().warn(f"Found {len(infos)} cameras, using the first two: "
                                   f"{[i['ip'] for i in infos[:2]]} (ignored: {[i['ip'] for i in infos[2:]]})")
        infos = infos[:2]
        self.get_logger().info(f"Found {len(infos)} cameras: {[i['ip'] for i in infos]}")

        base_cfg = {
            'pixel_format':             pixel_format,
            'exposure_us':              p('exposure_us'),
            'gain_db':                  p('gain_db'),
            'lut_enable':               p('lut_enable'),
            'lut_sigmoid_threshold':    p('lut_sigmoid_threshold'),
            'lut_sigmoid_strength':     p('lut_sigmoid_strength'),
            'lut_sigmoid_dark_limit':   p('lut_sigmoid_dark_limit'),
            'lut_sigmoid_bright_limit': p('lut_sigmoid_bright_limit'),
            'buffer_timeout':           self.buffer_timeout,
            'target_fps':               self.target_fps,
            'buffer_handling_mode':     p('buffer_handling_mode'),
            'stream_buffer_count':      int(p('stream_buffer_count')),
            'publisher_history_depth':  int(p('publisher_history_depth')),
            'report_period':            self.report_period,
        }
        sides = ('left', 'right')

        ctx = mp.get_context('spawn')
        self._stop_event   = ctx.Event()
        self._start_event  = ctx.Event()
        self._ready_events = [ctx.Event(), ctx.Event()]
        self._stats_queue  = ctx.Queue(maxsize=64)   # small per-second stats dicts, never frames

        self._workers = []
        for idx in range(2):
            cfg = dict(base_cfg,
                       side=sides[idx],
                       topic=f'/camera/{sides[idx]}/image_raw',
                       frame_id=sides[idx],
                       node_name=f'camera_{sides[idx]}')
            self._workers.append(ctx.Process(
                target=_camera_worker_main,
                args=(idx, infos[idx]['ip'], idx == 0, cfg,
                      self._ready_events[idx], self._start_event, self._stop_event,
                      self._stats_queue),
                daemon=True,
            ))
        for w in self._workers:
            w.start()

        try:
            self.get_logger().info("Waiting for both cameras to configure and PTP-converge...")
            deadline = time.monotonic() + 30.0
            for idx, evt in enumerate(self._ready_events):
                while not evt.wait(timeout=1.0):
                    if not self._workers[idx].is_alive():
                        raise RuntimeError(f"Camera {idx} worker exited during setup (see its log above)")
                    if time.monotonic() > deadline:
                        raise RuntimeError(f"Camera {idx} did not become ready within 30s (PTP convergence failed?)")
        except BaseException:
            # __init__ failed (or Ctrl-C): main() has no node to destroy, so stop the
            # workers here — they close their device handles in their own finally.
            self._stop_workers()
            raise
        self.get_logger().info(f"Both cameras configured — starting both streams, {self.target_fps} fps each")
        self._start_event.set()

        self._sync   = _SyncMonitor(int(self.sync_tolerance_us * 1000))
        self._latest = [None, None]
        self._last_health_log = time.monotonic()

        # Streaming status for the recording gate/watchdog (sensor_gate.py), so it
        # never has to subscribe to the images themselves. Published every report
        # period; latched so a late subscriber gets the current state immediately.
        self._streaming = False
        self._status_history = collections.deque(maxlen=3)
        self._status_pub = self.create_publisher(
            Bool, '/stereo_camera/streaming',
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self._status_pub.publish(Bool(data=False))

        self._timer = self.create_timer(0.25, self._poll_stats)

    def _drain_stats(self):
        while True:
            try:
                item = self._stats_queue.get_nowait()
            except queue.Empty:
                break
            except (EOFError, OSError):
                return
            self._latest[item['idx']] = item
            self._sync.add(item['idx'], item['stamps'])

    def _poll_stats(self):
        self._drain_stats()
        if not self._stop_event.is_set():
            for idx, w in enumerate(self._workers):
                if not w.is_alive():
                    raise RuntimeError(f"Camera {idx} worker died (exit code {w.exitcode}); shutting down")

        now = time.monotonic()
        if now - self._last_health_log >= self.report_period and all(self._latest):
            self._last_health_log = now
            self._update_streaming()
            self._log_health()

    def _update_streaming(self):
        # Streaming = both cameras published new frames AND left/right stamps kept
        # pairing. Compared against the snapshot two checks back (~2 report
        # periods): the workers report on their own 1 s clocks, so a single check
        # interval can legitimately contain no new report from one of them.
        published = tuple(self._latest[i]['counters']['published'] for i in range(2))
        self._status_history.append((published, self._sync.matched))
        if len(self._status_history) < self._status_history.maxlen:
            streaming = False
        else:
            (old_pub, old_matched), (new_pub, new_matched) = self._status_history[0], self._status_history[-1]
            streaming = all(new_pub[i] > old_pub[i] for i in range(2)) and new_matched > old_matched
        if streaming != self._streaming:
            self._streaming = streaming
            if streaming:
                self.get_logger().info("Both cameras publishing PTP-paired frames (streaming=True)")
            else:
                self.get_logger().warn("Cameras stopped publishing paired frames (streaming=False)")
        self._status_pub.publish(Bool(data=streaming))

    def _health_summary(self, idx):
        it = self._latest[idx]
        c, s = it['counters'], it['stream']
        not_published = s['delivered'] - c['published']
        return (f"cam{idx} pub={c['published']} deliv={s['delivered']} not_pub={not_published} "
                f"gap={c['gap_missed']} lost={s['lost']} missed_img={s['missed_img']} "
                f"missed_pkt={s['missed_pkt']} incomplete={c['incomplete']} "
                f"bad_size={c['bad_size']} pub_fail={c['publish_failed']}")

    def _log_health(self):
        avg, mx = self._sync.pop_delta_stats()
        self.get_logger().info(
            f"[health] {self._health_summary(0)} | {self._health_summary(1)} | "
            f"sync matched={self._sync.matched} unmatched=L{self._sync.unmatched[0]}/R{self._sync.unmatched[1]} "
            f"delta avg={avg:.1f}µs max={mx:.1f}µs")
        t = [self._latest[i]['timing'] for i in range(2)]
        self.get_logger().info(
            "[timing ms avg/max] " + " | ".join(
                f"cam{i} get={t[i]['get'][0]:.1f}/{t[i]['get'][1]:.0f} "
                f"build={t[i]['build'][0]:.1f}/{t[i]['build'][1]:.0f} "
                f"publish={t[i]['publish'][0]:.1f}/{t[i]['publish'][1]:.0f}" for i in range(2)))

    def _stop_workers(self):
        self._stop_event.set()
        for w in self._workers:
            w.join(timeout=self.buffer_timeout / 1000 + 2.0)
            if w.is_alive():
                self.get_logger().warn(f"Worker for cam (pid {w.pid}) did not exit cleanly, terminating.")
                w.terminate()
                w.join(timeout=2.0)

    def destroy_node(self):
        self.get_logger().info("Shutting down. . .")
        try:
            self._timer.cancel()
        except Exception:
            pass
        self._stop_workers()
        # drain whatever the workers reported on their way out so the final numbers are current
        try:
            self._drain_stats()
        except Exception:
            pass
        if all(self._latest):
            self.get_logger().info(f"[final] {self._health_summary(0)} | {self._health_summary(1)} | "
                                   f"sync matched={self._sync.matched} "
                                   f"unmatched=L{self._sync.unmatched[0]}/R{self._sync.unmatched[1]}")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = stereo_camera_node()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # A terminal delivers Ctrl-C to the whole foreground process group and
        # ros2 launch forwards SIGINT as well, so a second KeyboardInterrupt can
        # land while we are still joining the workers. Ignore it: an aborted
        # teardown leaves a camera handle open (see README Troubleshooting).
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
