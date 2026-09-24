# stereo_camera

ROS2 (Jazzy) driver node for a Lucid Vision Labs TRIO054S-CC stereo pair. Uses PTP
with native `AcquisitionStartMode=PTPSync` acquisition to hardware-synchronize the
two cameras (each camera's own firmware generates its periodic frame-start
internally, once armed). Each camera's own process then publishes every frame as a
raw Bayer `sensor_msgs/Image` (`bayer_rggb8`, ~5.4 MB) straight from the SDK buffer:
nothing is demosaiced or copied between processes on the host — debayer at playback
(see Published topics below).

Camera reference: https://support.thinklucid.com/triton-tri054s/

## Prerequisites

- ROS2 Jazzy (`rclpy`, `sensor_msgs` — installed via apt as usual). OpenCV or
  `image_proc` are only needed at playback time to debayer.
- Arena SDK (C++) installed and configured — see Lucid's "Initial Configuration in Linux"
  section, run `sudo sh Arena_SDK_Linux_x64_AVMP.conf`, reboot.
- `arena_api` Python bindings (the wheel bundled with the SDK download, e.g.
  `ArenaPy/arena_api-2.9.5-py3-none-any.whl`). Installing it into the system Python
  that ROS2 uses will hit Debian's PEP 668 guard:

  ```bash
  pip3 install --user --no-deps --break-system-packages \
    /path/to/ArenaPy/arena_api-*-py3-none-any.whl
  ```

  Verify with:

  ```bash
  python3 -c "from arena_api.system import system; print('ok')"
  ```

- Both cameras connected to 2.5G PoE++ ports on the same PoE switch as the host, with
  PTP enabled (handled automatically by the node — see below).

## Directory layout

```
stereo_camera/
├── config/stereo_camera.yaml            # camera parameters, keyed under stereo_camera_node
├── config/*.rviz                        # rviz layouts used by the record launch
├── launch/stereo_camera.launch.py       # cameras only, with config/stereo_camera.yaml
├── launch/stereo_lidar_record.launch.py # cameras + ouster + vectornav + rviz + ros2 bag record
├── scripts/bag_frame_check.py           # validates a recorded bag (counts, gaps, pairing)
├── stereo_camera/stereo_camera_node.py
├── package.xml
└── setup.py
```

Note this package lives directly at `~/ros_ws/stereo_camera` (no `src/` layer) — `colcon
build` run from `~/ros_ws` discovers it automatically.

## Build

```bash
cd ~/ros_ws
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

## Configuration

Parameters (`config/stereo_camera.yaml`, node name `stereo_camera_node`):

| Parameter           | Default    | Meaning                                                  |
|---------------------|-----------|-----------------------------------------------------------|
| `target_fps`         | 19.8      | Capture/publish rate — see the throughput section below for how this was reached |
| `exposure_us`        | 3000.0    | Fixed exposure time (µs); must fit under the ceiling PTPSync enforces at `target_fps` (config/stereo_camera.yaml sets 3000.0; the node's built-in default is the same) |
| `gain_db`            | 26.0      | Analog/digital gain (0-42dB); compensates brightness for the short exposure above |
| `buffer_timeout`     | 3000      | ms of no frames before logging a stall warning |
| `duration_sec`, `output_dir` | 10.0, "." | Reserved, not currently wired into node logic |
| `lut_enable`                 | true | Enables the on-camera LUT tone-mapping curve |
| `lut_sigmoid_threshold`      | 0.005 | Sigmoid midpoint, normalized 0–1 input. Lower = boosts darker signal |
| `lut_sigmoid_strength`       | 20.0 | Sigmoid steepness (1 = linear, up to 50 = hard step) |
| `lut_sigmoid_dark_limit`, `lut_sigmoid_bright_limit` | 0, 4095 | Output clamp range (12-bit) for the generated LUT |
| `sync_tolerance_us`          | 5000.0 | Max left/right stamp gap (µs) for the parent's sync *monitor* to count a pair as matched. A monitor only: every frame is published regardless |
| `pixel_format`               | BayerRG8 | Camera PixelFormat, published as-is (`BayerRG8` → `bayer_rggb8`; also `BayerGR8/GB8/BG8`, `Mono8`, `BGR8`, `RGB8`) |
| `buffer_handling_mode`       | OldestFirst | GenTL output-queue policy. `OldestFirst` keeps every frame the pool can hold and reports a real overflow in `lost`; `NewestOnly` (the old setting) silently overwrote frames whenever the host was late |
| `stream_buffer_count`        | 20     | Buffers announced per camera (20 × 5.4 MB). With `OldestFirst` this is how many frames a host stall can queue before the SDK starts discarding |
| `publisher_history_depth`    | 40     | Reliable KEEP_LAST depth of each image publisher. Keep ≥ 2 × `stream_buffer_count`: after a stall the worker drains its backlog in one burst, and a reliable writer only guarantees its last `depth` samples to the recorder |
| `report_period`              | 1.0    | Seconds between `[health]` log lines |

Edit `config/stereo_camera.yaml` and rebuild (`colcon build --packages-select
stereo_camera`, or build once with `--symlink-install`): the launch files load the
copy under `install/`. All parameters are read once at startup and handed to the
camera processes, so `ros2 param set` on the running node has no effect — restart it.

**Throughput history — how the node reached 19.8 fps synced:** at full 2880x1860
resolution this camera (TRI054S-CC) is rated 20.8 fps free-run, bandwidth-bound by
its 1000BASE-T GigE interface. Getting close to that under *hardware-synced* capture
(not free-run) took several rounds of measurement — kept here because each fix
addressed a real, independently-confirmed bottleneck, and the same failure modes
could resurface if this architecture changes again:

1. **GIL contention across cameras.** The original single-process, two-thread design
   let the two cameras' blocking `device.get_buffer()` calls serialize on the GIL —
   confirmed by isolating one camera in its own process (clean 49ms/call, ~19.7 fps)
   versus both cameras together in a two-thread design (60-90ms/call, ~11-13 fps,
   despite zero GigE-level packet loss). Fix: each camera's retrieval runs in its own
   `multiprocessing.Process`, not a thread.
2. **`get_buffer()` timeout granularity.** A long `get_buffer(timeout=...)` call adds
   real latency even when a frame is already available. Fix: poll with a short
   timeout (`POLL_TIMEOUT_MS = 20`) and loop on `TimeoutError` instead — dropped
   per-call latency from ~80-90ms back to single-digit ms on both cameras.
3. **Trigger-period margin.** With those two fixed, an earlier design using PTP +
   host-broadcast GenICam Action Commands (the host computing `target_ptp = now +
   schedule_delta` on a fixed schedule and broadcasting a scheduled trigger) still
   silently dropped close to half of all triggers at `target_fps≈19.76` — confirmed
   by counting host-side ticks fired (237 in 12s, full rate) against frames actually
   delivered (129, ~54%). Externally-scheduled Action Commands are open-loop: the
   host pre-commits to a future execute time with no feedback on whether the camera's
   real transfer has finished, so a period this close to the sensor's real cycle time
   gets rejected on whichever cycles run longer than average. Ruled out as the fix:
   `TriggerOverlap=PreviousFrame` (no effect — exposure wasn't the bottleneck) and
   shortening `exposure_us` alone (no effect either — confirmed the ceiling was
   transfer-time-bound, not exposure-bound). The working fix at the time was backing
   off to `target_fps=15.0` for enough margin (~98% success) — see below for what
   superseded this.
4. **The actual fix: `AcquisitionStartMode=PTPSync` + `GevSCFTD` transmission-start
   stagger.** Rather than the host broadcasting a trigger per frame, each camera's
   own firmware generates its periodic frame-start internally (once armed via
   `nm['AcquisitionStart'].execute()`), governed by `PTPSyncFrameRate`. This removes
   the open-loop host-scheduling problem in point 3 entirely. Separately, `GevSCFTD`
   (Stream Channel Frame Transmission Delay) staggers each camera's transmission
   *start* instant by `packet_size * 1e9 / DeviceLinkSpeed * 1.25 * camera_index` ns
   (Lucid's bandwidth-sharing app note formula) so the two PTP-synced cameras don't
   burst onto the wire at the exact same moment — `GevSCPD` (continuous inter-packet
   delay) was tested too but cut the achievable ceiling by more than half (bandwidth
   directly traded for collision-avoidance margin) and turned out not to be needed;
   `GevSCFTD` alone got the same reliability without that cost. Measured in isolation:
   100% frame delivery, zero packet loss, tight sync (mean 8.6µs, max 18.2µs) at
   ~19.7fps — up from the ~54% success rate in point 3 at the same target rate.
   Two config values had to change to fit under PTPSync's constraints:
   `exposure_us` dropped from 50429.312 to 30000.0 (the longer value exceeded the
   max exposure PTPSync allows at `target_fps≈19.8`) with `gain_db` raised from 12
   to 18 to compensate for brightness.
5. **A secondary pairing-drop issue, in the real node only.** Integrating (4) into
   the actual node initially showed periodic pairing drops (deltas of almost exactly
   one full frame period, climbing over a test: 1→4→11→25→35) despite each camera
   individually delivering ~19.6fps with zero GigE loss — meaning the sync/trigger
   mechanism was solid but something was dropping frames on the host side. Cause: the
   parent's two `_ipc_reader_loop` threads share one process (and GIL) with `Node`,
   and each one's synchronous `cv_bridge`+`rclpy.publish()` call (~15-25ms) could
   stall the *other* reader thread long enough that its worker's `maxsize=1` frame
   queue silently overwrote a frame before it was ever read. Fix: raised
   `frame_queue` `maxsize` from 1 to 4, giving enough slack to absorb a brief stall
   without a drop. Confirmed: drops went from 35 in one test to 1 in the next
   (~34s window, 665/666 frames paired).

6. **Frame loss during real recordings, and the redesign that removed the host
   pipeline (2026-09-24).** Five-minute recordings with the ouster driver, `ros2 bag
   record` (zstd, 8 threads) and rviz2 running lost 14-28% of camera frames even
   though the GigE counters were perfect (`delivered` = 19.66 fps × span, `lost=0`,
   `missed=0`) and the bag held every message the node published. The loss was
   host-side: the parent's per-pair cost (pipe recv + unpickle + `cv_bridge` +
   `rclpy` publish, all 16 MB bgr8 copies) was ~45 ms of the 50.76 ms period, so once
   the recorder, zstd and rviz2 shared this NUC's thermally throttled CPU (package at
   TjMax 105 °C within seconds of any recording) the 4-deep worker queue filled and
   `_enqueue_latest` silently discarded frames — never logged, never counted, and each
   discard cost 24-78 ms because it read the stale 16 MB frame back through the pipe,
   which in turn made the SDK's `NewestOnly` mode drop more. Once tipped, GIL
   contention between the two reader threads kept the node at ~14 fps for the rest
   of the run. The logged "Dropping unmatched frame" lines were only the orphaned
   partners (delta = exactly one frame period). Reproduced without cameras by a
   synthetic replica of the pipeline. Fix: **no host pipeline at all** — each camera
   process publishes its own topic as raw Bayer (`bayer_rggb8`, 5.4 MB instead of
   16 MB), the parent only coordinates and reports, `OldestFirst` buffering replaces
   `NewestOnly`, and every remaining loss path has a counter (see Health counters
   below). Measured cost per frame in the worker: ~0.6 ms to build the message and
   ~0.6 ms to hand it to `rclpy` with no subscriber attached.

Frame timestamps come from each camera's PTP-synced hardware clock. Left/right
pairing is done at playback by `header.stamp` (both cameras fire on the same PTP
boundaries, so matching stamps land within tens of µs of each other); the parent's
`[health]` line reports the same matching live as a sync monitor.

**Known issue — dark images:** this camera has no raw "Gamma Point" enum — ArenaView's
LUT Tone Mapping dropdown is a GUI abstraction over the `LUTFunctionSigmoid` feature.
`_configure_camera` generates a Sigmoid curve on every startup (params above). The
node now sets `Gain` explicitly (`gain_db`, default 18dB — see Configuration above);
raw sensor mean measured ~71/255 at `exposure_us=30000`/`gain_db=18`, and the
published (LUT-processed) image measured mean=67.7/255, full dynamic range — no
longer underexposed. If images are still dark for your scene, increase `gain_db`
(0-42dB range) or re-tune `lut_sigmoid_threshold`/`lut_sigmoid_strength`; raising
`exposure_us` also helps but costs available `target_fps` margin (see Throughput
history above).

## Running

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros_ws/install/setup.bash

ros2 run stereo_camera stereo_camera_node
# or, to load config/stereo_camera.yaml:
ros2 launch stereo_camera stereo_camera.launch.py
```

On startup the parent node (each camera configured and published from its own OS
process — see Throughput history above):
1. Discovers the connected devices; errors if fewer than 2, warns and uses the first
   two if more.
2. Configures the transport layer (`OldestFirst` buffering, auto packet size, resend).
3. Enables PTP — camera 0 becomes Master, camera 1 Slave — and waits for convergence
   plus a 3s stabilization period.
4. Configures pixel format, exposure/gain, `GevSCFTD` transmission-start stagger, and
   `AcquisitionStartMode=PTPSync` with `PTPSyncFrameRate=target_fps`.
5. Once both are configured, starts both streams and arms acquisition via
   `AcquisitionStart` within a few ms of each other (each camera's firmware then
   generates its own periodic frame-start internally on PTP boundaries — no
   per-frame host trigger). Streaming deliberately does not begin before this joint
   start: `start_stream` already puts a camera into acquisition, and a stream nobody
   retrieves fills its `OldestFirst` pool and is then discarded by the SDK as
   `missed_img`. Each camera's process (ROS node `camera_left` / `camera_right`)
   retrieves its own buffers and publishes them directly. The parent only collects
   the workers' counters and logs `[health]` / `[timing]` lines once per second.

Published topics:
- `/camera/left/image_raw` (`sensor_msgs/Image`, encoding `bayer_rggb8`, `frame_id: left`)
- `/camera/right/image_raw` (`sensor_msgs/Image`, encoding `bayer_rggb8`, `frame_id: right`)

The images are the raw sensor mosaic. To get colour at playback, either run
`image_proc`'s debayer node on the topic, or in Python:

```python
bayer = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width)
bgr   = cv2.cvtColor(bayer, cv2.COLOR_BayerBG2BGR)   # OpenCV's Bayer names are offset by
                                                     # one pixel from GenICam/ROS: rggb8 -> BayerBG2BGR
```

rviz2 renders `bayer_*` encodings as a grayscale mosaic, which is fine as a
"camera alive" check but is one more full-resolution subscriber; prefer
`show_cameras:=false` while recording.

**Health counters.** Once per second the parent logs, per camera,
`pub` (messages handed to rclpy), `deliv` (frames the stream engine completed since
`AcquisitionStart`), `not_pub` (= `deliv - pub`: frames the SDK completed that we
have not published — a transient 1-2 during a host stall is just the backlog still
in the `OldestFirst` queue, but it must return to 0 and the `final:` line must read
`not_published=0`), `gap` (frames missing between consecutive PTP stamps, whatever
the cause),
`lost` (buffer-pool underrun — the camera sent a frame and no free buffer existed),
`missed_img` / `missed_pkt` (network loss before resend), `incomplete` (buffers the
SDK flagged incomplete; they are published and counted, not dropped), `bad_size` and
`pub_fail`; then the sync monitor (`matched`, `unmatched`, left/right stamp delta).
A clean recording has `not_pub=0`, `gap=0`, `lost=0` and `unmatched` not growing
(one unmatched frame at start-up or shutdown is normal: whichever camera is armed
first catches one extra PTP boundary). Frames published in the ~100 ms after the
recorder stops are not in the bag, so `pub` may exceed the bag count by a frame or
two per camera at shutdown. Each worker
logs a `final:` line with the same totals at shutdown, and the parent a `[final]`
line. Compare `pub` with the bag's message count (`ros2 bag info`, or
`scripts/bag_frame_check.py`) to see whether anything was lost between the
publisher and the bag.

**Known issue — `ros2 topic hz` reads much lower than the real ~19.7fps rate** (e.g.
~8-9Hz, with the max inter-arrival gap growing over time): at ~5.4MB/frame uncompressed,
the default kernel UDP send-buffer size (`net.core.wmem_default`, often ~208KB on stock
Ubuntu — check with `sysctl net.core.wmem_max net.core.wmem_default`) is far smaller than
one frame's worth of fragments, so a burst of large messages backs up in the socket layer
independent of anything this node does. This node's own `[health]` counters (`pub`,
`deliv`, `gap`) are the source of truth for the real publish rate — they stayed clean
at ~19.7fps in testing while `ros2 topic hz` degraded. Tried
switching the publishers to best-effort QoS as a fix; that made it *worse* (whole frames
silently dropped on any buffer overflow instead of being retried), so it was reverted —
Reliable is the current, working configuration. The real fix is raising
`net.core.wmem_max`/`wmem_default` to match `rmem_max` (standard ROS2/Fast-DDS tuning
for large messages), not yet applied here.

Frame timestamps come from each camera's PTP-synced hardware clock, not host wall
time. The `[health]` line every second reports the inter-camera sync delta (typically
single-digit microseconds once PTP has converged).

## Recording to a bag

`launch/stereo_lidar_record.launch.py` starts the cameras, the ouster and vectornav
drivers, rviz and `ros2 bag record` (zstd per-message compression) together:

```bash
ros2 launch stereo_camera stereo_lidar_record.launch.py show_cameras:=false
```

or, with the camera node running in one terminal:

```bash
ros2 bag record --compression-mode message --compression-format zstd \
  --compression-queue-size 150 --compression-threads 8 \
  -o stereo_run /camera/left/image_raw /camera/right/image_raw
```

These are full-resolution raw Bayer images at ~19.7 fps from two cameras (~210 MB/s
before compression; a 5-minute run with lidar and IMU is ~20-25 GB zstd-compressed),
so check free disk before a long run.

Both launch files also set `FASTRTPS_DEFAULT_PROFILES_FILE` to
`config/fastdds_large_images.xml`, a Fast DDS profile with a 64 MB shared-memory
segment. With the default segment the 5.4 MB images are fragmented through a handful
of 64 KB slots; whenever the recorder or rviz was slow to release one, the reliable
protocol repaired it (the left topic arrived 30-50 ms late all run long) and the
frames still mid-repair when the recorder stopped were lost — 10 frames at the end
of a 150 s test. With the profile the same test recorded every published frame. If
you start `ros2 bag record` by hand, export the same variable in that shell first.

**Validating a recording.** Test the worst case — back-to-back runs on a hot
machine, with rviz as you would use it in the field — and check three things: the
workers' `final:` lines show `not_published=0` and `gap_missed=0`; the bag holds as
many messages per camera as `pub` reports; and

```bash
python3 stereo_camera/scripts/bag_frame_check.py ~/bags/<bag_dir> --fps 19.7
```

prints `PASS` (no stamp gaps, no duplicates, every left frame paired with a right
frame within `sync_tolerance_us`).

## Troubleshooting

- **`AccessException ... GC_ERR_ACCESS_DENIED`**: another process (ArenaView GUI,
  a leftover Jupyter kernel, a previous node instance) still has the camera open.
  Close it before starting this node. Since each camera's retrieval now runs in its
  own OS process (see the throughput section above), a node that's killed forcefully
  (e.g. `SIGKILL`, or a crashed launch that doesn't reach `destroy_node()`) can leave
  a camera's worker process alive and still holding its device handle. The workers
  now notice a vanished parent within one `report_period` and exit on their own, but
  if a camera still refuses to open, check `ps aux | grep -E "stereo_camera|multiprocessing"`
  and kill any leftover processes before relaunching.
- **Cameras not enumerating**: see
  https://support.thinklucid.com/knowledgebase/some-of-my-cameras-are-not-enumerating/
- **`ModuleNotFoundError: No module named 'arena_api'`**: the Python wheel isn't
  installed for the Python interpreter ROS2 is using — see Prerequisites above.

## Authors of this Package 

Devin Jones and Nikolas Sanderson
