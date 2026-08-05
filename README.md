# stereo_camera

ROS2 (Jazzy) driver node for a Lucid Vision Labs TRIO054S-CC stereo pair. Uses PTP
with native `AcquisitionStartMode=PTPSync` acquisition to hardware-synchronize the
two cameras (each camera's own firmware generates its periodic frame-start
internally, once armed), then publishes each frame as `sensor_msgs/Image`.

Camera reference: https://support.thinklucid.com/triton-tri054s/

## Prerequisites

- ROS2 Jazzy (`rclpy`, `sensor_msgs`, `std_msgs`, `cv_bridge` — installed via apt as usual)
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
├── config/stereo_camera.yaml       # camera parameters, keyed under stereo_camera_node
├── launch/stereo_camera.launch.py  # launches the node with config/stereo_camera.yaml
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
| `exposure_us`        | 30000.0   | Fixed exposure time (µs); must fit under the ceiling PTPSync enforces at `target_fps` |
| `gain_db`            | 18.0      | Analog/digital gain (0-42dB); compensates brightness for the shorter exposure above |
| `buffer_timeout`     | 3000      | ms of no frames before logging a stall warning |
| `duration_sec`, `output_dir` | 10.0, "." | Reserved, not currently wired into node logic |
| `lut_enable`                 | true | Enables the on-camera LUT tone-mapping curve |
| `lut_sigmoid_threshold`      | 0.005 | Sigmoid midpoint, normalized 0–1 input. Lower = boosts darker signal |
| `lut_sigmoid_strength`       | 20.0 | Sigmoid steepness (1 = linear, up to 50 = hard step) |
| `lut_sigmoid_dark_limit`, `lut_sigmoid_bright_limit` | 0, 4095 | Output clamp range (12-bit) for the generated LUT |
| `sync_tolerance_us`          | 5000.0 | Max timestamp gap (µs) allowed between left/right before a frame is dropped as unpaired |

Edit `config/stereo_camera.yaml` and re-run (no rebuild needed — it's read at launch
time), or override a single value live: `ros2 param set /stereo_camera_node exposure_us 40000`.

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

With all of this, measured sustained rate in the full node is **~19.8 fps** — the
originally-targeted rate — with 99.85%+ pairing success, zero GigE packet loss, and
sync delta staying in the same single-digit-to-low-tens-of-µs range measured
throughout this whole investigation. `NewestOnly` buffering at the GenTL layer and
the larger host-side frame queue both still mean a frame can occasionally arrive
out of pair; `_try_publish_pair` only publishes when both timestamps land within
`sync_tolerance_us` of each other (unpaired frames are dropped and logged, throttled
to once per 5s).

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

On startup the node (each camera configured in its own OS process — see
Throughput history above):
1. Discovers exactly 2 connected devices (errors if not exactly 2).
2. Configures the transport layer (`NewestOnly` buffering, auto packet size, resend).
3. Enables PTP — camera 0 becomes Master, camera 1 Slave — and waits for convergence
   plus a 3s stabilization period.
4. Configures pixel format, exposure/gain, `GevSCFTD` transmission-start stagger, and
   `AcquisitionStartMode=PTPSync` with `PTPSyncFrameRate=target_fps`.
5. Starts both streams, arms acquisition via `AcquisitionStart` (each camera's
   firmware then generates its own periodic frame-start internally — no per-frame
   host trigger), and each camera's process retrieves+demosaics its own buffers,
   handing them to the parent process for pairing and publishing.

Published topics:
- `/camera/left/image_raw` (`sensor_msgs/Image`, encoding `bgr8`, `frame_id: left`)
- `/camera/right/image_raw` (`sensor_msgs/Image`, encoding `bgr8`, `frame_id: right`)

**Known issue — `ros2 topic hz` reads much lower than the real ~19.7fps rate** (e.g.
~8-9Hz, with the max inter-arrival gap growing over time): at ~16MB/frame uncompressed,
the default kernel UDP send-buffer size (`net.core.wmem_default`, often ~208KB on stock
Ubuntu — check with `sysctl net.core.wmem_max net.core.wmem_default`) is far smaller than
one frame's worth of fragments, so a burst of large messages backs up in the socket layer
independent of anything this node does. This node's own internal counters (GigE
delivered/lost, the `frame N` log line) are the source of truth for the real publish
rate — they stayed clean at ~19.7fps in testing while `ros2 topic hz` degraded. Tried
switching the publishers to best-effort QoS as a fix; that made it *worse* (whole frames
silently dropped on any buffer overflow instead of being retried), so it was reverted —
Reliable is the current, working configuration. The real fix is raising
`net.core.wmem_max`/`wmem_default` to match `rmem_max` (standard ROS2/Fast-DDS tuning
for large messages), not yet applied here.

Frame timestamps come from each camera's PTP-synced hardware clock, not host wall
time. A log line every ~1s of frames reports the inter-camera sync delta (typically
single-digit microseconds once PTP has converged).

## Recording to a bag

With the node running in one terminal:

```bash
ros2 bag record -o stereo_run /camera/left/image_raw /camera/right/image_raw
```

These are full-resolution uncompressed `bgr8` images at ~19.8 fps from two cameras,
so bags grow fast (hundreds of MB/minute). For long recordings, consider
`image_transport`'s compressed plugin.

## Troubleshooting

- **`AccessException ... GC_ERR_ACCESS_DENIED`**: another process (ArenaView GUI,
  a leftover Jupyter kernel, a previous node instance) still has the camera open.
  Close it before starting this node. Since each camera's retrieval now runs in its
  own OS process (see the throughput section above), a node that's killed forcefully
  (e.g. `SIGKILL`, or a crashed launch that doesn't reach `destroy_node()`) can leave
  a camera's worker process alive and still holding its device handle — check
  `ps aux | grep stereo_camera` (and its `multiprocessing.spawn` children) and kill
  any leftover processes before relaunching if this happens.
- **Cameras not enumerating**: see
  https://support.thinklucid.com/knowledgebase/some-of-my-cameras-are-not-enumerating/
- **`ModuleNotFoundError: No module named 'arena_api'`**: the Python wheel isn't
  installed for the Python interpreter ROS2 is using — see Prerequisites above.

## Authors of this Package 

Devin Jones and Nikolas Sanderson
