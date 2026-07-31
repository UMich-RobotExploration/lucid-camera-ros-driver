# stereo_camera

ROS2 (Jazzy) driver node for a Lucid Vision Labs TRIO054S-CC stereo pair. Uses PTP
and scheduled Action Commands to hardware-synchronize the two cameras, then publishes
each frame as `sensor_msgs/Image`.

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
| `target_fps`         | 15.0      | Capture/publish rate — see the throughput known-issue below for why this isn't higher |
| `exposure_us`        | 50429.312 | Fixed exposure time (µs); must fit within `1/target_fps`   |
| `schedule_delta`     | 0.05      | Seconds ahead (PTP time) to schedule the sync trigger       |
| `buffer_timeout`     | 3000      | ms to wait for a frame buffer before erroring                |
| `action_device_key`, `action_group_key`, `action_group_mask` | 1 | Must match on both cameras for the broadcast Action Command to fire |
| `duration_sec`, `output_dir` | 10.0, "." | Reserved, not currently wired into node logic |
| `lut_enable`                 | true | Enables the on-camera LUT tone-mapping curve |
| `lut_sigmoid_threshold`      | 0.005 | Sigmoid midpoint, normalized 0–1 input. Lower = boosts darker signal |
| `lut_sigmoid_strength`       | 20.0 | Sigmoid steepness (1 = linear, up to 50 = hard step) |
| `lut_sigmoid_dark_limit`, `lut_sigmoid_bright_limit` | 0, 4095 | Output clamp range (12-bit) for the generated LUT |
| `sync_tolerance_us`          | 5000.0 | Max timestamp gap (µs) allowed between left/right before a frame is dropped as unpaired |

Edit `config/stereo_camera.yaml` and re-run (no rebuild needed — it's read at launch
time), or override a single value live: `ros2 param set /stereo_camera_node exposure_us 40000`.

**Known issue — throughput below `target_fps`:** at full 2880x1860 resolution this
camera (TRI054S-CC) is rated 20.8 fps free-run, bandwidth-bound by its 1000BASE-T
GigE interface (BayerRG8 at that resolution/rate is ~111 MB/s, near the line-rate
ceiling). Getting close to that under *hardware-triggered, PTP-synced* capture (as
opposed to free-run) turned out to require fixing three separate, independently
measured problems, in order of how they were found:

1. The original single-process, two-thread design (`_camera_retrieval_loop` on two
   `threading.Thread`s) let the two cameras' blocking `device.get_buffer()` calls
   serialize on the GIL — confirmed by isolating one camera in its own process
   (clean 49ms/call, ~19.7 fps) versus both cameras together in the old two-thread
   design (60-90ms/call, repeated ~100ms stalls, ~11-13 fps, despite zero GigE-level
   packet loss the whole time). Fix: each camera's retrieval now runs in its own
   `multiprocessing.Process`, not a thread, so the two `get_buffer()` calls run in
   true OS-level parallel. The master camera's process also owns the PTP-scheduled
   Action Command trigger, since it's the only process holding a handle that can
   read the master's PTP clock.
2. A long `get_buffer(timeout=...)` call adds real latency even when a frame is
   already available — whatever internal wait granularity the SDK uses scales with
   the requested timeout. Both camera loops now poll with a short timeout
   (`POLL_TIMEOUT_MS = 20`) and loop on `TimeoutError` instead of blocking for the
   full `buffer_timeout`; this alone dropped per-call latency from ~80-90ms back to
   single-digit ms on both cameras. (For the master this also fixes a *second*
   instance of the same GIL issue: a dedicated trigger-firing thread sharing a
   process with a long-blocking `get_buffer()` call gets starved for the same
   reason the two camera threads did in point 1 — so trigger-firing and retrieval
   now interleave on a single thread via the short poll instead.)
3. Even with (1) and (2) fixed, `target_fps: 19.8`/`19.7586` (the sensor's own
   reported `AcquisitionFrameRate.max` at the configured `exposure_us`) left the
   camera itself silently dropping close to half of all Action Command triggers —
   confirmed by counting host-side trigger ticks fired (237 in 12s, i.e. full rate)
   against frames actually delivered (129, ~54%). The externally-triggered exposure
   window doesn't self-pace the way free-run does, so a trigger period this close to
   the exposure time gets rejected by the camera whenever the previous frame's real
   exposure+readout cycle hasn't finished. Backing off to `target_fps: 15.0` (period
   ≈66.7ms vs. ≈50.4ms exposure) raised trigger success to ~98% in isolation; 17fps
   only reached ~67%, so 15fps was kept as the safe, verified value rather than
   bisecting further.

With all three fixed, measured sustained rate in the full node is **~12.3-12.5 fps**
(some further overhead vs. the ~14.7 fps seen in an isolated synthetic test, likely
IPC/ROS-publish cost nibbling at the trigger-timing margin) — a real, stable
improvement over the original architecture's ~11-13 fps, but now clean: GigE
counters stay at zero loss throughout, `get_buffer()` stays in the single-digit-ms
range on both cameras, and hardware sync stays tight (single-digit-µs, same as
before). `NewestOnly` buffering still means each camera's retrieval can drift onto
different trigger cycles; `_try_publish_pair` only publishes when both timestamps
land within `sync_tolerance_us` of each other (unpaired frames are dropped and
logged, throttled to once per 5s). If you need higher throughput than 15fps, the
next levers to try, in order: (a) lower `exposure_us` for more trigger-period
margin if scene lighting allows, (b) publish raw `BayerRG8` instead of demosaiced
`bgr8` to cut per-frame IPC/publish size and CPU ~3x — not yet implemented.

**Known issue — dark images:** this camera has no raw "Gamma Point" enum — ArenaView's
LUT Tone Mapping dropdown is a GUI abstraction over the `LUTFunctionSigmoid` feature.
`_configure_camera` now generates a Sigmoid curve on every startup (params above) tuned
against a scene where the raw pre-LUT signal measured mean=1.99/255, max=64/255 (with
`LUTEnable` off) — i.e. genuinely underexposed at the sensor, not a display artifact.
The Sigmoid curve is a cosmetic brightening of whatever signal exists; it cannot recover
detail that isn't there. If images are still dark, the real fix is more Gain (the node
doesn't currently set it — `Gain` sits at 0dB), more exposure (trades off `target_fps`),
or more scene light. Re-tune `lut_sigmoid_threshold`/`lut_sigmoid_strength` if the actual
signal level differs from the above.

## Running

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros_ws/install/setup.bash

ros2 run stereo_camera stereo_camera_node
# or, to load config/stereo_camera.yaml:
ros2 launch stereo_camera stereo_camera.launch.py
```

On startup the node:
1. Discovers exactly 2 connected devices (errors if not exactly 2).
2. Configures the transport layer (`NewestOnly` buffering, auto packet size, resend).
3. Enables PTP — camera 0 becomes Master, camera 1 Slave — and waits for convergence
   plus a 3s stabilization period.
4. Configures pixel format, Action Command keys, and trigger/exposure settings.
5. Starts both streams and a timer that fires a broadcast Action Command each cycle,
   grabs both buffers, and publishes them.

Published topics:
- `/camera/left/image_raw` (`sensor_msgs/Image`, `frame_id: left`)
- `/camera/right/image_raw` (`sensor_msgs/Image`, `frame_id: right`)

Frame timestamps come from each camera's PTP-synced hardware clock, not host wall
time. A log line every ~1s of frames reports the inter-camera sync delta (typically
single-digit microseconds once PTP has converged).

## Recording to a bag

With the node running in one terminal:

```bash
ros2 bag record -o stereo_run /camera/left/image_raw /camera/right/image_raw
```

These are full-resolution uncompressed images at ~15 fps from two cameras, so bags
grow fast (hundreds of MB/minute). For long recordings, consider recording the raw
Bayer topic instead of the demosaiced BGR8 one, or use `image_transport`'s compressed
plugin.

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
