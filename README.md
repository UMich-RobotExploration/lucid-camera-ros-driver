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
| `target_fps`         | 19.8      | Capture/publish rate                                       |
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

**Known issue — throughput below `target_fps`:** at full 2880x1860 resolution, the
camera only sustains 19.8 fps in free-run mode because it pipelines internally
(exposure N+1 overlaps transfer of frame N). A naive trigger→wait→publish→trigger
loop never lets that pipeline fill, capping throughput at ~4.1 fps regardless of
`target_fps`. The node now fires triggers on a fixed timer independently of
retrieval, and retrieves+publishes each camera on its own thread — measured
sustained rate is **~5.5–6.8 fps**, a ~5x improvement, with occasional stalls up to
~1.5s (likely GIL contention between the two threads' demosaic/convert work, or
GigE packet resends on the ~15MB/frame uncompressed images). Because `NewestOnly`
buffering lets each camera's retrieval drift onto different trigger cycles once
decoupled, `_try_publish_pair` only publishes when both timestamps land within
`sync_tolerance_us` of each other (measured sync delta stays in single/low-double-digit
µs; unpaired frames are dropped and logged, throttled to once per 5s). If you need
closer to the full 19.8 fps, the next lever is cutting per-frame data volume —
publish raw `BayerRG8` instead of demosaiced `bgr8` (cuts size and CPU 3x, push
debayering downstream) — not yet implemented.

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

These are full-resolution uncompressed images at ~19.8 fps from two cameras, so bags
grow fast (hundreds of MB/minute). For long recordings, consider recording the raw
Bayer topic instead of the demosaiced BGR8 one, or use `image_transport`'s compressed
plugin.

## Troubleshooting

- **`AccessException ... GC_ERR_ACCESS_DENIED`**: another process (ArenaView GUI,
  a leftover Jupyter kernel, a previous node instance) still has the camera open.
  Close it before starting this node.
- **Cameras not enumerating**: see
  https://support.thinklucid.com/knowledgebase/some-of-my-cameras-are-not-enumerating/
- **`ModuleNotFoundError: No module named 'arena_api'`**: the Python wheel isn't
  installed for the Python interpreter ROS2 is using — see Prerequisites above.

## Authors of this Package 

Devin Jones and Nikolas Sanderson
