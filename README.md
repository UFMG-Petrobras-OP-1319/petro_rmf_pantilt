# ptz_mini_hkcamera

ROS 2 drivers for HIKVISION / HIKROBOT cameras: live video plus pan-tilt-zoom control
from the command line.

Contains two ROS 2 packages, `hk_camera` (nodes and launch files) and
`hk_camera_interfaces` (service definitions).

Two different families of hardware are supported, and they use completely different
access paths — pick the one that matches your device:

| Hardware | Interface | Nodes | Needs MVS SDK |
| --- | --- | --- | --- |
| Industrial camera (GigE Vision / USB3, e.g. MV-CS, MV-CA) | MVS SDK | `hk_camera`, `hk_camera_compressed_*`, `hk_camera_server` | Yes |
| Network camera / IP camera (PTZ dome, NVR channel) | RTSP (video) + ISAPI over HTTP (pan-tilt-zoom) | `hk_camera_rtsp_pub`, `hk_camera_ptz` | No |

Network cameras are **not** GigE Vision devices — `MV_CC_EnumDevices` will never
find them. If your camera has a web login page, it is a network camera.

Originally based on
[HIKROBOT-MVS-CAMERA-ROS](https://github.com/luckyluckydadada/HIKROBOT-MVS-CAMERA-ROS)
(ROS 1), ported to ROS 2. The upstream code was tested on Ubuntu 20.04 / ROS 2 Foxy;
the current tree builds and runs on ROS 2 Jazzy.

---

## Quick guide (network camera)

Everything below is meant to be pasted straight into a terminal. It assumes Ubuntu with
ROS 2 already installed, and a HIKVISION network camera reachable on your network.

### 1. Install the dependencies

```bash
sudo apt update
sudo apt install -y git python3-colcon-common-extensions \
  ros-$ROS_DISTRO-cv-bridge ros-$ROS_DISTRO-image-transport \
  ros-$ROS_DISTRO-rviz2 ros-$ROS_DISTRO-tf2-ros \
  ros-$ROS_DISTRO-teleop-twist-keyboard \
  libopencv-dev libcurl4-openssl-dev
```

If `$ROS_DISTRO` is empty, source your ROS 2 installation first:
`source /opt/ros/jazzy/setup.bash` (replace `jazzy` with your distro).

### 2. Clone and build

```bash
git clone https://github.com/MACRO-UFMG/ptz_mini_hkcamera.git ~/ptz_ws
cd ~/ptz_ws
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build
source install/setup.bash
```

If the build prints a warning that the MVS SDK was not found, that is expected and
harmless when you are using a network camera — the RTSP and PTZ nodes are still built.
See [MVS SDK](#mvs-sdk-industrial-cameras-only) if you have an industrial camera.

**Every new terminal** needs the workspace sourced:

```bash
source ~/ptz_ws/install/setup.bash
```

### 3. Check that the camera answers

Replace `192.168.1.64` with your camera IP and `admin`/`PASSWORD` with your credentials:

```bash
ping -c 3 192.168.1.64
curl --digest -u admin:PASSWORD http://192.168.1.64/ISAPI/System/deviceInfo
```

The `curl` command should print an XML block with the model and serial number. If it
returns `401`, the credentials are wrong; if it hangs, the camera is unreachable.

### 4. Set your camera up once

Put your camera IP and password in the config file so you never have to type them again:

```bash
nano ~/ptz_ws/hk_camera/config/network_camera.yaml
```

Edit the three values at the top, then rebuild so the file is installed:

```yaml
/**:
  ros__parameters:
    host: "192.168.1.64"
    username: "admin"
    password: "YOUR_PASSWORD"
```

```bash
cd ~/ptz_ws && colcon build --packages-select hk_camera && source install/setup.bash
```

### 5. Start the camera

This single command starts the video stream, the PTZ control node, the static TF, and
rviz2 with the image already displayed:

```bash
ros2 launch hk_camera view_rtsp_camera_launch.py
```

Leave it running. If the video stutters, use the lower-resolution sub stream by adding
`channel:=102`. Any setting can still be overridden without touching the file:

```bash
ros2 launch hk_camera view_rtsp_camera_launch.py host:=192.168.1.108 password:=OTHER
```

### 6. Move the camera

In a **second terminal**:

```bash
source ~/ptz_ws/install/setup.bash

# pan right at 40% speed
ros2 topic pub -1 /hk_camera_ptz/cmd_vel geometry_msgs/msg/Twist '{angular: {z: 0.4}}'

# pan left
ros2 topic pub -1 /hk_camera_ptz/cmd_vel geometry_msgs/msg/Twist '{angular: {z: -0.4}}'

# tilt up
ros2 topic pub -1 /hk_camera_ptz/cmd_vel geometry_msgs/msg/Twist '{angular: {y: 0.4}}'

# tilt down
ros2 topic pub -1 /hk_camera_ptz/cmd_vel geometry_msgs/msg/Twist '{angular: {y: -0.4}}'

# zoom in
ros2 topic pub -1 /hk_camera_ptz/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.5}}'

# stop
ros2 topic pub -1 /hk_camera_ptz/cmd_vel geometry_msgs/msg/Twist '{}'
```

The camera stops on its own about half a second after the last command, so it never
runs away if you forget the stop.

Drive it with the keyboard instead:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r cmd_vel:=/hk_camera_ptz/cmd_vel
```

`j` / `l` pan, `i` / `,` zoom, `k` stops. Tilt is not reachable this way, because
`teleop_twist_keyboard` only publishes `linear.x` and `angular.z` — use the
`ros2 topic pub` commands above for tilt.

### 7. Check the video topic

```bash
ros2 topic hz /hk_camera/rgb          # frame rate actually being published
ros2 run rqt_image_view rqt_image_view /hk_camera/rgb
```

Full details on every argument, topic, and parameter are in the sections below.

---

## Repository layout

```
ptz_mini_hkcamera/             # repository root (also the colcon workspace)
├── hk_camera/                 # main package (ament_cmake)
│   ├── config/
│   │   ├── network_camera.yaml          # RTSP + PTZ settings  <-- edit this one
│   │   └── industrial_camera.yaml       # MVS industrial camera settings
│   ├── include/hk_camera.hpp  # camera::Camera — MVS SDK wrapper, parameter handling
│   ├── launch/
│   │   ├── view_camera_launch.py        # industrial camera + rviz2
│   │   ├── view_rtsp_camera_launch.py   # network camera + PTZ + rviz2
│   │   ├── take_a_photo_launch.py       # reliable-QoS test publisher
│   │   └── hk_camera_config.py          # helper that merges YAML + command line
│   ├── rviz/hk_camera.rviz    # rviz2 config (Image display on /hk_camera/rgb)
│   └── src/                   # all node sources
└── hk_camera_interfaces/      # service definitions (ament_cmake + rosidl)
    └── srv/TakePhoto.srv
```

---

## Requirements

ROS 2 (Jazzy tested) plus:

```bash
sudo apt install \
  ros-$ROS_DISTRO-cv-bridge \
  ros-$ROS_DISTRO-image-transport \
  ros-$ROS_DISTRO-rviz2 \
  ros-$ROS_DISTRO-tf2-ros \
  libopencv-dev libcurl4-openssl-dev
```

`libcurl` is used by the PTZ node for ISAPI HTTP requests. OpenCV must be built
with FFMPEG support for RTSP streaming (the Ubuntu package is).

### MVS SDK (industrial cameras only)

Download from [hikrobotics.com](https://www.hikrobotics.com) and install to `/opt/MVS`.
The build **auto-detects** it: if `MvCameraControl.h` and `libMvCameraControl` are not
found, all SDK-dependent nodes are skipped with a warning and the rest of the package
still compiles. Override the path with `-DMVS_ROOT=/your/path`.

---

## Build

```bash
cd ~/ptz_ws
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build
source install/setup.bash
```

Build a single package with
`colcon build --packages-select hk_camera_interfaces hk_camera`.

---

## Configuration files

Both launch files take every parameter from a YAML file under
[hk_camera/config/](hk_camera/config/), so settings live in one place instead of being
retyped on the command line.

| File | Used by | Configures |
| --- | --- | --- |
| [network_camera.yaml](hk_camera/config/network_camera.yaml) | `view_rtsp_camera_launch.py` | `hk_camera_rtsp` (video) and `hk_camera_ptz` (pan/tilt/zoom) |
| [industrial_camera.yaml](hk_camera/config/industrial_camera.yaml) | `view_camera_launch.py` | `hk_camera` (MVS industrial camera) |

These are standard ROS 2 parameter files, so they also work directly with a node:

```bash
ros2 run hk_camera hk_camera_ptz --ros-args \
  --params-file install/hk_camera/share/hk_camera/config/network_camera.yaml
```

**How the two nodes share settings.** `network_camera.yaml` has a `/**` section that
applies to every node, holding the connection settings the video and PTZ nodes have in
common, plus one section per node for what differs:

```yaml
/**:                      # applies to both nodes
  ros__parameters:
    host: "192.168.1.64"
    username: "admin"
    password: ""

hk_camera_rtsp:           # video node only
  ros__parameters:
    port: 554             # RTSP port; overrides /** where names collide
    channel: 101

hk_camera_ptz:            # PTZ node only
  ros__parameters:
    port: 80              # ISAPI HTTP port
    channel: 1
```

A node-specific section always wins over `/**`, which is why `port` can mean 554 for the
video node and 80 for the PTZ node in the same file.

**Command line still wins.** Every launch argument defaults to empty, meaning "use the
config file". Pass one and it overrides the file for that run only:

```bash
ros2 launch hk_camera view_rtsp_camera_launch.py channel:=102 password:=OTHER
```

The value is converted to the type of the entry it replaces, so `use_tcp:=false` becomes
a bool and `publish_rate:=15.0` a float rather than strings the node would reject.

Not every parameter has a launch argument — `reconnect_delay`, `max_reconnect_delay`,
`frame_id` and the PTZ scale factors are set in the file only.

**Remember to rebuild** after editing a config file — `colcon build` copies it into
`install/`, and that installed copy is what the launch file reads:

```bash
colcon build --packages-select hk_camera && source install/setup.bash
```

**Using a different file entirely**, for a second camera or to keep credentials out of
the repository:

```bash
cp hk_camera/config/network_camera.yaml ~/my_camera.yaml
# edit ~/my_camera.yaml, then:
ros2 launch hk_camera view_rtsp_camera_launch.py config_file:=~/my_camera.yaml
```

A file passed this way is read from the path you give, so it does not need a rebuild.
Files named `*.local.yaml` are gitignored for exactly this purpose.

---

## Quick start — network camera (RTSP + PTZ)

One command brings up the video stream, the PTZ control node, a static TF, and rviz2:

```bash
ros2 launch hk_camera view_rtsp_camera_launch.py
```

Common variants:

```bash
# sub-stream (lower resolution, lower latency)
ros2 launch hk_camera view_rtsp_camera_launch.py password:=xxx channel:=102
# different camera IP, no rviz
ros2 launch hk_camera view_rtsp_camera_launch.py host:=192.168.1.108 password:=xxx rviz:=false
# video only, no PTZ node
ros2 launch hk_camera view_rtsp_camera_launch.py password:=xxx ptz:=false
```

Launch arguments. The defaults below live in
[network_camera.yaml](hk_camera/config/network_camera.yaml); passing an argument
overrides the file for that run:

| Argument | Default (from config) | Meaning |
| --- | --- | --- |
| `host` | `192.168.1.64` | Camera IP address |
| `port` | `554` | RTSP port |
| `username` / `password` | `admin` / *(empty)* | Login credentials — password is **required** |
| `channel` | `101` | `101` = main stream, `102` = sub stream |
| `rtsp_url` | *(empty)* | Full RTSP URL; overrides host/username/port/channel |
| `use_tcp` | `true` | RTSP over TCP (stable) vs UDP |
| `publish_rate` | `0.0` | Publish rate in Hz; `0` follows the camera frame rate |
| `rviz` / `rviz_config` | `true` / package config | Start rviz2 and which config to load |
| `config_file` | `config/network_camera.yaml` | Parameter file the nodes are configured from |
| `ptz` | `true` | Also start the PTZ control node |
| `http_port` | `80` | ISAPI HTTP port (PTZ) — *not* the RTSP port |
| `ptz_channel` | `1` | ISAPI PTZ channel, normally `1` |

The video is published on `/hk_camera/rgb` (`sensor_msgs/Image`, BGR8), so
`image_transport` republishing, `rqt_image_view`, and rviz2 all work directly.

---

## Mounting the camera upside down

Two things need flipping: the picture and the PTZ directions.

**The picture.** Prefer fixing it in the camera, under **Configuration → Image → Display
Settings → Mirror** in the web UI (`Center` is the 180° option on most models). That costs
nothing on the ROS side and corrects the stream for every client, not just ROS.

If you cannot change the camera, the node can do it. Set `flip_mode` in
[network_camera.yaml](hk_camera/config/network_camera.yaml):

| `flip_mode` | Effect |
| --- | --- |
| `none` *(default)* | Publish frames unchanged |
| `rotate_180` | Upside-down mount — mirrors both axes |
| `horizontal` | Mirror left/right |
| `vertical` | Mirror top/bottom |

```bash
ros2 run hk_camera hk_camera_rtsp_pub --ros-args -p flip_mode:=rotate_180
```

Use `rotate_180`, not `180`: on the command line `-p flip_mode:=180` is parsed as an
integer and the node rejects it with an `InvalidParameterTypeException`. The value `"180"`
does work inside a YAML file, where it is quoted.

This costs one full-frame copy per image, which is a few milliseconds at 1080p — real but
usually not worth worrying about. An unrecognised value logs a warning and publishes
frames unchanged rather than failing.

**The PTZ directions.** Upside down, "pan right" moves the picture left. Invert the axes
by setting the scales negative in the `hk_camera_ptz` section:

```yaml
hk_camera_ptz:
  ros__parameters:
    pan_scale: -1.0
    tilt_scale: -1.0
```

---

## Pan / tilt / zoom control

`hk_camera_ptz` talks to the camera over **ISAPI** (HTTP + Digest auth, port 80),
which is a separate connection from the RTSP video stream. It exposes three topics,
all drivable from the command line.

### Continuous motion — `~/cmd_vel` (`geometry_msgs/Twist`)

| Field | Axis | Sign |
| --- | --- | --- |
| `angular.z` | pan | + = right |
| `angular.y` | tilt | + = up |
| `linear.x` | zoom | + = zoom in |

Values are normalized to `-1.0 … 1.0` and mapped to the ISAPI speed range `-100 … 100`.

```bash
# pan right at 40% speed
ros2 topic pub -1 /hk_camera_ptz/cmd_vel geometry_msgs/msg/Twist '{angular: {z: 0.4}}'
# tilt up and zoom in at the same time
ros2 topic pub -1 /hk_camera_ptz/cmd_vel geometry_msgs/msg/Twist '{angular: {y: 0.3}, linear: {x: 0.5}}'
# stop
ros2 topic pub -1 /hk_camera_ptz/cmd_vel geometry_msgs/msg/Twist '{}'
```

Keyboard teleop works out of the box:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r cmd_vel:=/hk_camera_ptz/cmd_vel
```

Note that `teleop_twist_keyboard` only publishes `linear.x` and `angular.z`, so it drives
pan (`j` / `l`) and zoom (`i` / `,`) but not tilt.

Two behaviors worth knowing:

* **ISAPI continuous motion keeps running until a stop command is sent.** A watchdog
  (`command_timeout`, default 0.5 s) automatically sends a stop if no `cmd_vel` arrives,
  so the camera does not keep spinning if the teleop process dies. The node also stops
  the gimbal on shutdown.
* **Repeated identical commands are dropped.** Teleop nodes resend the same Twist at a
  fixed rate; since ISAPI motion is latching, re-sending only wastes HTTP round-trips.

### Presets — `~/goto_preset` (`std_msgs/Int32`)

```bash
ros2 topic pub -1 /hk_camera_ptz/goto_preset std_msgs/msg/Int32 '{data: 1}'
```

Presets themselves are created in the camera's web UI.

### Absolute positioning — `~/absolute` (`geometry_msgs/Vector3`)

`x` = azimuth in degrees, `y` = elevation in degrees, `z` = zoom multiplier.

```bash
# azimuth 90°, elevation -10°, 2x zoom
ros2 topic pub -1 /hk_camera_ptz/absolute geometry_msgs/msg/Vector3 '{x: 90.0, y: -10.0, z: 2.0}'
```

Internally converted to ISAPI units (0.1°, azimuth 0–3600, elevation ±900, zoom 10–1000).
Only cameras that support absolute positioning will accept this.

### Parameters

Set in the `hk_camera_ptz` section of
[network_camera.yaml](hk_camera/config/network_camera.yaml):

| Parameter | Default | Meaning |
| --- | --- | --- |
| `host` | `192.168.1.64` | Camera IP |
| `port` | `80` | ISAPI HTTP port |
| `username` / `password` | `admin` / *(empty)* | Credentials (Digest, falls back to Basic) |
| `channel` | `1` | ISAPI PTZ channel |
| `pan_scale` / `tilt_scale` / `zoom_scale` | `1.0` | Speed multipliers applied before clamping |
| `command_timeout` | `0.5` | Seconds without `cmd_vel` before auto-stop; `0` disables |
| `http_timeout` | `2.0` | Per-request HTTP timeout in seconds |

Standalone (without the launch file):

```bash
ros2 run hk_camera hk_camera_ptz --ros-args \
  -p host:=192.168.1.64 -p password:=YOUR_PASSWORD
```

---

## Quick start — industrial camera (MVS SDK)

```bash
# raw images + rviz2, all settings from config/industrial_camera.yaml
ros2 launch hk_camera view_camera_launch.py
ros2 launch hk_camera view_camera_launch.py ExposureTime:=20000 FrameRate:=30
ros2 launch hk_camera view_camera_launch.py rviz:=false
```

Or run the nodes directly:

```bash
ros2 run hk_camera hk_camera                  # sensor_msgs/Image on /hk_camera/rgb
ros2 run hk_camera hk_camera_compressed_pub   # sensor_msgs/CompressedImage on /hk_camera/rgb/compressed
ros2 run hk_camera hk_camera_compressed_sub   # subscribes and shows the stream via cv::imshow
```

### Camera parameters

Set in [industrial_camera.yaml](hk_camera/config/industrial_camera.yaml), read by
`camera::Camera` in [hk_camera.hpp](hk_camera/include/hk_camera.hpp). They can also be
passed with `--ros-args -p name:=value`, and `width`, `height`, `ExposureTime`,
`FrameRate` and `GainAuto` are exposed as launch arguments:

| Parameter | Default | Meaning |
| --- | --- | --- |
| `width` / `height` | `1920` / `1200` | Image size |
| `Offset_x` / `Offset_y` | `0` / `0` | ROI offset |
| `ExposureTime` | `50000` | Exposure in microseconds. 50 ms overexposes indoors and caps the rate below 20 Hz — 10000 is a better starting point |
| `FrameRateEnable` / `FrameRate` | `true` / `30` | Acquisition frame rate limit |
| `GainAuto` | `2` | `0` = off, `1` = once, `2` = continuous |
| `GammaEnable` / `Gamma` | `false` / `0.7` | Gamma correction |
| `SaturationEnable` / `Saturation` | `true` / `128` | Saturation |
| `TriggerMode` / `TriggerSource` | `1` / `2` | Trigger configuration |
| `BurstFrameCount` | `10` | Frames acquired per trigger |
| `LineSelector` | `2` | I/O line selection |

---

## Take-photo service

Both camera types can write a still to disk on demand. They share the same service
definition — [TakePhoto.srv](hk_camera_interfaces/srv/TakePhoto.srv):

```
string save_path
---
bool success
string message
```

The directory must already exist; it is not created. `success` is false with the reason in
`message` when the path is unwritable, the extension has no encoder, or no frame is
available yet.

### Network camera (RTSP)

`hk_camera_rtsp_pub` serves `~/take_photo`, which saves the most recent frame it published.
No second connection to the camera is opened, and the `flip_mode` correction is already
applied, so the photo matches what `/hk_camera/rgb` showed.

```bash
ros2 launch hk_camera view_rtsp_camera_launch.py
ros2 service call /hk_camera_rtsp/take_photo hk_camera_interfaces/srv/TakePhoto \
  "{save_path: '/home/user/Pictures/image.jpg'}"
```

The image is whatever the stream carries, so it is the sub-stream resolution when `channel`
is 102. For a full-resolution still independent of the stream, ask the camera directly:

```bash
curl --digest -u admin:PASSWORD \
  "http://192.168.1.64/ISAPI/Streaming/channels/101/picture" -o photo.jpg
```

While the stream is down the service answers immediately with
`The camera stream is disconnected.` rather than blocking for the reconnect backoff.

### Industrial camera (MVS SDK)

`hk_camera_server` grabs a fresh frame from the camera on each call.

```bash
ros2 run hk_camera hk_camera_server     # node name: image_server, service: /save_image
ros2 service call /save_image hk_camera_interfaces/srv/TakePhoto "{save_path: '/home/user/Pictures/image.jpg'}"
```

`hk_camera_client` is a minimal example client for this service. Note that its save path is
currently hard-coded in [hk_camera_client.cpp](hk_camera/src/hk_camera_client.cpp) — edit it
before use.

---

## Node and topic reference

| Executable | MVS SDK | Publishes | Subscribes / Serves |
| --- | --- | --- | --- |
| `hk_camera` | yes | `/hk_camera/rgb` (`Image` + `CameraInfo`) | — |
| `hk_camera_compressed_pub` | yes | `/hk_camera/rgb/compressed` | — |
| `hk_camera_compressed_sub` | yes | — | `/hk_camera/rgb/compressed`, displays with OpenCV |
| `hk_camera_compressed_pub_test` | yes | `/hk_camera/rgb/compressed`, `/hk_camera/strings` (best-effort, depth 1) | — |
| `hk_camera_compressed_sub_test` | yes | — | same topics, best-effort |
| `hk_camera_compressed_pub_test_reliable` | yes | same topics, **reliable** QoS depth 30 | — |
| `hk_camera_compressed_sub_test_reliable` | yes | — | same topics, reliable QoS depth 30 |
| `hk_camera_server` | yes | — | service `/save_image` |
| `hk_camera_client` | no | — | calls `/save_image` |
| `hk_camera_rtsp_pub` | no | `/hk_camera/rgb` (`Image`, BGR8) | service `~/take_photo` |
| `hk_camera_ptz` | no | — | `~/cmd_vel`, `~/goto_preset`, `~/absolute` |

The `*_test` / `*_test_reliable` pairs exist to benchmark DDS throughput (see below);
the `_reliable` variants use reliable QoS with a depth of 30, the others best-effort depth 1.
`take_a_photo_launch.py` is a one-line convenience wrapper that starts
`hk_camera_compressed_pub_test_reliable` on its own:

```bash
ros2 launch hk_camera take_a_photo_launch.py
```

---

## Cyclone DDS tuning

Relevant when streaming full-resolution images across a network link.

### Configure environment

```bash
sudo apt install ros-$ROS_DISTRO-rmw-cyclonedds-cpp
echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> ~/.bashrc
```

### Parameter setting

Set the kernel parameters per the official
[DDS tuning guide](https://docs.ros.org/en/rolling/How-To-Guides/DDS-tuning.html#cross-vendor-tuning):

```bash
sudo sysctl net.ipv4.ipfrag_time=3
sudo sysctl -w net.core.rmem_max=2147483648
sudo sysctl net.ipv4.ipfrag_high_thresh=2147483648
```

Create a `config.xml`:

```xml
<CycloneDDS xmlns="https://cdds.io/config" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:schemaLocation="https://cdds.io/config https://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/master/etc/cyclonedds.xsd">
<Domain id="any">
<Internal>
<MinimumSocketReceiveBufferSize>1024MB</MinimumSocketReceiveBufferSize>
</Internal>
</Domain>
</CycloneDDS>
```

Then:

```bash
export CYCLONEDDS_URI=/path/to/config.xml
```

### Usage

```bash
ros2 run hk_camera hk_camera_compressed_pub_test_reliable
ros2 run hk_camera hk_camera_compressed_sub_test_reliable
```

As tested on an NVIDIA Jetson Xavier NX X509, image transmission at 1920x1200 reaches
20 fps, and the subscription rate hits the publishing rate limit. The camera used tops
out at 40 Hz; with a stronger CPU the publisher should be able to reach that maximum.

![](docs/images/CycloneDDSTuning.png)

Network card upload/download speed is approximately 8 MB/s.

![](docs/images/pc_network_speed.png)
<center>pc network speed</center>

![](docs/images/edge_device_network_speed.png)
<center>edge device network speed</center>

---

## Troubleshooting

**RTSP connection fails / "Failed to read a frame".** Check that the password is correct (HIKVISION
requires Digest auth — an empty password always fails), that the camera is reachable
(`ping`), and that the channel exists. Try `channel:=102` for the sub stream, and
`use_tcp:=false` if the network blocks TCP interleaving. If several attempts already
failed, see the lockout entry below before assuming the password is wrong.

**Everything returns 401 even with the right password — the camera locked you out.**
HIKVISION cameras block a client IP after a few failed logins (five by default), and
answer with an ordinary 401 while blocked, so a lockout is indistinguishable from a bad
password until you ask directly:

```bash
curl --digest -u admin:PASSWORD http://192.168.1.64/ISAPI/Security/userCheck
```

```xml
<userCheck>
  <statusValue>401</statusValue>
  <lockStatus>lock</lockStatus>   <!-- locked -->
  <unlockTime>785</unlockTime>    <!-- seconds remaining -->
</userCheck>
```

**Stop the nodes while a lock is active.** Every further attempt renews it, so retrying
turns a brief mistake into a permanent lockout. Wait out `unlockTime`, or clear it in the
camera web UI under Configuration → System → Security → Illegal Login Lock; a reboot also
clears it. The RTSP node backs off exponentially (`reconnect_delay` doubling up to
`max_reconnect_delay`) so that it does not cause this on its own, but it still cannot
connect while the lock lasts.

**PTZ returns HTTP 401.** Wrong username or password.

**PTZ returns HTTP 403.** The account authenticates but lacks PTZ permission, or ISAPI /
"Open Network Video Interface" is disabled. Both are configured in the camera web UI.

**PTZ commands time out.** ISAPI is on HTTP port 80 (`http_port`), not the RTSP port 554.
Confirm with `curl --digest -u admin:PASSWORD http://CAMERA_IP/ISAPI/System/deviceInfo`.

**The camera keeps moving after Ctrl-C.** Should not happen — the node sends a stop on
shutdown and the watchdog stops it after `command_timeout`. If it does, publish an empty
Twist to `~/cmd_vel` to halt it.

**`MV_CC_EnumDevices` finds no devices.** Either the MVS SDK is not installed, or the
camera is a network camera rather than a GigE Vision industrial camera — use the RTSP
node instead.

**Build warns "HIKROBOT MVS SDK not found".** Expected when the SDK is absent; only the
non-SDK nodes (`hk_camera_rtsp_pub`, `hk_camera_ptz`, `hk_camera_client`) are built.

**rviz2 shows nothing / TF error.** The Fixed Frame must exist in the TF tree. The launch
files publish a static `map` → `hk_camera` transform for this reason; if running nodes
manually, publish it yourself or set the Fixed Frame to `hk_camera`.

---

## Credits

Based on [HIKROBOT-MVS-CAMERA-ROS](https://github.com/luckyluckydadada/HIKROBOT-MVS-CAMERA-ROS).
