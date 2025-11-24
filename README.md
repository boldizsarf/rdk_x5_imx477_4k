# IMX477 4K + 1080p Pipeline for Horizon RDK X5

This repository contains a complete camera + vision pipeline for the Horizon Robotics RDK X5 using an IMX477 camera.

It provides:

- A custom C++ camera driver node that grabs **4K NV12** frames from the IMX477 via `sp_vio`.
- A VPS (Video Processing Subsystem) resizer node that downscales 4K → **1920×1080 NV12** using hardware scaling.
- A YOLO-based detection pipeline using the vendor `dnn_node_example` package on the **1080p** stream.
- Two hardware-accelerated JPEG encoder nodes (`hobot_codec`) that publish **1080p** and **4K** JPEG topics.
- A websocket node that serves an MJPEG stream plus detections to the **TogetheROS Web UI**.

The goal is to have:

- 4K raw NV12 available for high-resolution consumers.
- 1080p NV12 for real‑time DNN inference.
- 1080p JPEG for the browser UI.
- 4K JPEG for future consumers (e.g. cropper / recorder).

---

## Repository Layout

```text
rdk_x5_imx477_4k/
├── pipeline/
│   └── src/
│       ├── imx477_dual_camera/
│       │   ├── launch/
│       │   │   └── dual_camera.launch.py
│       │   ├── src/
│       │   │   └── imx477_dual_camera_node.cpp
│       │   ├── CMakeLists.txt
│       │   └── package.xml
│       └── imx477_vps_resizer/
│           ├── launch/
│           │   └── vps_resizer.launch.py
│           ├── src/
│           │   └── vps_resizer_node.cpp
│           ├── CMakeLists.txt
│           └── package.xml
├── python/
│   └── dnn_imx477_1080.launch.py
└── services/
    └── imx477-pipeline.service
```

### Packages & files

#### `imx477_dual_camera`

- C++ ROS 2 node that:
    - Initializes the IMX477 via `sp_init_vio_module` / `sp_open_camera_v2`.
    - Grabs **4K NV12** frames using `sp_vio_get_yuv`.
    - Publishes them as `sensor_msgs/msg/Image` on `/imx477/image_raw_4k`.
    - Uses **RELIABLE SensorData QoS** so downstream hardware codecs and nodes can subscribe without QoS errors.

Key topic:

- `/imx477/image_raw_4k` — `sensor_msgs/Image`
    - `width = 4000`, `height = 3000`
    - `encoding = "nv12"`
    - `step = width`
    - QoS: `SensorDataQoS` with `RELIABLE` reliability

#### `imx477_vps_resizer`

- C++ ROS 2 node that:
    - Subscribes to `/imx477/image_raw_4k` (4K NV12).
    - Feeds frames into VPS via `sp_vio_set_frame` / `sp_vio_get_frame`.
    - Produces a **1920×1080 NV12** downscaled frame.
    - Publishes it as `sensor_msgs/msg/Image` on `/imx477/image_raw_1080`.
    - Uses **RELIABLE SensorData QoS** both for subscription and publication.

Key topic:

- `/imx477/image_raw_1080` — `sensor_msgs/Image`
    - `width = 1920`, `height = 1080`
    - `encoding = "nv12"`
    - `step = width`
    - QoS: `SensorDataQoS` with `RELIABLE` reliability

#### `python/dnn_imx477_1080.launch.py`

Launch file that:

- Starts the vendor **YOLO example node** (`dnn_node_example`) configured for 1920×1080 input.
- Remaps the image input to `/imx477/image_raw_1080` so the DNN works on the downscaled stream.
- Starts two `hobot_codec` encoder nodes:
    - `codec_1080_nv12_to_jpeg`: `/imx477/image_raw_1080` → `/image_jpeg` (1080p JPEG)
    - `codec_4k_nv12_to_jpeg`: `/imx477/image_raw_4k`   → `/image_jpeg_4k` (4K JPEG)
- Starts the **websocket** node which:
    - Subscribes to `/image_jpeg` (1080p MJPEG stream).
    - Subscribes to `/hobot_dnn_detection` for overlay metadata.
    - Serves the annotated stream to the TogetheROS web dashboard.

The launch file is meant to be installed to:

```text
/opt/tros/humble/share/dnn_node_example/launch/dnn_imx477_1080.launch.py
```

…and then used instead of the default `dnn_imx477_1080.launch.py`.

#### `services/imx477-pipeline.service`

A systemd unit that:

- Runs at boot.
- Starts the entire pipeline:
    - IMX477 dual camera node.
    - VPS resizer node.
    - YOLO + hobot\_codec + websocket via `dnn_imx477_1080.launch.py`.
- Ensures the ROS 2 environment and the custom workspace are sourced before launching.

Typically, `ExecStart` points to a small shell script (e.g. `/opt/vision/imx477_pipeline.sh`) that performs:

1. `source /opt/tros/humble/setup.bash`
2. `source ~/dual_cam_ws/install/setup.bash`
3. Launch of the ROS 2 pipeline.

---

## Data Flow / Architecture

High‑level data flow:

```text
IMX477 sensor
   │  (4K NV12 via sp_vio)
   ▼
[imx477_dual_camera_node]
   publishes /imx477/image_raw_4k (4000x3000, nv12)
   │
   ├────────────► [hobot_codec: codec_4k_nv12_to_jpeg]
   │                    └─► /image_jpeg_4k (4K JPEG)
   │
   └────────────► [imx477_vps_resizer_node]  (VPS hardware scale)
                          │
                          └─► /imx477/image_raw_1080 (1920x1080, nv12)
                                      │
                                      ├─► [dnn_node_example (YOLO)]
                                      │       └─► /hobot_dnn_detection
                                      │
                                      └─► [hobot_codec: codec_1080_nv12_to_jpeg]
                                              └─► /image_jpeg (1080p JPEG)

/image_jpeg_4k + /hobot_dnn_detection
   │
   └─► [websocket node] ──► TogetheROS Web UI (MJPEG with overlays)
```

Note:

- The **DNN** operates purely on the 1080p NV12 stream.
- The **4K stream** is published both as NV12 and JPEG for downstream consumers that need full resolution.
- The **web UI** shows the 1080p JPEG stream because it is lighter and easier to display in browsers.

---

## Installation & Setup (Fresh System)

The instructions below assume:

- You are on an RDK X5 running the TogetherROS **Humble** image.
- Vendor packages like `dnn_node_example`, `hobot_codec`, `websocket`, and the IMX477 driver are already installed in `/opt/tros/humble`.
- You want to use a ROS 2 workspace called `~/dual_cam_ws`.

### 1. Clone the repository

```bash
cd ~
# Adapt URL once the repo is public
git clone https://github.com/boldizsarf/rdk_x5_imx477_4k.git
```

### 2. Create the ROS 2 workspace and copy the packages

```bash
mkdir -p ~/dual_cam_ws/src
cd ~/dual_cam_ws/src

# Copy the two C++ packages from the repo
cp -r ~/rdk_x5_imx477_4k/pipeline/src/imx477_dual_camera .
cp -r ~/rdk_x5_imx477_4k/pipeline/src/imx477_vps_resizer .
```

At this point, your workspace should look like:

```text
~/dual_cam_ws/
  └── src/
      ├── imx477_dual_camera/
      └── imx477_vps_resizer/
```

### 3. Build the camera & VPS packages

```bash
cd ~/dual_cam_ws/

# Source TogetherROS environment
source /opt/tros/humble/setup.bash

# Build each package (these commands are fixed in this pipeline)
colcon build --packages-select imx477_dual_camera
colcon build --packages-select imx477_vps_resizer
```

After a successful build, overlay the workspace:

```bash
source ~/dual_cam_ws/install/setup.bash
```

You will normally want the two `source` commands whenever you open a new shell:

```bash
source /opt/tros/humble/setup.bash
source ~/dual_cam_ws/install/setup.bash
```

### 4. Install the YOLO + codec + websocket launch file

Copy the launch file into the vendor `dnn_node_example` package:

```bash
sudo cp ~/rdk_x5_imx477_4k/python/dnn_imx477_1080.launch.py \
  /opt/tros/humble/share/dnn_node_example/launch/
```

This file will be used instead of the default `dnn_imx477_1080.launch.py`.

### 5. Install the systemd service

Copy the service file into systemd and enable it:

```bash
sudo cp ~/rdk_x5_imx477_4k/services/imx477-pipeline.service \
  /etc/systemd/system/

sudo systemctl daemon-reload
sudo systemctl enable imx477-pipeline.service
sudo systemctl start imx477-pipeline.service
```

> **Note:** If your paths differ (e.g. workspace not in `~/dual_cam_ws` or launch script not in `/opt/vision/imx477_pipeline.sh`), edit `services/imx477-pipeline.service` before copying so that `ExecStart=` points to the correct script or `ros2 launch` invocation.

### 6. Manual run without systemd (for debugging)

To run everything manually from a terminal:

```bash
# 1) Source environments
source /opt/tros/humble/setup.bash
source ~/dual_cam_ws/install/setup.bash

# 2) Start the IMX477 camera node
ros2 launch imx477_dual_camera dual_camera.launch.py

# 3) In another terminal, start the VPS resizer
source /opt/tros/humble/setup.bash
source ~/dual_cam_ws/install/setup.bash
ros2 launch imx477_vps_resizer vps_resizer.launch.py

# 4) In a third terminal, start YOLO + codec + websocket
source /opt/tros/humble/setup.bash
source ~/dual_cam_ws/install/setup.bash
ros2 launch dnn_node_example dnn_imx477_1080.launch.py \
  dnn_example_config_file:=/opt/tros/humble/lib/dnn_node_example/config/<your_yolo_config>.json \
  dnn_example_image_width:=1920 \
  dnn_example_image_height:=1080
```

After everything is running, open the TogetheROS web UI in a browser (for example):

```text
http://<board-ip>:8000/TogetheROS/
```

Select the websocket stream that uses `/image_jpeg_4k` and `/hobot_dnn_detection` to see the annotated video.

---

## Usage Notes

- The camera node runs a 30 FPS timer by default. You can adjust the frame rate by changing the timer period in `imx477_dual_camera_node.cpp`.
- The VPS resizer expects that incoming frames match the configured `input_width`/`input_height` (defaults: 4000×3000). It will log a warning and skip frames if sizes mismatch.
- Both nodes use **RELIABLE** QoS to be compatible with `hobot_codec` and other consumers.
- JPEG quality can be tuned via `jpg_quality` parameters in `dnn_imx477_1080.launch.py`.

---

## Troubleshooting

### 1. No image in the TogetheROS web UI

Checklist:

1. **Verify topics exist**

   ```bash
   ros2 topic list
   ```

   You should see at least:

    - `/imx477/image_raw_4k`
    - `/imx477/image_raw_1080`
    - `/image_jpeg`
    - `/image_jpeg_4k`
    - `/hobot_dnn_detection`

2. **Verify that image topics actually have data**

   ```bash
   ros2 topic echo /imx477/image_raw_1080
   ros2 topic echo /image_jpeg
   ```

   If you see headers and data arrays, the pipeline is publishing correctly.

3. **Check that the websocket is running**

   ```bash
   ros2 node list
   ```

   Look for a node like `/websocket` (depending on the name set in the launch file).

4. **Check websocket parameters**

   The launch file must start the websocket with:

    - `image_topic := /image_jpeg`
    - `image_type := mjpeg`
    - `smart_topic := /hobot_dnn_detection`

   If `image_topic` is misconfigured (e.g. set to `/image_jpeg_4k`), the UI may be very slow or show nothing.

### 2. `hobot_codec` logs `New publisher discovered ... incompatible QoS (RELIABILITY_QOS_POLICY)`

This means the publisher of the raw NV12 topic is using `BEST_EFFORT` while `hobot_codec` expects `RELIABLE`.

Solution:

- Ensure your camera and resizer nodes create publishers with a QoS similar to:

  ```cpp
  auto qos = rclcpp::SensorDataQoS();
  qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  pub_full_ = this->create_publisher<sensor_msgs::msg::Image>(full_topic_, qos);
  ```

- Do the same for subscriptions and publishers in `imx477_vps_resizer`.

After rebuilding (`colcon build`) and restarting the nodes, the QoS warning should disappear and `hobot_codec` will start encoding.

### 3. `Websocket did not receive image data` error

- Confirm that `/image_jpeg` is published and not empty.

- Check that the encoder node `codec_1080_nv12_to_jpeg` is running:

  ```bash
  ros2 node list
  ros2 topic info -v /image_jpeg
  ```

- Ensure the encoder parameters in `dnn_imx477_1080.launch.py` match the actual raw topic:

  ```yaml
  sub_topic: /imx477/image_raw_1080
  in_format: nv12
  out_format: jpeg
  ```

### 4. YOLO (`dnn_node_example`) not detecting anything

- Make sure the DNN node subscribes to the correct topic (`/imx477/image_raw_1080`).

- Ensure the image size in the launch file matches the resizer output:

  ```bash
  dnn_example_image_width:=1920
  dnn_example_image_height:=1080
  ```

- Confirm that your YOLO config file path is valid and the model exists.

### 5. Systemd service fails to start

- Check status and logs:

  ```bash
  sudo systemctl status imx477-pipeline.service
  journalctl -u imx477-pipeline.service -f
  ```

- Common issues:

    - Wrong path in `ExecStart`.
    - Missing `source /opt/tros/humble/setup.bash` or workspace overlay.
    - `dual_cam_ws` not built or install directory missing.

Edit the service file or the helper script, fix the paths, run:

```bash
sudo systemctl daemon-reload
sudo systemctl restart imx477-pipeline.service
```

---

## Extending the Pipeline

Once this base pipeline is working, you can:

- Add additional nodes that subscribe to `/image_jpeg_4k` for recording, cropping, or high‑res analysis.
- Adjust the output resolution in `imx477_vps_resizer` if you need a different inference size.
- Swap the YOLO config/model file to experiment with different detectors.

The pipeline is intentionally modular:

- **Camera & VPS** are pure C++ nodes that expose standard ROS topics.
- **DNN + codecs + websocket** are orchestrated purely by the launch file.

This makes it easy to reuse the IMX477 4K + 1080p setup across different applications on the RDK X5.

### ExecStart script and service wiring

The systemd unit **imx477-pipeline.service** does **not** launch the ROS2 nodes directly. Instead, its `ExecStart` points to a small shell wrapper:

```ini
ExecStart=/opt/vision/imx477_pipeline.sh
```

In the repo, both the unit file and the startup script live under:

- `services/imx477-pipeline.service`
- `services/imx477_pipeline.sh`

The `imx477_pipeline.sh` script is responsible for:

1. Sourcing the ROS2 environment:
    - `source /opt/tros/humble/setup.bash`
2. Sourcing the workspace with the custom nodes:
    - `source ~/dual_cam_ws/install/setup.bash`
3. Starting the 4K camera + 1080p VPS resizer pipeline, for example:
    - `ros2 launch imx477_dual_camera dual_camera.launch.py`
    - `ros2 run imx477_vps_resizer vps_resizer_node`

#### Installing the service and script from the repo

From the root of the cloned repo on a fresh system:

```bash
# 1) Copy the startup script to /opt/vision and make it executable
sudo mkdir -p /opt/vision
sudo cp services/imx477_pipeline.sh /opt/vision/imx477_pipeline.sh
sudo chmod +x /opt/vision/imx477_pipeline.sh

# 2) Install the systemd unit
sudo cp services/imx477-pipeline.service /etc/systemd/system/imx477-pipeline.service

# 3) Reload systemd and enable the pipeline on boot
sudo systemctl daemon-reload
sudo systemctl enable imx477-pipeline.service

# 4) Start (or restart) the pipeline
sudo systemctl start imx477-pipeline.service
# or, after changes:
sudo systemctl restart imx477-pipeline.service
```

After these steps, `systemctl status imx477-pipeline.service` should show that `ExecStart=/opt/vision/imx477_pipeline.sh` is invoked, and under the cgroup you should see the `dual_camera.launch.py` and `vps_resizer_node` processes as part of the pipeline.

