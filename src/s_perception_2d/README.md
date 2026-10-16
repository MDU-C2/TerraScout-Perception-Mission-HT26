# `s_perception_2d` - 2D Perception & Object Detection Package

ROS 2 Jazzy package providing real-time 2D object detection, confidence filtering, class validation, and structured logging using YOLOv8n on the SAGE TerraScout autonomous mobile robot.

---

## 📋 Table of Contents
1. [Architecture & Pipeline](#architecture--pipeline)
2. [Supported Object Classes](#supported-object-classes)
3. [Topics & Interface](#topics--interface)
4. [Downstream 3D Localization Integration](#downstream-3d-localization-integration)
5. [Prerequisites & Build Instructions](#prerequisites--build-instructions)
6. [Step-by-Step Execution Guides](#step-by-step-execution-guides)
   - [Workflow A: Live 3D Simulation in Gazebo](#workflow-a-live-3d-simulation-in-gazebo)
   - [Workflow B: 2D Synthetic / Offline Testing (`image_streamer.py`)](#workflow-b-2d-synthetic--offline-testing-image_streamerpy)
   - [Workflow C: Live Physical OAK-D Camera Testing](#workflow-c-live-physical-oak-d-camera-testing)
7. [Where to Find and Inspect Logs](#where-to-find-and-inspect-logs)
8. [Adding Custom 3D Meshes & Objects to Gazebo](#adding-custom-3d-meshes--objects-to-gazebo)
9. [Files Modified & Required for Git](#files-modified--required-for-git)

---

## 🏗 Architecture & Pipeline

```text
[Gazebo / Real OAK-D Camera]             [2D Synthetic Streamer]
          │                                         │
          │ (/terrascout1/oakd/rgb/image_raw)       │ (/terrascout1/oakd/rgb/image_raw)
          └──────────────────┬──────────────────────┘
                             ▼
               [detector_node.py (YOLOv8n)]
           - Inference throttled to target FPS (5 Hz)
           - Confidence threshold filtering (default: 0.35)
                             ▼
           [object_record.py - Validation Engine]
           - Validates bounding box coordinates
           - Maps to valid classes or flags as 'unknown'
                             ├──► Published: /terrascout1/oakd/detections_2d (vision_msgs/Detection2DArray)
                             │    (For Downstream 3D Bounding Box & Localization Nodes)
                             │
                             ├──► Published: /terrascout1/oakd/detections_image (sensor_msgs/Image)
                             │    (On-demand visual boxes for Foxglove Studio / RQt)
                             │
                             └──► Logged: Structured JSON Lines (.jsonl)
                                  (Timestamped bounding boxes, confidences, classes)
```

---

## 🏷 Supported Object Classes

The trained model ([`models/best.pt`](models/best.pt)) identifies the following 4 classes:
* `stop-sign` (traffic stop sign)
* `pedestrian_sign` (pedestrian crossing sign)
* `roadwork_sign` (construction / road work sign)
* `traffic-cone` (traffic hazard cone)

Any detection that cannot be verified or matched against configured classes is safely preserved with class label `unknown`.

---

## 📡 Topics & Interface

### Subscribed
* `/terrascout1/oakd/rgb/image_raw` (`sensor_msgs/msg/Image`)
  * Color camera stream (1280x720 RGB8/BGR8) from the simulated or physical OAK-D sensor. Uses Best-Effort QoS.

### Published
* `/terrascout1/oakd/detections_2d` (`vision_msgs/msg/Detection2DArray`)
  * **Primary programmatic output for downstream autonomy:**
    * `bbox.center.position.x`: Bounding box center horizontal pixel ($u$)
    * `bbox.center.position.y`: Bounding box center vertical pixel ($v$)
    * `bbox.size_x`: Bounding box width (pixels)
    * `bbox.size_y`: Bounding box height (pixels)
    * `results[0].hypothesis.class_id`: String label (e.g., `stop-sign`)
    * `results[0].hypothesis.score`: Detection confidence score (e.g., `0.85`)
* `/terrascout1/oakd/detections_image` (`sensor_msgs/msg/Image`, conditional)
  * Rendered image with bounding boxes, labels, and center crosshairs.
  * **Optimized:** Only renders and publishes when at least one subscriber (e.g. Foxglove Studio) is listening, saving CPU cycles.

---

## 🎯 Downstream 3D Localization Integration

For teammates developing 3D object localization:
1. Subscribe to `/terrascout1/oakd/detections_2d` to receive the 2D bounding boxes and class names.
2. Subscribe to synchronized depth data:
   * Depth image: `/terrascout1/oakd/depth/image_raw` (`sensor_msgs/msg/Image`, 32FC1 / 16UC1)
   * Or Point Cloud: `/terrascout1/oakd/points` (`sensor_msgs/msg/PointCloud2`)
3. Extract depth values within the $[x_1, y_1, x_2, y_2]$ region of interest (ROI).
4. Apply median depth filtering (e.g., MAD filter) to remove background outliers.
5. Project $(u, v, \text{depth})$ into 3D camera coordinates $(X_c, Y_c, Z_c)$ using camera intrinsic matrix from `/terrascout1/oakd/rgb/camera_info`.
6. Transform coordinates from frame `oakd_rgb_optical_frame` into `map` or `odom` via TF2 to obtain 3D obstacle poses and 3D bounding boxes.

---

## ⚙️ Prerequisites & Build Instructions

### Dependencies
Dependencies are pre-configured in `docker/Dockerfile`:
* Python packages: `ultralytics>=8.0.0`, `torch>=2.0.0`, `torchvision`, `numpy<2.0.0`, `opencv-python`
* ROS 2 packages: `vision-msgs`, `cv-bridge`, `teleop-twist-keyboard`

### Building the Package
Inside the Docker environment:
```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select s_perception_2d
source install/setup.bash
```

### Running Unit Tests
```bash
colcon test --packages-select s_perception_2d
colcon test-result --verbose
```

---

## 🚀 Step-by-Step Execution Guides

### First-Time Clone Setup (Required Once for Teammates)

When cloning the repository for the first time, the `build/` and `install/` folders do not exist yet (as they are git-ignored). **You must build the SAGE workspace once before running `make up`:**

```bash
# From the repository root:
make build
```
> **What `make build` does:**
> 1. Imports third-party submodule dependencies (`third_party.repos`).
> 2. Builds the Docker base, simulation, and dev images with PyTorch and YOLO dependencies.
> 3. Compiles all ROS 2 packages via `colcon` to generate `install/setup.bash` and `build/`.

---

### Workflow A: Live 3D Simulation in Gazebo (Recommended)

This workflow runs the full simulation with TerraScout, Gazebo 3D forest world, OAK-D camera, and the YOLO detector node automatically running in the robot lifecycle.

#### Terminal 1: Launch Gazebo & SAGE Fleet
```bash
# 1. Navigate to the repository root
cd ~/TerraScout-Perception-Mission-HT26

# 2. Select the TerraScout standalone simulation target
make use TARGET=sim-scout

# 3. Launch Gazebo and robot containers
make up
```
*Wait until Gazebo opens and the terminal prints robot status.*

> **Troubleshooting Note:**  
> If you get `start failed: operation: start Compose project sage1; cause: the project already has running containers`, simply run `make down` to stop the previous session before running `make up`.

#### Terminal 2: Teleoperate the Robot (Drive Around)
Open a new terminal:
```bash
docker compose exec unit-terrascout1-1 bash -c "source /sage/install/setup.bash && ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/terrascout1/cmd_vel_teleop"
```
*Use keys `i`, `j`, `k`, `l`, `,` to drive TerraScout toward the stop signs.*

#### Foxglove Studio Visualization
1. Open Foxglove Studio (desktop app or browser at `https://studio.foxglove.dev/`).
2. Connect to Foxglove Bridge: `ws://localhost:8765`.
3. Open an **Image Panel** and select the topic:
   * `/terrascout1/oakd/detections_image` (to see the camera feed with live detection boxes).
4. Open a **3D Panel** to observe the robot in the map.

*(Note: If you see a warning icon beside "Gradient" in Foxglove's image panel settings, it is harmless. It simply indicates that gradient color-mapping is for single-channel grayscale/depth images, whereas this is a full-color RGB image.)*

---

### Workflow B: 2D Synthetic / Offline Testing (`image_streamer.py`)

This workflow allows testing the detector without launching Gazebo, using standalone test images.

#### Terminal 1: Run the YOLO Detector Node
```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch s_perception_2d yolo_detector.launch.py input_image_topic:=/terrascout1/oakd/rgb/image_raw
```

#### Terminal 2: Stream Test Images
```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 src/s_perception_2d/s_perception_2d/image_streamer.py \
  --image dataset/test/images/sample1.jpg \
  --topic /terrascout1/oakd/rgb/image_raw \
  --rate 5.0
```

#### Terminal 3: Verify Output Topic
```bash
source /opt/ros/jazzy/setup.bash
# Echo detections:
ros2 topic echo /terrascout1/oakd/detections_2d

# Check publishing rate:
ros2 topic hz /terrascout1/oakd/detections_2d
```

---

### Workflow C: Live Physical OAK-D Camera Testing

This workflow connects a physical Luxonis OAK-D camera to your laptop and runs the YOLO detector with live Foxglove / RQt visualization.

#### Step 1: Create Lightweight Camera Config
Run this once to create a clean, power-optimized camera config that avoids USB bandwidth drops:
```bash
cat << 'EOF' > /tmp/oak_rgb.yaml
/**:
  ros__parameters:
    camera:
      i_pipeline_type: RGB
      i_nn_type: none
      i_enable_imu: false
      i_enable_ir: false
    rgb:
      i_resolution: "1080P"
      i_fps: 15.0
EOF
```

#### Terminal 1: Launch Physical Camera Driver
```bash
source /opt/ros/jazzy/setup.bash
ros2 launch depthai_ros_driver camera.launch.py camera_model:=OAK-D-PRO params_file:=/tmp/oak_rgb.yaml
```
*Wait until it prints: `Finished setting up pipeline` and starts publishing to `/oak/rgb/image_raw`.*

#### Terminal 2: Launch Foxglove Bridge (for Visualization)
```bash
source /opt/ros/jazzy/setup.bash
ros2 run foxglove_bridge foxglove_bridge --ros-args -p port:=8765
```
*Wait until it prints: `Foxglove bridge listening on port 8765`.*

#### Terminal 3: Run the YOLO Detector
```bash
cd ~/TerraScout-Perception-Mission-HT26  # or ~/sage-0.1.0-extracted
source /opt/ros/jazzy/setup.bash
python3 src/s_perception_2d/s_perception_2d/detector_node.py --ros-args -p input_image_topic:=/oak/rgb/image_raw
```

#### Foxglove Studio / RQt Visualization
1. In Foxglove Studio, connect to `ws://localhost:8765`.
2. Add an **Image Panel** and select the topic:
   * **`/oakd/detections_image`**
3. *(Alternative using ROS 2 native viewer)*:
   ```bash
   source /opt/ros/jazzy/setup.bash
   ros2 run rqt_image_view rqt_image_view /oakd/detections_image
   ```

---

## 📝 Where to Find and Inspect Logs

During execution, detections are logged into structured JSON Lines (`.jsonl`) files.

### Log File Locations
* **Active/Recent Runs:**
  ```text
  runs/<TIMESTAMP>-sim-scout.1/log/perception/detections_<TIMESTAMP>.jsonl
  ```
* **Symlink to Most Recent Run:**
  ```text
  .sage/run/1/log/perception/detections_*.jsonl
  ```

### Useful Commands to Inspect Logs
```bash
# Find all perception log files:
find runs -name "*.jsonl"

# View only the lines where objects were detected (filtering out empty frames):
grep -v '"num_detections": 0' $(find runs -name "*.jsonl" | tail -n 1) | head -n 10

# Count total detections in the latest run:
grep -v '"num_detections": 0' $(find runs -name "*.jsonl" | tail -n 1) | wc -l
```
---


