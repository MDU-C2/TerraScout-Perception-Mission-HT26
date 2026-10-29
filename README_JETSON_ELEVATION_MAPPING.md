# TerraScout — Jetson GPU Elevation Mapping

**Status:** Local Jetson proof of concept **working**; PC ↔ Jetson ROS 2/Zenoh integration **not yet implemented**.  
**Last updated:** 28 October 2026  
**Target hardware:** Seeed Studio reComputer J3010 (NVIDIA Jetson Orin Nano, 4 GB)  
**Related TerraScout branch:** `feature/distributed-simulation` in `MDU-C2/TerraScout-Perception-Mission-HT26`

## 1. Purpose

Run [`leggedrobotics/elevation_mapping_cupy`](https://github.com/leggedrobotics/elevation_mapping_cupy) on the Jetson GPU to generate local **elevation**, **variance**, and **traversability** maps for TerraScout. The Jetson is intended to be a separate mapping computer rather than host Gazebo, FAST-LIO2, or Nav2.

We deliberately avoided installing PyTorch on the 4 GB Jetson. Instead, we implemented a **CuPy-only traversability filter** that reuses the original filter's learned weights. This replacement has passed a functional test and runs in the mapping node, but its **numerical equivalence to the PyTorch filter has not yet been verified**.

## 2. Architecture

### 2.1 Implemented and tested: synthetic demonstration on Jetson

```text
Jetson Orin Nano (Ubuntu 22.04 / JetPack 6.2.2)
└── Docker: ROS 2 Jazzy + CUDA/CuPy
    ├── Synthetic PointCloud2 publisher
    │   ├── /camera/depth/points (10 Hz)
    │   └── TF: map → base_link
    │
    └── elevation_mapping_cupy node
        ├── CuPy GPU elevation mapping
        ├── CuPy-only traversability filter (custom)
        └── /elevation_mapping_node/elevation_map
            └── grid_map_msgs/msg/GridMap
                ├── elevation
                ├── variance
                └── traversability
```

This setup runs **entirely within one Docker container**. During these tests the container used `--network=none`; no cross-computer ROS communication has been established yet.

### 2.2 Intended: PC + Jetson integration (next phase)

```text
PC — Ubuntu 24.04 / ROS 2 Jazzy               Jetson — JetPack 6.2.2
┌─────────────────────────────────┐           ┌───────────────────────────────┐
│ TerraScout Gazebo simulation    │           │ ROS 2 Jazzy Docker           │
│    ├── raw LiDAR + IMU          │           │ elevation_mapping_cupy        │
│    └── /clock                   │           │    ├── GPU elevation map     │
│ FAST-LIO2                       │           │    └── CuPy traversability   │
│    └── odometry / TF            │           └───────────────┬───────────────┘
│ Nav2 + missions + Foxglove      │                           │
└────────────────┬────────────────┘                           │
                 │                                            │
                 └──────── Zenoh over Ethernet ───────────────┘
                             LiDAR / TF / clock →
                             ← GridMap
```

**Initial data-flow choice:** send TerraScout's **raw LiDAR** (`/terrascout1/lidar_points_raw`) plus FAST-LIO-derived **TF** and `/clock` to the Jetson. The elevation node will transform raw `lidar_link` scans into `odom`. This preserves the actual sensor origin used for mapping.

FAST-LIO2's `/terrascout1/fast_lio/cloud_registered` is in its internal `camera_init` frame. We **will not** connect that topic directly without addressing its frame conventions and the sensor origin. A motion-compensated body-frame cloud can be evaluated later.

**Existing targets remain untouched:** `sim` (everything on PC), `sim-pc` (Gazebo-side PC), and `sim-nuc` (navigation-side NUC). A dedicated PC + Jetson target was **proposed but not created** at the time of writing.

## 3. Jetson environment

| Component | Verified configuration |
| --- | --- |
| Board | Seeed reComputer J3010 / Jetson Orin Nano, 4 GB |
| Host OS | Ubuntu 22.04, JetPack 6.2.2 / Jetson Linux R36.5.0 |
| CUDA | 12.6; host and Docker GPU kernel tests succeeded |
| Container OS / ROS | Ubuntu 24.04 ARM64, ROS 2 Jazzy (`ros:jazzy-ros-base`) |
| Mapping source | `leggedrobotics/elevation_mapping_cupy`, `ros2` branch |
| CuPy / NumPy | CuPy 13.6.0 / NumPy 1.26.4 |
| SciPy | 1.11.4 |
| Python | 3.12 in the Jazzy container |
| PyTorch | **Not installed or required for our CuPy-selected filter path** |
| Jetson host Ethernet | `eno1`, `10.132.134.79/23` |
| Simulation PC Ethernet | `130.243.66.211` |

The Jetson is aarch64, so we built and tested directly on the device rather than using the PC's x86_64 Docker environment.

## 4. Files and directories created or modified

The Jetson-side working directory is **`~/jetson-elevation/`**. The source checkout is separate from the TerraScout repository.

```text
~/jetson-elevation/
├── Dockerfile                         [ADDED]
├── Dockerfile.mapping                 [ADDED]
├── run-jazzy-gpu.sh                   [ADDED]
└── ros2_ws/                           [ADDED]
    └── src/
        └── elevation_mapping_cupy/    [CLONED: upstream repo, ros2 branch]
            ├── elevation_map_msgs/    [UPSTREAM: ROS messages]
            ├── sensor_processing/
            │   └── semantic_sensor/   [UPSTREAM: dependency]
            └── elevation_mapping_cupy/ [UPSTREAM: ROS package directory]
                ├── config/           [UPSTREAM]
                ├── launch/           [UPSTREAM]
                ├── setup.py          [UPSTREAM]
                └── elevation_mapping_cupy/  [PYTHON MODULE DIRECTORY]
                    ├── elevation_mapping.py         [MODIFIED]
                    ├── traversability_filter.py     [UPSTREAM, preserved]
                    ├── traversability_filter_cupy.py [ADDED]
                    ├── elevation_mapping_node.py    [UPSTREAM]
                    └── ...
```

> **Important nested paths:** There are three nested directories called `elevation_mapping_cupy` (repository → ROS package → Python module). Our new filter belongs in the **innermost Python module**, next to `elevation_mapping.py`, not in the ROS package root. It was initially placed one level too high and then moved to the correct location.

### `Dockerfile` — added

Creates the first ROS 2 Jazzy / CuPy image, tagged **`jetson-jazzy-cupy:test`**. It starts from `ros:jazzy-ros-base`, creates a Python virtual environment at `/opt/venv` with access to system Python packages, and installs the CuPy runtime and compatible NumPy.

**Reason:** ROS 2 Jazzy needs an Ubuntu 24.04 environment, while the Jetson host uses Ubuntu 22.04. Docker lets us retain the Jetson's NVIDIA runtime and host CUDA toolkit while running Jazzy.

### `Dockerfile.mapping` — added

Builds **`jetson-jazzy-elevation:dev`** on top of the CuPy image. It installs the extra ROS/Python dependencies required by `elevation_mapping_cupy`, including build tooling, SciPy, OpenCV, Shapely, `ruamel.yaml`, `cv_bridge`, `grid_map_msgs`, `tf_transformations`, `rosbag2_py`, `simple-parsing`, `pybase64`, and `ros2-numpy`.

Important dependency choices:

- Pinned **`numpy==1.26.4`**: prevents NumPy 2.x from breaking the available SciPy ABI.
- Pinned **`transformations==2024.5.24`**: avoids a later dependency resolution that had pulled in NumPy 2.x.
- Installed **`ros2-numpy==0.0.5 --no-deps`**: its pinned older NumPy dependency was incompatible with this Python 3.12 environment; runtime point-conversion imports were tested successfully.
- **Did not install PyTorch**: limited Jetson RAM motivated the CuPy-only filter experiment.

**Reason:** Package installation and version constraints are reproducible and kept inside the container rather than changing the host Python environment.

### `run-jazzy-gpu.sh` — added

Starts the development container with the appropriate NVIDIA runtime, source mount, and CUDA library paths. The working directory is `/workspace/ros2_ws`.

Notable runtime settings used during the tests:

```text
--runtime=nvidia
--network=none
NVIDIA_VISIBLE_DEVICES=all
NVIDIA_DRIVER_CAPABILITIES=compute,utility
CUDA_PATH=/usr/local/cuda
LD_LIBRARY_PATH=/usr/local/cuda/targets/aarch64-linux/lib:
                /usr/local/cuda/lib64:/usr/local/nvidia/lib:/usr/local/nvidia/lib64
-v /usr/local/cuda:/usr/local/cuda:ro
-v ~/jetson-elevation/ros2_ws:/workspace/ros2_ws
--user <host UID>:<host GID>     (avoids root-owned build files)
```

**Reason:** CuPy initially failed to find `libnvrtc.so.12` inside the Jazzy container. Mounting `/usr/local/cuda` from the Jetson host and setting the CUDA paths solved this. The workspace bind mount keeps edits and builds persistent when the container is deleted. The script must be changed to a network-enabled mode for the future PC–Jetson integration.

### `ros2_ws/src/elevation_mapping_cupy/` — cloned

Cloned the upstream **`ros2` branch**, rather than the default or ROS 1 branch. Built its three ROS 2 packages:

1. `elevation_map_msgs`
2. `semantic_sensor`
3. `elevation_mapping_cupy`

**Reason:** Provides the main CuPy elevation mapping node, supporting message definitions, launch files, and configurations.

### `.../elevation_mapping_cupy/elevation_mapping_cupy/traversability_filter_cupy.py` — added

This file implements **`get_filter_cupy(w1, w2, w3, w_out)`**, a CuPy-only replacement for the original PyTorch-based traversability filter.

- Converts the original learned weight arrays to GPU arrays.
- Evaluates three sets of four 3×3 convolutions/correlations at dilation factors **1, 2, and 3**, using `cupyx.scipy.ndimage.correlate`.
- Crops the results to a common size, combines their absolute responses using the learned 1×1 output weights, and applies `exp(-score)`.
- Preserves the output shape expected by elevation mapping: **`(1, 1, H-6, W-6)`**.

**Reason:** Avoids bringing the heavyweight PyTorch runtime into a 4 GB Jetson deployment while retaining the general structure and trained weights of the upstream traversability model.

The standalone GPU test used a random `32×32` height array and synthetic weights; it returned **`(1, 1, 26, 26)`**, finite values, and passed its assertions. This test established functional operation, **not numerical equality** with PyTorch.

### `.../elevation_mapping_cupy/elevation_mapping_cupy/elevation_mapping.py` — modified

Added an import:

```python
from elevation_mapping_cupy.traversability_filter_cupy import get_filter_cupy
```

Changed the normal (`use_chainer: false`) branch of `ElevationMap.__init__` from:

```python
self.traversability_filter = get_filter_torch(param.w1, param.w2, param.w3, param.w_out)
```

to:

```python
self.traversability_filter = get_filter_cupy(param.w1, param.w2, param.w3, param.w_out)
```

The upstream `traversability_filter.py` remains available and has **not** been replaced. The original Chainer branch was retained. We fixed a tabs/spaces error introduced during editing and verified that Python imports the updated module.

**Reason:** Select the lightweight CuPy filter when running the mapping node, without rewriting the mapping engine.

## 5. Tests completed and observed results

### 5.1 GPU and package bring-up — passed

- A CUDA kernel compiled for **`sm_87`** ran on the host, in NVIDIA's CUDA container, and in an Ubuntu 24.04 / ROS 2 Jazzy container.
- CuPy GPU kernel compilation and arithmetic succeeded after exposing host CUDA/NVRTC libraries.
- The ROS 2 workspace built all three packages successfully.
- Verified imports: `elevation_mapping_cupy.traversability_filter_cupy`, `ElevationMap`, and `ElevationMappingNode`.
- Verified NumPy 1.26.4, SciPy 1.11.4, CuPy 13.6.0, and one visible CUDA device.

### 5.2 Synthetic elevation-mapping demo — passed

Launched:

```bash
ros2 launch elevation_mapping_cupy synthetic_depth_demo.launch.py launch_rviz:=false
```

The synthetic publisher reported point clouds and TF at **10 Hz**. The mapping node reported a map initialized with **length 20.0 m, resolution 0.1 m, 202 internal cells per side**; its published map contains **200×200 = 40,000 cells per layer**.

Verified topic:

```text
/elevation_mapping_node/elevation_map [grid_map_msgs/msg/GridMap]
```

ROS 2 reported a steady mean map publication rate of approximately **10.26 Hz**.

Actual output checked by subscribing to `GridMap` and examining finite cell counts:

| Layer | Finite cells / 40,000 | Minimum | Maximum | Mean |
| --- | ---: | ---: | ---: | ---: |
| `elevation` | **14,697** (36.7%) | 0.0 m | ≈0.400 m | ≈0.00616 m |
| `variance` | **40,000** | ≈0.000132 | 1000.0 | ≈632.58 |
| `traversability` | **14,710** | ≈0.146 | 1.0 | ≈0.965 |

**Interpretation:** The node published actual finite height measurements and nonconstant traversability scores, not merely initialized/empty maps. Many unobserved cells retained large variance; that is not itself a failure.

### 5.3 Memory and thermal sampling

`tegrastats` during the demonstration showed approximately:

- **1,915 / 3,589 MB RAM** in use.
- **600 / 1,795 MB swap** in use.
- **GPU utilization bursts up to ~32%** (`GR3D_FREQ`; activity indicator, not a per-kernel performance measurement).
- **GPU temperature ~50 °C**.

These are encouraging for the 4 GB module, but swapping, processing latency, and sustained performance require further measurement under TerraScout's real LiDAR load.

### 5.4 PC/Jetson network — basic connectivity passed

- Jetson `eno1`: `10.132.134.79/23`.
- PC: `130.243.66.211`.
- Jetson routing to PC: via `10.132.134.1`, out `eno1`.
- Ping: **0% loss**, average RTT **~0.82 ms**.

This does **not yet** confirm a Zenoh connection or ROS topic transfer. The PC's existing `sim` target binds its fleet Zenoh router to loopback, so a separate network-enabled configuration is needed. The contemplated `sim-elevation-pc` target has **not yet been created or rendered**.

### 5.5 TerraScout point cloud and TF — verified on PC

- `/terrascout1/lidar_points_raw` is `sensor_msgs/msg/PointCloud2` with frame **`lidar_link`** and `x/y/z` `FLOAT32` fields (additional `intensity` and `ring` fields are present).
- `/terrascout1/fast_lio/cloud_registered` uses **`camera_init`**.
- `/terrascout1/fast_lio/odometry` uses **`camera_init`**.
- `tf2_echo odom lidar_link` returned valid transforms via TerraScout's namespaced TF topics:
  `/terrascout1/tf` and `/terrascout1/tf_static`.

Our initial plan is `map_frame: odom`, `base_frame: base_link`, sensor data from raw `lidar_link` point clouds, and remapped namespaced TF. Simulation time must use `/clock`.

## 6. Useful commands (current working Jetson setup)

### On the Jetson host

```bash
cd ~/jetson-elevation
./run-jazzy-gpu.sh bash
```

### Inside the Jetson Docker container

```bash
cd /workspace/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# If source files changed, rebuild the Python mapping package:
colcon build --symlink-install \
  --packages-select elevation_mapping_cupy \
  --executor sequential --parallel-workers 1 \
  --cmake-args -DBUILD_TESTING=OFF
source install/setup.bash

# Launch synthetic demonstration (no RViz on the headless Jetson):
ros2 launch elevation_mapping_cupy synthetic_depth_demo.launch.py launch_rviz:=false
```

To inspect topics **while the demo is running**, open a *second shell in the same container* (not a separate `--network=none` container):

```bash
# On Jetson host:
sudo docker ps
sudo docker exec -it <CONTAINER_ID> bash

# In that container:
source /opt/ros/jazzy/setup.bash
source /workspace/ros2_ws/install/setup.bash
ros2 topic list -t
ros2 topic hz /elevation_mapping_node/elevation_map
ros2 topic echo --once /elevation_mapping_node/elevation_map --field layers
```

Monitor hardware on the **Jetson host**:

```bash
tegrastats --interval 1000
```

## 7. Remaining work / next steps

1. **Validate filter equivalence:** Compare the CuPy filter numerically with the original PyTorch convolutions using identical *real learned weights* and test elevation arrays, preferably on a more memory-capable computer; assess boundary behavior and error tolerance.
2. **Add a dedicated PC test target:** The proposed `sim-elevation-pc` should keep Gazebo, FAST-LIO2, Nav2, and Foxglove on the PC while exposing the fleet Zenoh router on an appropriate network interface. Leave `sim`, `sim-pc`, and `sim-nuc` unchanged.
3. **Configure topic forwarding and access control:** Only the required point cloud, dynamic/static TF, simulation clock, and returning GridMap should cross Zenoh. Verify routed TCP connectivity and constrain access to trusted hosts.
4. **Enable networking on the Jetson container:** Replace the isolated `--network=none` development setup with an appropriate Zenoh/ROS 2 networking arrangement and matching ROS domain (TerraScout fleet uses **ROS_DOMAIN_ID=33** for its configured instance).
5. **Configure elevation mapping for TerraScout:** Use `/terrascout1/lidar_points_raw` (`lidar_link`), `map_frame: odom`, `base_frame: base_link`, ROS time `/clock`, and the namespaced `/terrascout1/tf` and `/terrascout1/tf_static` topics. Test TF availability at actual cloud timestamps.
6. **Verify real data and latency:** Confirm valid elevation/traversability cells under Gazebo motion, map centering, 5–10 Hz publication, CPU/GPU/RAM/swap use, and point-cloud/Zenoh network bandwidth. Reduce rates or cloud density if needed.
7. **Visualize on PC:** Forward `/elevation_mapping_node/elevation_map` back to the PC and display it in Foxglove (or another GridMap-capable viewer).
8. **Later:** Integrate terrain-derived costs with Nav2 and extend the PC + NUC + Jetson distributed topology without disturbing existing navigation behavior.

## 8. Notes and cautions

- **Successful message publication is not yet a navigation safety validation.** The replacement traversability filter's learned weights, numerical output, and map conventions require testing before its values influence costs or keepout decisions.
- The `map` frame used in the synthetic demo is not the same as TerraScout's proposed `odom` integration frame; do not use the demo topic configuration unchanged for TerraScout.
- Docker images (`jetson-jazzy-cupy:test` and `jetson-jazzy-elevation:dev`) are locally built images; source edits live in the bind-mounted `~/jetson-elevation/ros2_ws` folder.
- Keep the new `traversability_filter_cupy.py` tracked in version control if rebasing or recloning the upstream mapping repository; otherwise it can be lost. The original `traversability_filter.py` was not changed.

---

**Summary:** As of 28 October 2026, the Jetson can successfully run the ROS 2 Jazzy CuPy elevation-mapping node on its GPU without PyTorch and publish a populated **20 × 20 m GridMap at ~10 Hz**. **Zenoh integration with the TerraScout PC is the next milestone, not yet a completed feature.**
