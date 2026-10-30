# TerraScout — Jetson GPU Elevation Mapping

**Status:** **Working PC ↔ Jetson distributed simulation integration.** The Jetson receives filtered LiDAR and TF via ROS 2/Zenoh, computes elevation/variance/traversability on the GPU, and returns a `GridMap` that is visible in Foxglove on the PC. A 12-minute-51-second memory test with CuPy memory-pool trimming enabled showed stable system RAM. This is a working research/development configuration, **not yet a validated safety-critical navigation costmap**.

**Last updated:** 29 October 2026  
**Jetson:** Seeed Studio reComputer J3010, Jetson Orin Nano 4 GB  
**PC repository:** `MDU-C2/TerraScout-Perception-Mission-HT26`  
**Working branch:** `feature/distributed-simulation`  
**PC target:** `sim-elevation-pc`

> **Scope:** This README documents the tested simulation-to-Jetson mapping pipeline, local development files, launch procedures, and observations as of the date above. The Jetson mapping checkout is **separate** from the TerraScout repository. Changes to local workspaces are not automatically committed to either repository.

## 1. Purpose and current design

Run [`leggedrobotics/elevation_mapping_cupy`](https://github.com/leggedrobotics/elevation_mapping_cupy), `ros2` branch, on the Jetson GPU to generate a local terrain `grid_map_msgs/msg/GridMap` containing:

- `elevation` — estimated ground height;
- `variance` — uncertainty in height estimates;
- `traversability` — terrain score from a custom CuPy-only filter.

**Task distribution:** Gazebo, FAST-LIO2, LiDAR filtering, Nav2, missions, and Foxglove remain on the Ubuntu PC. The Jetson runs only the elevation-mapping ROS 2 stack inside a CUDA-enabled Docker container. A Zenoh TCP connection carries sensor/transform data to the Jetson and the resulting GridMap back to the PC.

PyTorch is deliberately omitted from the Jetson runtime to reduce memory requirements. The custom CuPy traversability filter runs successfully, but **numerical equivalence with the original PyTorch filter remains unverified**.

## 2. Implemented architecture

```text
Ubuntu 24.04 PC                                      Jetson Orin Nano 4 GB
TerraScout target: sim-elevation-pc                    Ubuntu 22.04 / JetPack 6.2.2

Gazebo ── /terrascout1/lidar_points_raw ──┬─> FAST-LIO2 (raw cloud)
                                         │
                                         └─> lidar_filter.py
                                                │
                                     /terrascout1/lidar_points_filtered
                                                │
FAST-LIO2 / robot TF ── /terrascout1/tf[ _static ] ─┤
Simulation clock ─────────────────────── /clock ─────┤
                                                     │
PC fleet Zenoh TCP router, 130.243.66.211:7447       │
                   │                                 │
                   └──────── Zenoh over Ethernet ────┤
                                                     ▼
                                           Docker: ROS 2 Jazzy
                                           RMW: rmw_zenoh_cpp
                                           CUDA 12.6 + CuPy
                                           elevation_mapping_cupy
                                                     │
                                           20 m × 20 m / 0.1 m
                                           elevation / variance /
                                           traversability
                                                     │
                    /elevation_mapping_node/elevation_map
                           grid_map_msgs/msg/GridMap
                                                     │
                   ◄──────── Zenoh over Ethernet ────┘
                   │
             PC ROS 2 graph
                   │
             Foxglove bridge :8765
                   │
             Foxglove visualization
```

**Actual mapping input:** `/terrascout1/lidar_points_filtered` (`sensor_msgs/msg/PointCloud2`, `frame_id: lidar_link`). The filter removes invalid and near-body returns, including points below approximately 0.5 m from the sensor. FAST-LIO2 intentionally continues to consume the raw LiDAR stream.

**Frames and time:** Map frame `odom`; robot base frame `base_link`; incoming cloud frame `lidar_link`. The Jetson node uses `use_sim_time:=true`, consumes `/clock`, and remaps `/tf` and `/tf_static` to the robot's namespaced transform topics.

**Not used as mapping input:** `/terrascout1/fast_lio/cloud_registered`. It is expressed in FAST-LIO2's `camera_init` frame and would require additional frame/origin handling before substitution.

**Deployment boundary:** The `sim-elevation-pc` target adds the needed mapping streams while preserving the existing `sim`, `sim-pc`, and `sim-nuc` targets. Other standard robot exports may also be configured by the fleet boundary; the diagram shows the streams specifically used for elevation mapping.

## 3. Verified environment

| Component | Configuration |
| --- | --- |
| Jetson board | Seeed reComputer J3010 / Orin Nano 4 GB (ARM64) |
| Jetson host | Ubuntu 22.04, JetPack 6.2.2, Jetson Linux R36.5.0 |
| Jetson display mode | Headless systemd `multi-user.target` |
| CUDA | 12.6, NVIDIA Docker runtime; GPU kernels tested |
| Jetson Docker image | `jetson-jazzy-elevation:dev` |
| Container userspace | Ubuntu 24.04 ARM64 / ROS 2 Jazzy |
| Middleware | `rmw_zenoh_cpp`, `ROS_DOMAIN_ID=33` |
| Mapping checkout | `leggedrobotics/elevation_mapping_cupy`, `ros2` branch |
| CuPy / NumPy / SciPy | 13.6.0 / 1.26.4 / 1.11.4 |
| Python | 3.12 in Jazzy container |
| PyTorch | Not installed or required for the selected CuPy filter |
| Jetson network | `eno1`, `10.132.134.79/23` at test time |
| PC network | `130.243.66.211` at test time |
| PC router endpoint | `tcp/130.243.66.211:7447` |
| PC Foxglove WebSocket | `ws://localhost:8765` (when viewing on PC) |

The listed addresses reflect the tested network; change them when the network or PC address changes.

### 3.1 Why Docker is used

ROS 2 Jazzy runs on Ubuntu 24.04 inside the container while the Jetson host remains on its JetPack-compatible Ubuntu 22.04 installation. The launcher exposes NVIDIA compute libraries and bind-mounts the ROS 2 workspace, so source edits persist when temporary containers exit.

### 3.2 Headless operation

The Jetson is operated over SSH:

```bash
ssh terrascout1@10.132.134.79
```

The default boot target is currently `multi-user.target` (no desktop), which frees RAM. A directly attached monitor may remain black in this configuration; SSH access is the tested management method.

To **restore the graphical desktop** if needed, run on the Jetson host:

```bash
sudo systemctl set-default graphical.target
sudo reboot
```

To switch back to **headless** mode:

```bash
sudo systemctl set-default multi-user.target
sudo reboot
```

## 4. Files and source changes

### 4.1 Jetson-side workspace (outside the TerraScout repository)



Dockerfile
   └── Ubuntu 24.04 + ROS 2 Jazzy + CuPy
          │
          ▼
Dockerfile.mapping
   └── Adds elevation_mapping_cupy + dependencies
          │
          ▼
     ONE Docker container



```text
~/jetson-elevation/
├── Dockerfile                         # Base image: jetson-jazzy-cupy:test
├── Dockerfile.mapping                 # Mapping image: jetson-jazzy-elevation:dev
├── run-jazzy-gpu.sh                   # Earlier isolated/synthetic test launcher
├── run-jazzy-zenoh.sh                 # GPU-enabled, --network=host launcher
├── start-terrascout-mapping.sh        # One-command startup helper (if created)
└── ros2_ws/
    └── src/
        └── elevation_mapping_cupy/    # Upstream checkout, ros2 branch
            ├── elevation_map_msgs/
            ├── sensor_processing/
            │   └── semantic_sensor/
            └── elevation_mapping_cupy/  # ROS package
                ├── config/
                │   └── setups/terrascout/terrascout.yaml
                ├── launch/
                ├── setup.py
                └── elevation_mapping_cupy/  # Python module
                    ├── elevation_mapping.py              # MODIFIED
                    ├── elevation_mapping_node.py
                    ├── traversability_filter.py         # Upstream retained
                    └── traversability_filter_cupy.py    # ADDED
```

> There are **three nested** `elevation_mapping_cupy` directories: upstream repository → ROS package → Python module. `traversability_filter_cupy.py` belongs in the innermost directory, next to `elevation_mapping.py`. Do not lose this locally added file when updating/recloning upstream.

#### Docker images and runtime

- `Dockerfile` builds `jetson-jazzy-cupy:test` from `ros:jazzy-ros-base`, using a virtual environment with system packages available.
- `Dockerfile.mapping` builds `jetson-jazzy-elevation:dev`, adding the ROS and Python mapping dependencies (including `grid_map_msgs`, OpenCV, SciPy, Shapely, `ruamel.yaml`, `tf_transformations`, `rosbag2_py`, `simple-parsing`, `pybase64`, and `ros2-numpy`).
- Dependency compatibility choices: `numpy==1.26.4`, `transformations==2024.5.24`, and `ros2-numpy==0.0.5 --no-deps`. These were chosen to avoid incompatible NumPy dependency resolution.
- `run-jazzy-gpu.sh` uses `--network=none` for the earlier self-contained synthetic demo; **it is not the launcher used for the distributed setup**.
- `run-jazzy-zenoh.sh` uses `--network=host`, `--runtime=nvidia`, the host CUDA library paths, and a bind mount from `~/jetson-elevation/ros2_ws` to `/workspace/ros2_ws`. It uses `docker run --rm -it`; exiting removes the container, **not** the bind-mounted source or configs.
- Mounting `/usr/local/cuda` and providing appropriate CUDA paths resolved the earlier `libnvrtc.so.12` lookup error.

#### CuPy-only traversability filter

The custom `traversability_filter_cupy.py` provides `get_filter_cupy(w1, w2, w3, w_out)`. It converts the learned weight arrays to CuPy arrays, applies 3 × 3 correlations at dilations 1, 2, and 3 using `cupyx.scipy.ndimage.correlate`, combines weighted absolute responses, and evaluates `exp(-score)`. The expected output shape is `(1, 1, H-6, W-6)`.

The original `elevation_mapping.py` was modified to import `get_filter_cupy` and select it in the normal (`use_chainer: false`) path instead of `get_filter_torch`. The upstream `traversability_filter.py` and Chainer branch were preserved. A synthetic 32 × 32 test passed the expected 26 × 26 output-shape and finite-value checks. **This is functional validation, not numerical equivalence testing.**

### 4.2 PC-side TerraScout repository changes

These are development-tree changes associated with `feature/distributed-simulation`; confirm they are committed before relying on a fresh clone.

| File | Purpose |
| --- | --- |
| `src/fleet_config/fleet_config/targets.py` | Defines `sim-elevation-pc` with deployment `elevation-pc` and PC Zenoh uplink address/interface. |
| `src/fleet_config/fleet_config/unit.py` | Defines additional ROS/Zenoh export and import boundary entries for mapping (LiDAR, TF, `/clock`, returned `GridMap`). |
| `src/terrascout_navigation/terrascout_navigation/lidar_filter.py` | Produces filtered LiDAR for the Jetson while FAST-LIO2 stays on raw LiDAR. |
| `docker/Dockerfile` | Adds `ros-${ROS_DISTRO}-grid-map-msgs` to the relevant PC image build so the message type is installed. |
| `src/terrascout_mission/config/simulation/foxglove_bridge.yaml` | Whitelists the returned elevation map topic for the existing Foxglove bridge. |

The PC's tested target declaration uses:

```python
"sim-elevation-pc": replace(
    SIM,
    deployment="elevation-pc",
    topology=Topology(uplink_address="130.243.66.211", uplink_interface="eno1"),
),
```

The `elevation-pc` communication boundary includes the mapping-specific topics:

| Direction relative to PC | Topic | Type |
| --- | --- | --- |
| PC → Jetson | `/clock` | `rosgraph_msgs/msg/Clock` |
| PC → Jetson | `/terrascout1/lidar_points_filtered` | `sensor_msgs/msg/PointCloud2` |
| PC → Jetson (also exported) | `/terrascout1/lidar_points_raw` | `sensor_msgs/msg/PointCloud2` |
| PC → Jetson | `/terrascout1/tf` | `tf2_msgs/msg/TFMessage` |
| PC → Jetson | `/terrascout1/tf_static` | `tf2_msgs/msg/TFMessage` |
| Jetson → PC | `/elevation_mapping_node/elevation_map` | `grid_map_msgs/msg/GridMap` |

The `GridMap` is a **ROS topic**; the fact that it is present in `ros2 topic list` does not automatically make it available in Foxglove. The Foxglove bridge has its **own** topic whitelist.

## 5. Persistent TerraScout mapping configuration

The Jetson's active setup file is:

```text
~/jetson-elevation/ros2_ws/src/elevation_mapping_cupy/elevation_mapping_cupy/config/setups/terrascout/terrascout.yaml
```

Relevant tested settings (shown as a focused excerpt; retain the rest of the existing file):

```yaml
/elevation_mapping_node:
  ros__parameters:
    map_frame: "odom"
    base_frame: "base_link"
    corrected_map_frame: "odom"
    subscribers:
      lidar:
        topic_name: "/terrascout1/lidar_points_filtered"
        data_type: "pointcloud"
    publishers:
      elevation_map:
        layers: ["elevation", "variance", "traversability"]
        basic_layers: ["elevation"]
        fps: 5.0
    cupy_memory_pool_trim_interval_s: 10.0
```

**Recommended operational value: `cupy_memory_pool_trim_interval_s: 10.0`.** Disabling trimming (`0.0`) caused sustained RAM growth in the recorded comparison; see Section 8.

The 20 m × 20 m map with 0.1 m resolution publishes three layers. This corresponds to 200 × 200 = **40,000 cells per layer** in the `GridMap` message. A configured publisher target of **5 Hz is not a measured sustained rate**; it must be verified with `ros2 topic hz`.

## 6. Start and stop the distributed system

Start **PC first, then Jetson**. The PC provides `/clock`, LiDAR, and TF. Restarting Gazebo while the Jetson mapper is still processing old timestamps can cause `TF_OLD_DATA` warnings; if the PC simulation is restarted, restart the Jetson mapping node afterward.

### 6.1 On the PC host

From the TerraScout repository checkout:

```bash
make render TARGET=sim-elevation-pc
make up TARGET=sim-elevation-pc
```

In a separate PC terminal, enter the running ROS 2 container:

```bash
make shell TARGET=sim-elevation-pc
```

`make up` is the simulation process; keep it running while testing the Jetson. The dedicated target is intended not to modify the existing `sim`, `sim-pc`, or `sim-nuc` configurations.

### 6.2 On the Jetson host — preferred one-command launcher

With the PC running:

```bash
ssh terrascout1@10.132.134.79
cd ~/jetson-elevation
./start-terrascout-mapping.sh
```

The `start-terrascout-mapping.sh` helper, where installed, calls `run-jazzy-zenoh.sh` and performs the ROS sourcing, Zenoh environment configuration, and mapping-node launch shown below. It is an interactive launcher; **automatic startup at boot is not yet configured**.

### 6.3 Jetson manual fallback (works with the existing Docker launcher)

If the one-command helper has not been created, or while debugging, run on the Jetson host:

```bash
cd ~/jetson-elevation
./run-jazzy-zenoh.sh bash
```

Then **inside that container**:

```bash
source /opt/ros/jazzy/setup.bash
source /workspace/ros2_ws/install/setup.bash

export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ROS_DOMAIN_ID=33
export ZENOH_CONFIG_OVERRIDE='mode="client";connect/endpoints=["tcp/130.243.66.211:7447"];scouting/multicast/enabled=false'

CORE=$(ros2 pkg prefix --share elevation_mapping_cupy)/config/core/core_param.yaml
SETUP=/workspace/ros2_ws/src/elevation_mapping_cupy/elevation_mapping_cupy/config/setups/terrascout/terrascout.yaml

ros2 run elevation_mapping_cupy elevation_mapping_node.py \
  --ros-args --params-file "$CORE" --params-file "$SETUP" \
  -p use_sim_time:=true \
  -r /tf:=/terrascout1/tf \
  -r /tf_static:=/terrascout1/tf_static
```

The `ZENOH_CONFIG_OVERRIDE` endpoint is the **PC**, not the Jetson's own address. Ensure TCP port `7447` is reachable and the PC fleet router is running.

### 6.4 Stop/restart

- Stop the Jetson mapper with **Ctrl+C** in its launching terminal. The current `docker run --rm -it` setup removes its temporary container on exit.
- Change `terrascout.yaml` on the Jetson **host** and launch a new mapping container to apply the new parameters. A Docker image rebuild is **not** required for YAML-only changes because the workspace is bind-mounted.
- Stop the PC simulation with Ctrl+C in the `make up` terminal or use the project's `make down TARGET=sim-elevation-pc` workflow when appropriate.

To inspect active Jetson containers and host processes:

```bash
sudo docker ps
ps -eo pid,comm,rss --sort=-rss | head -n 15
free -h
```

## 7. Validate ROS 2, Zenoh, and Foxglove

### 7.1 Confirm PC receives the GridMap

Inside the **PC** ROS 2 container (`make shell TARGET=sim-elevation-pc`):

```bash
ros2 pkg prefix grid_map_msgs
ros2 topic list -t --no-daemon | grep elevation

timeout 20 ros2 topic echo --once \
  /elevation_mapping_node/elevation_map --field layers
```

**Observed successful results:**

```text
/opt/ros/jazzy
/elevation_mapping_node/elevation_map [grid_map_msgs/msg/GridMap]
['elevation', 'variance', 'traversability']
```

These checks were performed successfully with the Jetson mapping node running. They verify PC-side topic discovery and receipt of an actual GridMap message over Zenoh.

### 7.2 Confirm Foxglove is allowed to advertise it

The **existing** Foxglove bridge is `/terrascout1/foxglove_bridge`; the expected WebSocket port is `8765`.

In `src/terrascout_mission/config/simulation/foxglove_bridge.yaml`, add the following **alongside the existing whitelist entries**, preserving the file's indentation and other topics:

```yaml
- ^/elevation_mapping_node/elevation_map$
```

Inspect the **running bridge's** parameters from the PC ROS 2 container:

```bash
ros2 node list | grep foxglove
ros2 param get /terrascout1/foxglove_bridge topic_whitelist | grep elevation_map
```

**Observed issue and resolution (29 October):** The PC received GridMap data, but the live bridge whitelist initially did not contain the elevation topic, so the topic was absent from Foxglove. After correcting the whitelist and reloading/restarting the relevant PC configuration, elevation mapping became visible in Foxglove.

If source changes do not appear in the running bridge, re-render/restart the relevant PC target. Rebuilding the Docker image is needed when a missing **package** must be installed, but **not** merely to change a runtime YAML whitelist.

Connect the Foxglove application to the standard bridge on `ws://localhost:8765` when running/viewing on the PC. The earlier temporary preview port `8766` is not needed for the regular setup. In the 3D view, elevation can define the terrain surface and traversability can be used as its coloring layer; this is a visualization, not an automatic Nav2 costmap integration.

### 7.3 Check measured publication rate

Inside the PC ROS 2 container:

```bash
ros2 topic hz /elevation_mapping_node/elevation_map
```

The configured publisher target is `5.0 Hz`; **sustained PC-side reception frequency and latency have not yet been recorded for this distributed configuration**.

## 8. Tests and measured results

### 8.1 Earlier synthetic Jetson proof of concept — passed

The isolated synthetic demo used:

```bash
ros2 launch elevation_mapping_cupy synthetic_depth_demo.launch.py launch_rviz:=false
```

It published synthetic point clouds and transforms at 10 Hz, with the Jetson map node producing a 20 m × 20 m map at 0.1 m resolution. The original synthetic test measured approximately **10.26 Hz** GridMap publication and the following sample counts per 40,000-cell layer:

| Synthetic layer | Finite cells | Minimum | Maximum | Mean |
| --- | ---: | ---: | ---: | ---: |
| `elevation` | 14,697 (36.7%) | 0.0 m | ≈0.400 m | ≈0.00616 m |
| `variance` | 40,000 | ≈0.000132 | 1000.0 | ≈632.58 |
| `traversability` | 14,710 | ≈0.146 | 1.0 | ≈0.965 |

The synthetic test previously sampled roughly 1,915/3,589 MB RAM and 600/1,795 MB swap. **Those earlier memory figures are not representative of the present headless distributed configuration.**

### 8.2 Distributed mapping — functional checks passed

The Jetson received filtered LiDAR (`lidar_link`) and transforms from the PC, and the PC received the returned GridMap via Zenoh. A live sample of the 200 × 200 published map contained:

| Live layer | Finite values / 40,000 | Observed values / notes |
| --- | ---: | --- |
| `elevation` | 10,390 (26.0%) | Approx. −3.03 m to +1.77 m |
| `variance` | 40,000 (100%) | Unobserved cells may retain a default value of 1000; finite does not imply well observed |
| `traversability` | 32,532 (81.3%) | Approx. 0 to 0.943; finite scores alone do not establish a valid elevation measurement |

These measurements show populated layers and working transport, **not yet accurate ground truth or a validated navigation safety classification**.

### 8.3 Memory comparison: trimming disabled vs. every 10 seconds

Both tests were recorded with `tegrastats --interval 5000` on the **headless Jetson host** while distributed mapping was running. These are **total system RAM measurements**, not the memory size of the CuPy pool alone.

| Measurement | Trim `0.0` (disabled) | Trim `10.0` (enabled) |
| --- | ---: | ---: |
| Date | 29 Oct 2026 | 29 Oct 2026 |
| Time window | 13:14:23–13:38:40 | 13:49:35–14:02:26 |
| Recorded duration | 24 min 17 sec | 12 min 51 sec |
| RAM at start | 989 / 3589 MB | 1267 / 3589 MB |
| RAM at end | **2514 / 3589 MB** | **1271 / 3589 MB** |
| RAM change | **+1525 MB** | **+4 MB** |
| Typical behavior | Nearly continuous growth, ≈63 MB/min average | Stable within approximately 1256–1283 MB |
| Swap | 0 MB in the `tegrastats` recording | 14 MB throughout the recording |
| Assessment | **Not acceptable unattended on 4 GB device** | **Preferred tested setting** |

With trimming **disabled**, a subsequent host `ps` sample showed the mapping `python3` process at **2,514,400 KB RSS** (about 2.40 GiB); `free -h` showed 588 MiB available and 9 MiB swap in use. The run did not OOM during the recorded window, but sustained growth significantly reduced headroom.

With trimming **enabled**, RAM was nearly unchanged for almost 13 minutes, even while GPU activity continued. The 14 MB swap value was constant and may have been inherited from previous memory pressure, rather than reflecting ongoing swapping.

**Interpretation:** The A/B comparison is strongly consistent with unused CuPy allocations being retained by the default memory pool when trimming is disabled. It does **not** prove the entire increase is CuPy caching, rule out another memory leak, or establish that every workload will remain stable indefinitely. The tests had different durations and initial conditions.

**Decision:** Keep `cupy_memory_pool_trim_interval_s: 10.0` for the Jetson Orin Nano 4 GB until a controlled benchmark shows another configuration is equally stable and measurably faster.

### 8.4 Hardware observations

In the sustained trim-enabled test, the reported Jetson GPU temperature was approximately 51–52 °C, with intermittent GPU activity (`GR3D_FREQ`). No temperature-related failure was reported. `GR3D_FREQ` is an activity/frequency indicator; it is **not** a direct measurement of kernel runtime or achieved map rate.

To record future tests to a **new file** on the Jetson host:

```bash
tegrastats --interval 5000 | tee ~/elevation_trim_10s_new_test.log
```

Avoid reusing `~/elevation_memory_test.log` with `tee` if you need to preserve previous results: `tee` overwrites the file by default.

## 9. Build/development and diagnostics

### 9.1 Rebuild the Jetson mapping package after code changes

Inside the Jetson Docker container:

```bash
source /opt/ros/jazzy/setup.bash
cd /workspace/ros2_ws

colcon build --symlink-install \
  --packages-select elevation_mapping_cupy \
  --executor sequential --parallel-workers 1 \
  --cmake-args -DBUILD_TESTING=OFF

source install/setup.bash
```

Changing a Python source module may require restarting the node even when built using symlinks. Changing only the mounted YAML setup requires restarting the mapper, not rebuilding its image.

### 9.2 Inspect the Jetson mapping container

On the Jetson host:

```bash
sudo docker ps
sudo docker exec -it <CONTAINER_ID> bash
```

Inside the container, source the ROS environment and set Zenoh-related environment variables as in Section 6.3 if using ROS 2 CLI tools manually:

```bash
source /opt/ros/jazzy/setup.bash
source /workspace/ros2_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ROS_DOMAIN_ID=33
export ZENOH_CONFIG_OVERRIDE='mode="client";connect/endpoints=["tcp/130.243.66.211:7447"];scouting/multicast/enabled=false'

ros2 topic list -t --no-daemon | grep elevation
```

The Jetson host's `ps` process list includes the container's mapping `python3` process, which is useful for comparing process RSS with total device RAM.

### 9.3 Common issues

| Symptom | Check / response |
| --- | --- |
| Jetson cannot receive LiDAR | PC target running; PC router reachable at `130.243.66.211:7447`; Zenoh client override; filtered topic name and fleet boundary. |
| Mapper reports missing transforms | Check namespaced `/terrascout1/tf` and `/terrascout1/tf_static`; `use_sim_time`; restart mapper after Gazebo/`/clock` resets. |
| `GridMap` visible in PC `ros2 topic list`, but not in Foxglove | Check live `/terrascout1/foxglove_bridge` `topic_whitelist` and that Foxglove connects to port `8765`. |
| PC cannot decode `grid_map_msgs/msg/GridMap` | Check `ros2 pkg prefix grid_map_msgs`; rebuild/use updated PC image if package is absent. |
| Jetson RAM grows toward OOM | Confirm `cupy_memory_pool_trim_interval_s: 10.0` is in the active setup YAML and that the mapper was restarted after changing it. |
| Container disappears after Ctrl+C | Expected: `run-jazzy-zenoh.sh` uses `docker run --rm -it`. Source is preserved by bind mount. |
| Jetson monitor black after disabling GUI | Use SSH; restore `graphical.target` if an attached desktop is required. |

## 10. Remaining work

1. **Measure distributed map publication rate and latency.** Check actual PC-side `ros2 topic hz /elevation_mapping_node/elevation_map`, LiDAR rates, end-to-end transport delay, and any dropped messages under motion.
2. **Validate map geometry and semantics.** Compare elevation against known Gazebo terrain, inspect unknown-cell treatment, check map centering, variance, local-frame consistency, and traversability score interpretation.
3. **Check CuPy-only filter numerical equivalence.** Compare against the original PyTorch filter using the actual learned weights and identical height arrays, ideally on a computer with sufficient RAM. Investigate boundary/shape differences.
4. **Longer stability/performance testing.** Repeat controlled trim-enabled runs under sustained motion, and quantify whether 10-second pool trimming affects update latency. Avoid disabling trimming on the 4 GB deployment without a clear performance justification.
5. **Integrate terrain-derived costs with Nav2.** Define how elevation, uncertainty, unknown cells, and traversability map into a conservative costmap/keepout policy; test in simulation before allowing it to influence autonomous driving.
6. **Harden deployment.** Consider a Jetson `systemd` service or a detached container launch, automatic recovery, logs, firewall/host access to TCP 7447, and a repeatable deployment procedure. The current launcher is interactive.
7. **Later:** Extend the distributed topology to PC + NUC + Jetson, preserving existing navigation targets and behavior.

## 11. Safety and maintenance notes

- A populated, visually plausible GridMap **does not validate obstacle avoidance, traversability classification, or a safe Nav2 costmap**.
- Keep distinct the older synthetic demo (local-only, ~10.26 Hz) and the current distributed TerraScout setup (5 Hz **configured**, measured rate pending).
- Do not replace the mapping input with FAST-LIO2's `camera_init` cloud without explicit frame and sensor-origin handling.
- Preserve the custom `traversability_filter_cupy.py`, Jetson setup YAML, and all relevant TerraScout PC changes in version control. The Jetson's `ros2_ws` is outside the PC repository.
- ROS 2/Zenoh connectivity and Foxglove advertisement are separate checks; a valid ROS topic does not imply it appears in Foxglove.
- Maps and filters should be validated with known obstacles and terrain before their output is used to command the physical UGV.

---

**Summary (29 October 2026):** TerraScout's PC simulation and the headless Jetson now exchange filtered LiDAR, TF, clock, and elevation `GridMap` through Zenoh. The Jetson computes elevation, variance, and custom CuPy traversability without PyTorch; the PC receives and visualizes the map in Foxglove. In the recorded comparison, **10-second CuPy memory-pool trimming held total Jetson RAM near 1.27 GB for almost 13 minutes**, whereas disabling trimming led to **+1.525 GB over roughly 24 minutes**. The next milestone is validating mapping frequency, terrain accuracy, and a conservative Nav2 costmap integration.
