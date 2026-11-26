# TerraScout Jetson: one-command elevation mapping + YOLO

**Status: untested integration candidate**, prepared based on your proven Jetson images and the `integration/full-system-nav-base` source. Run in **simulation first**. This is not a tested physical-robot deployment or a turnkey patch for the PC repository.

## What this bundle does

- Starts **two containers** using `./start-terrascout-perception.sh`:
  1. `jetson-jazzy-perception:dev`, derived from your **existing** `jetson-jazzy-elevation:dev`, runs the unchanged `elevation_mapping_cupy` entry point plus a new Python 3.12 ROS 2 adapter. The derived image adds only `ros-jazzy-vision-msgs`.
  2. `ultralytics/ultralytics:latest-jetson-jetpack6`, runs a Python 3.10 CUDA YOLO worker using your `best.pt` model.
- Both use NVIDIA GPU, `network_mode: host`; only the ROS adapter/mapping container participates in Zenoh. YOLO is accessed via **loopback TCP**, using JPEG images and JSON detections (only on this Jetson).
- Adapter subscribes `/terrascout1/oakd/rgb/image_raw` (best-effort QoS), validates detections using your `s_perception_2d.object_record.ObjectRecordHandler`, publishes `/terrascout1/oakd/detections_2d` (standard `vision_msgs/msg/Detection2DArray`, original timestamps) and keeps JSONL logs in `~/jetson-elevation/perception-logs/`. An annotated image is optional.
- The adapter uses a **latest-frame queue** and defaults to 5 FPS with `imgsz=640`; it does not itself throttle the **PC → Jetson** camera stream. JPEG compression consumes some Jetson CPU; benchmark it.
- The code is an integration prototype, not a verbatim execution of `detector_node.py`. It reuses the same validation code and topics, but has not been verified on your Jetson. Compare its outputs against the original node before field use.

## Installation on Jetson host

Copy the entire contents of this bundle into `~/jetson-elevation/`. This does **not** overwrite your existing `Dockerfile`, `Dockerfile.mapping`, `run-jazzy-zenoh.sh` or `start-terrascout-mapping.sh`.

```bash
cd ~/jetson-elevation
cp .env.perception.example .env.perception
nano .env.perception  # change PC_ZENOH_HOST and ROS_DOMAIN_ID as required
chmod +x start-terrascout-perception.sh
./start-terrascout-perception.sh
```

The script builds `jetson-jazzy-perception:dev` **once** if necessary, and subsequently starts both services using Compose. On Ctrl+C, Compose stops the services; the mapping container terminates both the adapter and mapping process.

**Prerequisites:** Docker Compose v2, your existing built `~/jetson-elevation/ros2_ws/install/` workspace, existing images, model at `~/terrascout-yolo/src/s_perception_2d/models/best.pt`, correct Zenoh PC endpoint, and the **PC repo changes described below**. Do not start the old mapping script simultaneously or you will run duplicate mappers.

## Required changes on the simulation PC repository

The current `integration/full-system-nav-base` branch does **not** forward the camera to Jetson and still launches the local detector. These changes are REQUIRED before an end-to-end test.

File: `src/fleet_config/fleet_config/unit.py`

In `distributed_simulation_boundary(...)`, `if deployment == "elevation-pc":`, add this to `exports` alongside the filtered LiDAR and TF:

```python
BoundaryEntry(
    f"/{namespace}/oakd/rgb/image_raw",
    "sensor_msgs/msg/Image",
    5.0,  # optional network downsampling; adjust after performance measurements
    None, # Gazebo bridge publishes in global node namespace
),
```

Add this to `imports` alongside the elevation `GridMap`:

```python
BoundaryEntry(
    f"/{namespace}/oakd/detections_2d",
    "vision_msgs/msg/Detection2DArray",
    None,
    None,
),
```

File: `src/fleet_config/fleet_config/classes/terrascout.py`

The current `perception_processes` launches `YOLO_DETECTOR` locally and creates `enemy_reader` with `after=[YOLO_DETECTOR]`. Modify the **function** like this:

```python
def perception_processes(unit: UnitContext, after: list[str]) -> list[Process]:
    # Distributed detector is declared by its imported ROS topic.
    jetson_yolo = any(
        entry.topic == f"/{unit.namespace}/oakd/detections_2d"
        for entry in unit.boundary.imports
    )
    processes = []
    if not jetson_yolo:
        processes.append(
            node(
                YOLO_DETECTOR,
                package=PERCEPTION_PACKAGE,
                executable="detector_node.py",
                node_name=YOLO_DETECTOR,
                namespace=unit.namespace,
                params=(YOLO_PARAMS,),
                after=after,
                ready=NodeProbe(f"/{unit.namespace}/{YOLO_DETECTOR}"),
            )
        )
    if unit.mode == SIMULATION:
        processes.append(
            node(
                "enemy_reader",
                package=SIMULATION_PACKAGE,
                executable="enemy_reader.py",
                node_name="enemy_reader",
                namespace=unit.namespace,
                after=after if jetson_yolo else [YOLO_DETECTOR],
                ready=NodeProbe(f"/{unit.namespace}/enemy_reader"),
                output="log",
            )
        )
    return processes
```

This preserves the usual local detector for normal targets while offloading it in `sim-elevation-pc`. **Check this against local uncommitted edits and run your repository's existing render/validation tests before starting.** The camera stream must actually reach the Jetson through Zenoh; `5.0` here is only a transport downsampling target, not a guarantee.

Then restart the **PC** simulation with the regenerated configuration:

```bash
make render TARGET=sim-elevation-pc
make up TARGET=sim-elevation-pc
```

With PC simulation running, start the new Jetson script.

## Verify behavior

On Jetson host in a second terminal:

```bash
sudo docker ps
tegrastats --interval 5000
```

On PC, in ROS 2 container (substitute your exact shell command/target):

```bash
ros2 topic hz /terrascout1/oakd/detections_2d
ros2 topic info /terrascout1/oakd/detections_2d --verbose
ros2 topic hz /elevation_mapping_node/elevation_map
```

Expected **if configured and working**: one YOLO detection publisher, one elevation map publisher, no locally launched `detector_node.py` on the PC, and no CPU YOLO fallback on Jetson. `enemy_reader` requires the detection header timestamps to align with the depth cloud timestamps; those are preserved by the adapter, but latency and synchronizer queue capacity still need testing. View structured YOLO logs in `~/jetson-elevation/perception-logs/`.

## Physical robot: outstanding integration

The **Jetson side** accepts either a simulation or physical **uncompressed** `sensor_msgs/msg/Image` topic selected with `YOLO_INPUT_TOPIC`. But your physical robot's current launch declares a `front_camera/image` stream rather than the simulated `oakd/rgb/image_raw`; verify the actual topic *and message type* (`sensor_msgs/msg/Image` vs `CompressedImage`) before changing the setting. The physical deployment also needs the corresponding Zenoh boundary and camera stream export, and must disable its local detector. Those physical-side changes are **not** included in this simulation-first prototype. The model should be validated against actual physical-camera frames.

## Notes / limitations

- This is a *candidate implementation* produced without access to a running Jetson. I could statically validate source syntax but not test model loading, ROS middleware routing, CPU/GPU memory coexistence, or physical camera compatibility.
- GPU memory reported by `torch.cuda.memory_reserved` only covers PyTorch's CUDA allocator, **not** total Jetson RAM use. Test combined memory over time under real sensor traffic.
- Keep `cupy_memory_pool_trim_interval_s: 10.0` in your mapping setup.
- Your original detector's file logging format is reused, but the annotated-image feature is **disabled by default** to reduce overhead. It can be enabled with `YOLO_PUBLISH_ANNOTATED=true` and a matching PC Zenoh import rule for `/terrascout1/oakd/detections_image`.
- Don't change `best.pt` based on performance of the blank synthetic test; evaluate on representative camera images.
- For physical driving, retain independent collision monitoring and validate map/detector freshness and fail-safe responses to stale perception.
