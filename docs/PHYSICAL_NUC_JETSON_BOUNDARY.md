# Physical NUC–Jetson ROS topic boundary (Step 24)

## Scope

- Applies only to `TARGET=terrascout-physical` (`deployment="physical-jetson-nuc"`).
- The implementation is in `src/fleet_config/fleet_config/unit.py`, function `distributed_simulation_boundary`.
- The boundary is consumed by `fleet_config/zenoh.py` to create the generated Zenoh router access-control rules and appears in the generated unit `description.json`.
- `terrascout1`, all simulation targets, and `station` are unchanged.
- This step configures **allowed ROS topic exchange**, not a running Jetson node or a tested network connection.

## Physical NUC exports (toward the Jetson/network)

- `/terrascout1/lidar_points_filtered` — `sensor_msgs/msg/PointCloud2`; input for elevation mapping.
- `/terrascout1/tf` — `tf2_msgs/msg/TFMessage`; dynamic transforms.
- `/terrascout1/tf_static` — `tf2_msgs/msg/TFMessage`; static transforms.
- Existing `robot_exports` status/health/position/mission topics remain exported.

## Physical NUC imports (from Jetson)

- `/elevation_mapping_node/elevation_map` — `grid_map_msgs/msg/GridMap`.
- `/terrascout1/oakd/detections_2d` — `vision_msgs/msg/Detection2DArray`.
- `/terrascout1/oakd/detections_image` — `sensor_msgs/msg/Image`.
- Importing Jetson detections intentionally prevents the physical target from also launching the local YOLO detector.

## Differences from `sim-elevation-pc`

- No `/clock` export: the physical target uses wall-clock time and must not publish simulation time.
- No `/terrascout1/oakd/rgb/image_raw` export yet: the current physical `camera_processes()` config publishes `front_camera/image` using `gscam2`; it does **not** establish an OAK-D RGB raw-image publisher with the simulated topic and type. Do not assume these two camera streams are interchangeable.
- Before physical YOLO testing, establish where the OAK-D is physically connected, which node publishes its images, and which topic/type the Jetson detector subscribes to. If the camera is attached directly to the Jetson, no NUC RGB export may be needed.

## Not covered by this change

- Jetson-side ROS 2 / Zenoh startup, router connectivity and correct `ROS_DOMAIN_ID`.
- Network address and interface provisioning on the NUC/Jetson (site and station configuration).
- Running `elevation_mapping_cupy` and YOLO on Jetson, with appropriate GPU/JetPack installation.
- Point-cloud frame correctness, timestamp synchronization, QoS, transform availability, throughput and camera image type.
- Live FAST-LIO2/GNSS handover. `experimental_localization_handover` remains disabled.
- Runtime validation on physical NUC/Jetson hardware.

## Verify

- `git apply --check step24_physical_nuc_jetson_boundary.patch`
- `git apply step24_physical_nuc_jetson_boundary.patch`
- `make render TARGET=terrascout-physical`
- `make build TARGET=terrascout-physical`
- Regression tests: `src/fleet_config/test/test_physical_jetson_boundary.py`.
