# Local SAGE Setup Notes

This document is the implementation and system-behavior reference for TerraScout: how the Docker/ROS deployment is assembled, how perception becomes a keepout, how missions reach Nav2, what the recent feature work changed, and what remains to be validated. Operator instructions for building, starting, and using the menu/CLI are kept in [README.md](README.md).

Some values are specific to each development computer. Do not blindly copy machine-specific values between computers.

## Related setup and technical notes

- [NVIDIA setup](NVIDIA_SETUP.md) — GPU drivers, NVIDIA Container Toolkit, and Docker GPU access.
- [Nav2 CPU fix](NAV2_CPU_FIX.md) — CPU/resource troubleshooting for Nav2.
- [FAST-LIO2 URDF integration](FASTLIO2_URDF_integration.md) — robot description and FAST-LIO2 integration notes.
- [Jetson migration checklist](TerraScout_Jetson_Migration_Checklist.md) — steps and validation for the distributed Jetson setup.
- [URDF-based LiDAR self-filtering](URDF-Based_LiDAR_self_filtering.md) — filtering robot-body points from LiDAR data.

## Implemented work and where it lives

This section indexes the main TerraScout work documented here. It describes implementation intent and current wiring; it does not claim that every behavior has been validated on physical hardware.

### Fleet and Docker deployment

Python target definitions are rendered into Docker Compose units. Simulation and physical builds use different image/build modes, and TerraScout processes run together in a namespaced unit.

Locations: `Makefile`, `src/fleet_config/fleet_config/targets.py`, `src/fleet_config/fleet_config/classes/terrascout.py`, and `docker/`.

### PC–Jetson deployment

The PC simulation can expose selected ROS traffic through Zenoh to Jetson mapping/perception. SSH is for host administration; machine addresses, ROS domain, and Gazebo partition must match the intended deployment and isolation.

Locations: `src/fleet_config/fleet_config/targets.py`, `src/fleet_config/fleet_config/model.py`, and [TerraScout_Jetson_Migration_Checklist.md](TerraScout_Jetson_Migration_Checklist.md).

### Simulation, sensors, and forest world

The Gazebo model defines TerraScout's simulated sensors; bridges expose topics to ROS and Foxglove. Terrain generation, forest assets, and the world template define the simulated environment and spawn context, with tree placement/elevation fitted to the generated heightmap.

Locations: `src/terrascout_description/urdf/scout.urdf.xacro`, `src/terrascout_description/config/gz_bridge.yaml`, `src/terrascout_mission/config/simulation/foxglove_bridge.yaml`, `src/fleet_simulation/terrain/`, `src/fleet_simulation/worlds/`, and `src/fleet_simulation/models/`.

### Perception, localization, and tracking

`s_perception_2d` runs the configured detector against OAK-D RGB input and publishes 2D detections. `enemy_reader` synchronizes detections, depth, and timestamped robot pose, converts observations to map ENU, and feeds the class-aware `object_tracker`. Confirmed enemy tracks are published and separately sent to the keepout service; detailed behavior is described below.

Locations: `src/s_perception_2d/`, `src/fleet_simulation/src/enemy_reader.py`, and `src/fleet_simulation/src/object_tracker.py`.

### Keepouts and visualization

Named GPS keepouts are accepted by services, converted using the configured datum, persisted during the unit run, published for Nav2, and exposed to Foxglove as a 3D marker and 2D geographic GeoJSON.

Locations: `src/fleet_interfaces/srv/`, `src/fleet_common/fleet_common/keepout_zone.py`, `src/terrascout_mission/terrascout_mission/keepout_server.py`, and `src/terrascout_mission/terrascout_mission/keepout_geometry.py`.

### Mission execution and Foxglove input

Mission requests are validated and sequenced by the mission server, reported as state, and executed through Nav2 goals. Mission-level route ordering is distinct from Nav2 path planning. In Foxglove, map-frame clicks can form a waypoint draft and submit ordered or unordered missions through the same server path.

Locations: `src/fleet_common/fleet_common/send_mission.py`, `src/terrascout_mission/terrascout_mission/mission_server.py`, `src/terrascout_navigation/`, `src/terrascout_mission/terrascout_mission/foxglove_mission_input.py`, and `src/terrascout_mission/config/`.

### Mission test menu

This host-side test client exercises route presets, Home, waypoint updates, mission state/control, keepout CRUD, and simulation-only localization fault injection. It is not a runtime dependency.

Location: `src/terrascout_mission_tester/`.

### Recording and evaluation

Recording-enabled targets capture simulation topics. Post-processing compares FAST-LIO odometry to ground truth and produces a mission report when recorded mission state is available.

Locations: `src/fleet_config/fleet_config/targets.py`, `src/terrascout_navigation/terrascout_navigation/scripts/process_mission_recording.py`, and `src/terrascout_navigation/terrascout_navigation/scripts/compare_fast_lio_gt.py`.

### Navigation safety and tuning

Ground segmentation, costmaps, controller behavior, behavior-tree recovery, and Collision Monitor jointly affect obstacle avoidance. The current values are experimental and require scenario testing.

Locations: `src/terrascout_navigation/config/ground_seg.yaml`, `src/terrascout_navigation/config/nav2.yaml`, `src/terrascout_navigation/config/navigate_to_pose.xml`, and `patches/ground_segmentation_ros2/`.

### Robot models

Simulated Apache and Ka-52 model visuals/collision geometry use the current 1.5× scale adjustment.

Locations: `src/fleet_simulation/models/apache_friend/model.sdf` and `src/fleet_simulation/models/ka52_enemy/model.sdf`.

The sections below explain how these components interact, which coordinate/frame assumptions matter, where persistent state lives, and how to inspect/test the result. For concise run commands and menu key-by-key usage, see [README.md](README.md).

## System overview and PC–Jetson workflow

The repository's Makefile renders a fleet configuration from `src/fleet_config/fleet_config/targets.py`. Docker Compose runs each configured unit with its own ROS 2 processes and namespace. Within a host, SAGE uses Zenoh to connect units and the station. ROS domain IDs isolate unrelated ROS graphs; machines that are meant to exchange ROS data must use the same domain.

For `sim-elevation-pc`, the PC runs Gazebo, FAST-LIO2, TerraScout/Nav2, and the fleet router. The Jetson runs the separate `elevation_mapping_cupy` workspace and connects to the PC's Zenoh TCP endpoint. LiDAR, camera, clock, and TF flow PC → Jetson; elevation GridMap and perception results flow Jetson → PC. SSH is only the remote administration shell—it does not transport ROS topics. Zenoh carries the distributed ROS traffic.

```text
PC: Docker SAGE (Gazebo / FAST-LIO2 / Nav2 / Foxglove)
     └── Zenoh TCP, PC_IP:7447
             ├── filtered LiDAR, camera, /clock, TF ──→ Jetson mapping/perception
             └── elevation GridMap and detections   ←── Jetson

Developer laptop/PC ── SSH ──→ Jetson host (shell/launch/admin only)
```

### Connect to the Jetson over SSH

SSH requires the Jetson to be reachable over the network, with its SSH server enabled and a valid account. The repository does not define the Jetson's username, password, or a permanent address; obtain those from the device owner/network configuration. Do not commit credentials or private keys.

1. Find the Jetson's address on the Jetson itself with `ip -4 addr`, or obtain its DHCP reservation from the network administrator. Find the PC's address separately; they are not interchangeable.
2. From the PC/developer machine, connect with the account and Jetson address supplied for your device:

    ```sh
    ssh <jetson_user>@<jetson_ip>
    ```

    On first connection, verify the host-key fingerprint through a trusted channel before accepting it. If public-key login is configured, use an existing key or create one with `ssh-keygen` and install the public key using `ssh-copy-id <jetson_user>@<jetson_ip>`. Never copy the private key to the Jetson.
3. Optional: add a local-only `~/.ssh/config` alias (do not commit it):

    ```sshconfig
    Host sage-jetson
         HostName <jetson_ip>
         User <jetson_user>
         IdentityFile ~/.ssh/id_ed25519
    ```

    Then connect with `ssh sage-jetson`. Replace placeholders with values confirmed for your machine. The SSH address is the Jetson's IP; the Jetson's Zenoh endpoint must instead point to the PC's IP.
4. Once connected, inspect the mapping workspace and launcher on the Jetson. The currently documented example is `~/jetson-elevation/start-terrascout-mapping.sh`; verify that this path exists on the device before running it. Run the launcher on the Jetson host (or use the device's documented procedure), and confirm its container/node has the expected ROS and Zenoh environment.

See [TerraScout_Jetson_Migration_Checklist.md](TerraScout_Jetson_Migration_Checklist.md) for the topic boundary, current example endpoint, and end-to-end GridMap checks. Its IP/interface examples are machine-specific and must be replaced with current values.

### Build and run from the PC

Run these commands from the repository root. For the all-local simulation, use `sim`; for the distributed Jetson mapping topology, use `sim-elevation-pc`:

```sh
make build TARGET=sim
make up TARGET=sim
# Stop cleanly from another terminal, if needed:
make down
```

For the distributed target, first configure the PC uplink address and interface in `src/fleet_config/fleet_config/targets.py`, then build and run:

```sh
make render TARGET=sim-elevation-pc
make build TARGET=sim-elevation-pc
make up TARGET=sim-elevation-pc
```

Once the fleet is running, `make shell TARGET=sim-elevation-pc SERVICE=unit-terrascout1` opens a shell in the PC's TerraScout container. Use SSH separately to open a shell on the Jetson host; do not expect `ros2` commands over SSH to see PC topics unless that Jetson process has joined the configured Zenoh graph.

### What Docker starts and how to inspect it

`make build TARGET=<target>` first renders the fleet definition, builds the mode-specific Docker image, builds ROS packages with colcon, runs the package tests, and for simulation also builds PX4 SITL. It is the build/test step; it does not start the robot stack. `make up TARGET=<target>` renders the fleet again, records the source state and rendered configuration under `runs/`, then starts the generated Compose services in the foreground. The rendered files are under `.sage/render/<target>.<instance>/`; the active run is linked under `.sage/run/<instance>/`. Each robot unit has namespaced ROS processes inside its container. The fleet router and the units exchange configured topics/services over Zenoh. The simulation world has GUI/Gazebo host requirements, including working NVIDIA container support for the configured GPU target.

```sh
make list
make build TARGET=sim
make up TARGET=sim
```

Keep `make up` running in its terminal. In another terminal, inspect logs or enter the TerraScout container:

```sh
make logs TARGET=sim
make shell TARGET=sim SERVICE=unit-terrascout1
```

`make shell` runs `ros-workspace-exec` in the selected container, so ROS and the installed workspace are sourced there. For one command without an interactive shell, use `make exec TARGET=sim COMMAND='ros2 topic list'`. Stop with Ctrl+C in the `make up` terminal and wait for Compose to stop, or use `make down TARGET=sim` from another terminal. `TARGET` is required unless you first saved one using `make use TARGET=<target>`; using explicit `TARGET` avoids accidentally addressing another rendered target. Do not mix simulation and physical builds in the same `build/` and `install/` output without cleaning as directed by the build error.

To see exactly what a target will run before launch, use `make render TARGET=sim` and inspect `.sage/render/sim.1/fleet.yaml` and the generated unit/Compose files. After a run, inspect the matching `runs/<timestamp>-<target>.<instance>/compose.log`, `git-status.txt`, and `git.diff`; the run record captures working-tree changes as well as the commit.

## Detection-to-keepout pipeline

### How the feature developed

This pipeline was assembled in stages, rather than being one node that detects and avoids objects:

1. **YOLO model and image detections.** The `s_perception_2d` package supplies the trained model and detector node. The detector consumes OAK-D RGB images and publishes 2D bounding boxes with class IDs/scores on `oakd/detections_2d`; an annotated image can also be published for visual inspection. In the distributed `sim-elevation-pc` setup, the Jetson can run this detector and send detections to the PC over the configured Zenoh boundary. The model's class labels must match the IDs/names expected by the reader (`Ka-52 Enemy` for automatic enemy keepouts).
2. **3D localization in `enemy_reader`.** The simulation-side reader pairs detections with the organized OAK-D point cloud, uses camera calibration and a timestamp-matched ENU odometry pose, and estimates a map-frame XYZ position for each detection. This bridges 2D image detection to a world-coordinate observation; it is not itself a tracker or a Nav2 keepout.
3. **Temporal tracking in `object_tracker`.** The reader passes depth-backed observations to `ObjectTracker`. The tracker associates observations by class and 3D position, updates a Kalman filter, and maintains tentative/confirmed tracks. Confirmation requires ten matched depth observations and confidence-weighted existence score of at least 4; expected visible misses reduce the score. Only confirmed tracks labelled `Ka-52 Enemy` are selected for the automatic keepout path.
4. **Enemy message and keepout service are separate outputs.** The reader publishes confirmed enemy X/Y tracks to `enemies` for consumers, and separately calls `set_keepout_zone` with GPS coordinates for the keepout server. Publishing `enemies` does not itself create a keepout.
5. **Keepout server and navigation.** The server converts the GPS center into the configured map frame, builds a named rectangle, persists it, and publishes the geometry/obstacles consumed by Nav2 collision/costmap configuration. It also publishes Foxglove visualizations. This makes the zone available to navigation; it does not prove that the original detection or GPS conversion was accurate.
6. **Mission server.** `mission_server` owns mission readiness, validation, waypoint ordering, Nav2 action goals, state reporting, recovery, and keepout-aware waypoint handling. It checks destinations against the persisted keepout definitions; Nav2 separately plans each active leg using its costmaps. The mission server does not run YOLO or create enemy tracks.
7. **TerraScout mission tester.** `terrascout_menu.py` is an operator/test client, not part of the autonomous runtime pipeline. It calls the mission and keepout CLI/service paths to exercise routes, waypoints, Home, state, keepouts, suspend/resume, and simulation fault injection. A launched command is not evidence that the mission was accepted or completed; use the mission state and node logs to confirm.

Relevant source locations:

- YOLO package/model: `src/s_perception_2d/`
- Detection/depth localization and keepout client: `src/fleet_simulation/src/enemy_reader.py`
- Track association/filter/confirmation: `src/fleet_simulation/src/object_tracker.py`
- Keepout storage, Nav2 outputs, and visualization: `src/terrascout_mission/terrascout_mission/keepout_server.py`
- Mission readiness/execution/navigation: `src/terrascout_mission/terrascout_mission/mission_server.py`
- Foxglove map-click mission input: `src/terrascout_mission/terrascout_mission/foxglove_mission_input.py`
- Manual keepout CLI: `src/fleet_common/fleet_common/keepout_zone.py`
- Operator/test menu and preset waypoints: `src/terrascout_mission_tester/`
- Unit wiring, topics, and Jetson deployment boundary: `src/fleet_config/fleet_config/classes/terrascout.py`, `src/fleet_config/fleet_config/unit.py`, and `src/fleet_config/fleet_config/targets.py`

In the distributed target, the Jetson supplies YOLO detections on `/terrascout1/oakd/detections_2d`; the depth cloud and robot state are provided by the PC-side simulation. `enemy_reader` subscribes to detections and `/terrascout1/oakd/points` and approximately synchronizes them (queue 10, up to 0.1 s apart). It also uses `/terrascout1/oakd/rgb/camera_info`, timestamped `/terrascout1/fixposition/odometry_enu`, and the datum from `/terrascout1/fixposition/datum`.

For each bounding box, the reader scales RGB pixels to the organized cloud resolution, selects valid forward-facing cloud samples, rejects depth outliers with a median/MAD filter, and requires at least 10 valid points. It transforms the resulting camera/robot-relative position into map ENU using the odometry pose at the sensor timestamp. The point-cloud convention is currently assumed to be Gazebo FLU (+X forward, +Y left, +Z up); verify this assumption against the live cloud before treating coordinates as ground truth. Perception diagnostics print pixel/bounding-box, cloud XYZ, robot ENU, camera offset, and resulting map ENU.

`ObjectTracker` then class-associates and gates detections, updates 3D Kalman tracks, and tracks tentative/confirmed status. A new track starts at observation 1; each matched depth observation increments the count and adds its YOLO confidence (clamped to [0, 1]) to the existence score. Confirmation requires at least 10 matched observations and an existence score of at least 4. Thus, ten high-confidence hits are required at minimum; lower-confidence hits or expected visible misses can delay confirmation. Only confirmed tracks with class ID `Ka-52 Enemy` are used for keepouts. The `enemies` message is another reader output; it is not a prerequisite input to the keepout server because the reader sends the service request directly.

Every 0.5 s, the reader checks confirmed enemy tracks. It converts each track's ENU position to WGS84 latitude/longitude using the current datum, then calls the namespaced `set_keepout_zone` service with label `enemy_track_<id>`, 4 m × 4 m dimensions, and yaw 0°. A zone is sent again if that track moves at least 1 m from its last successfully accepted keepout position. Pending service calls are deduplicated. Accepted/rejected service results and GPS/ENU coordinates are logged. The reader also publishes confirmed enemies on `enemies` and track data on `tracked_objects`.

The keepout server validates and converts the received GPS center back to map ENU using the locked datum, builds the square polygon, stores the named zone in `/run/sage/<namespace>/keepout_zones.json`, and republishes Nav2 collision/costmap data. For Foxglove it publishes the 3D purple volume on `keepout_zone_marker` and a GPS GeoJSON polygon with a purple center point on `keepout_zones_geo`. The server log reports the zone GPS, computed ENU, datum, frame, dimensions, and yaw. Confirm the same center appears in the Foxglove 2D map and that the 3D volume is in the expected map location; successful service acceptance alone does not validate the sensor-frame assumption or perception accuracy.

### Mission request to robot motion

The mission client (CLI, menu, or Foxglove click-input service) submits an `ExecuteMission` plan to `mission_server`. The server first checks readiness and localization, validates route/keepout constraints, then sends the active destination to Nav2 through `NavigateToPose`. Nav2 plans and controls each active leg from its configured maps/costmaps; the server tracks command/mission state and publishes waypoint GeoJSON for Foxglove. Ordered mode preserves the submitted order; unordered mode selects/reorders remaining eligible waypoints using straight-line geographic distance, which is not the same as a Nav2 path-cost calculation. Keepout updates can invalidate destinations and affect the remaining mission; test active-goal cancellation, failure reporting, and continuation in simulation. On takeover or restart the mission can be suspended; resume requires an explicit operator request, healthy localization/readiness, and a fresh path. Consult the current Nav2 and mission parameters before physical operation.

Manual keepout create/list/read/update/delete is also available through `ros2 run fleet_common keepout_zone.py`; examples are in the README. A manual zone and an enemy-generated zone use the same service/server path.

## TerraScout mission tester menu

Start the stack first, then run the tester from the workspace root in a second host terminal:

```sh
python3 src/terrascout_mission_tester/terrascout_menu.py
```

The menu reads `src/terrascout_mission_tester/waypoints.json`. Set `TERRASCOUT_NAMESPACE` to select another robot namespace; by default it targets `terrascout1`. If the host has the ROS package, commands run directly; otherwise the menu wraps them through `make exec`. In that fallback, select the same target first (for example `make use TARGET=sim`) or otherwise ensure the selected target matches the running container. Mission choices 1–4 use the preset routes; ordered mode preserves waypoint order, unordered mode asks the mission CLI to reorder. Each send prompts for return-to-start, saved Home, or no explicit return flag. It creates a timestamped results bundle containing the selected route, metadata, and a client log. A message that the client started is not proof that the mission was accepted—check Foxglove and mission state/logs.

Other menu options add/clear waypoints, set or navigate to Home, suspend/resume, inspect mission state, manage keepout zones, and inject/clear the simulation-only localization fault. Keepout creation accepts a test waypoint or GPS coordinates plus dimensions/yaw; list and delete operate on zone labels. Localization fault injection deliberately invalidates simulated localization, requires explicit confirmation, and should never be used on physical hardware. Menu selections that start a mission do not stop the menu from another process; inspect the saved log/result bundle and mission state to establish outcome.

---

## 1. ROS Domain ID

File:

```text
src/fleet_config/fleet_config/model.py
```

Use the same ROS domain ID for nodes that must communicate across the PC–Jetson deployment. Use a different ID to isolate an unrelated fleet/development setup; do not change just one machine in a connected deployment.

### dator10

```python
@property
def domain_id(self) -> int:
    return 133
```
Example:

| Deployment | ROS Domain ID |
|---|---:|
| Current SAGE configuration (PC; Jetson must match) | `133` |
| Independent ROS graph (example only) | choose a different ID |

---

## 2. Gazebo Partition

File:

```text
src/fleet_config/fleet_config/model.py
```

SAGE originally generates the Gazebo partition from the SAGE instance:

```python
@property
def gz_partition(self) -> str:
    return f"sage_{self.index}"
```

This causes a problem when two computers on the same network both run the same SAGE instance.

For example, both computers using from the git repo:

```text
sage_1
```

can cause their Gazebo simulations to discover and interact with each other.

This can result in behavior such as controlling the TerraScout on one computer and seeing the TerraScout on the other computer move as well.

Use a unique machine name as part of the Gazebo partition.

### dator10

```python
@property
def gz_partition(self) -> str:
    return f"dator10_sage_{self.index}"
```
The resulting partitions for SAGE instance 1 are:

Use distinct partition names for independent Gazebo simulations on the same network. The current checked-in partition is `dator10_sage_<instance>`; update the machine-specific prefix when using another host.

---

## 3. Foxglove Station Port

Use the Station Foxglove WebSocket address:

```text
ws://127.0.0.1:8767
```

## 4. TerraScout Simulation Model and Topics

When adding, removing, or modifying simulated TerraScout components or sensors, multiple files may need to be updated.

The main files are:

```text
src/terrascout_description/urdf/scout.urdf.xacro
src/terrascout_description/config/gz_bridge.yaml
src/terrascout_mission/config/simulation/foxglove_bridge.yaml
```

### `scout.urdf.xacro`

File:

```text
src/terrascout_description/urdf/scout.urdf.xacro
```

Contains the TerraScout robot model and simulated components/sensors.

For example, simulated cameras and depth sensors are defined here.

Changes here may add or remove:

- sensors
- cameras
- depth cameras
- links
- Gazebo plugins
- sensor parameters
- simulation topics

---

### `gz_bridge.yaml`

File:

```text
src/terrascout_description/config/gz_bridge.yaml
```

Contains ROS 2 ↔ Gazebo topic bridges.

When a sensor or simulated component publishes a Gazebo topic that is needed by ROS 2, its bridge should be configured here.

If a sensor is removed from the model, check whether its bridge should also be removed.

---

### `foxglove_bridge.yaml`

File:

```text
src/terrascout_mission/config/simulation/foxglove_bridge.yaml
```

Controls which ROS topics are exposed through the Foxglove bridge.

When adding a new simulated sensor, its topics may need to be added here before they appear in Foxglove.

These topics should also use the appropriate QoS configuration where required.

---

## 5. Record and Compare FAST-LIO2 Odometry

Recording is disabled for the default `sim` target. Use `sim-record` for the standard recording simulation, or `sim-elevation-pc` for the elevation-PC Zenoh deployment, which currently also enables recording in `src/fleet_config/fleet_config/targets.py`.

From the workspace root, build and start the elevation-PC deployment:

```sh
make build TARGET=sim-elevation-pc
make up TARGET=sim-elevation-pc
```

For the standard local recording simulation, run `make build TARGET=sim-record` and `make up TARGET=sim-record` instead. Confirm whether recording is enabled for the selected target in `src/fleet_config/fleet_config/targets.py` and that the recorder reports ready in startup output. The elevation-PC uplink address and interface are configured in that file; update them if the computer's network interface/address changes.

Run the mission or route you want to evaluate while the simulation is running. To finish the recording, press **Ctrl+C** in the terminal running `make up` and wait for the stack to stop and the prompt to return. Alternatively, run `make down` from a second terminal. Do not kill the recorder/container forcefully; it needs to shut down normally to write the bag metadata.

Stop the stack normally with Ctrl+C in `make up` or `make down` from another terminal; do not force-kill the recorder, because it must finalize `metadata.yaml`. For SAGE instance 1, recordings are written under `.sage/run/1/recordings/terrascout1/` and completed runs are archived under `runs/<run-id>/recordings/terrascout1/`. Use the matching instance directory if `SAGE_INSTANCE` differs. Verify the new bag contains `metadata.yaml`, then run the processor from the workspace root:

```sh
source /opt/ros/jazzy/setup.bash
source install/host_interfaces/setup.bash
python3 src/terrascout_navigation/terrascout_navigation/scripts/process_mission_recording.py --headless
```

Create the host interface overlay once after cloning the workspace (or after changing `fleet_interfaces` definitions):

```sh
source /opt/ros/jazzy/setup.bash
colcon build --base-paths src/fleet_interfaces \
    --build-base build/host_interfaces \
    --install-base install/host_interfaces
```

For host-side bag processing, do not source the workspace's top-level `install/setup.bash`: it may reference container-only package paths and omit importable generated message modules. The separate `install/host_interfaces` overlay supplies `MissionState` and `TrackedObjectArray` to the processor and its child scripts. The normal processing command above only needs to source that already-built overlay. The processor checks for the message types before doing any work and prints a setup hint if the overlay is missing or stale.

Without `--bag`, the processor selects the completed bag with the newest metadata-file modification time across the active recording directory and archived runs. To choose a specific recording, pass `--bag /absolute/path/to/rosbag_directory`. A mission report requires a positive mission ID in the recorded state topic; if none was recorded, the processor still keeps the FAST-LIO comparison and notes that the mission report was skipped.

The comparison script is `compare_fast_lio_gt.py` (there is no `compare_odom_mcap.py` file). It reads `/terrascout1/fast_lio/odometry` and `/terrascout1/odom_gt`, then writes its timestamped error CSV and three plots into the same `mission_results/mission_*` bundle as the mission report. It does not use or create a separate `fast_lio_error_results/` directory. The filename prefix uses Swedish local time (Europe/Stockholm), converting rosbag folder timestamps from UTC. `--show-trajectory` opens the overlay after saving it. Its errors are relative trajectory drift after first-pose alignment, not absolute localization error.

The interactive tester at `src/terrascout_mission_tester/terrascout_menu.py` offers four route presets using `src/terrascout_mission_tester/waypoints.json`. WP7–WP20 are generated simulation coordinates, not recovered historical values. The global costmap is a 110 m × 110 m rolling window. Mission legs/home distance may be up to 200 m; each NavigateToPose request is split into intermediate targets no farther than 35% of the smaller global window dimension (38.5 m for the current config). Nav2 plans each segment against the current costmap; this is not a guarantee that an obstacle-blocked segment is reachable. Validate long routes in simulation.

Current experimental navigation tuning is in `src/terrascout_navigation/config/nav2.yaml` and `config/navigate_to_pose.xml`: local costmap update/publish rates are 10/8 Hz; global rates are 3/2 Hz; local/global inflation radii are 0.9/1.0 m with scaling factors 5.0/6.0; MPPI maximum forward speed is 1.0 m/s and forward acceleration limit is 2.5 m/s². Angular response has also been reduced experimentally. Collision Monitor uses outer 30%-slowdown, middle 60%-slowdown, and inner stop polygons. These settings have not established the cause of the reported startup turnaround. Compare initial robot heading, first goal, planned path, and `cmd_vel_nav`/`cmd_vel_auto` in the same `map` frame before making further controller changes. Validate obstacle detection, slow/stop activation, controller timing, and recovery in simulation; these settings cannot prevent collisions if obstacles are absent from the input cloud.

---

**Mission route length:** the fleet configuration permits mission legs and home distances up to 200 m. The rolling global costmap is 110 m × 110 m; the mission server stages each longer destination through intermediate goals sized at 38.5 m under the current configuration. Every segment still requires a valid Nav2 path, and the mission destination is not marked complete until its actual goal is reached. Confirm rendered parameters and test long routes, blocked segments, cancellation, and backtracking. Increasing costmap dimensions increases memory and planning workload.

### Mission ordering and keepout updates

- `ordered` missions preserve the submitted waypoint sequence, including after a keepout-zone update. The next waypoint remains the next eligible destination in that order.
- `unordered` missions optimize the full remaining route after a keepout-zone update, using straight-line geographic distance and the configured mission start/return point. Destinations inside keepout zones are excluded as failed before reordering.
- If an updated keepout covers the active waypoint, the mission server cancels/stops that active navigation goal before marking the waypoint failed and continuing under the configured waypoint-failure policy. Future blocked waypoints are skipped.
- Nav2 replans each active leg using its costmaps, independently of the mission-level order optimizer. The optimizer does not score Nav2 path lengths or establish reachability; a selected route can still contain an unreachable leg.

For validation, test both modes with a keepout covering (1) the active destination and (2) a future destination. Confirm ordered missions retain their remaining sequence, unordered missions update their selected order when needed, and blocked commands appear as failed in mission state/visualization. A successful build does not prove live Nav2 obstacle updates or cancellation timing; verify these in simulation.

### MPR-09 policy proposal — project-owner approval required

This is a draft for review, not evidence of approval. Before assessed trials, the project owner must approve the policy and the team must record the approver, date, and approved revision here or in the trial procedure.

1. **Ordering:** an ordered mission follows its submitted sequence. An unordered mission selects the remaining route with the shortest complete straight-line tour from the current position through eligible destinations and back to mission start. Recalculate the unordered remainder after a keepout update makes a destination invalid. This is a geographic-distance heuristic; it does not compare Nav2 planned path lengths and does not prove reachability.
2. **Keepout updates:** apply each accepted named-zone add/update/removal to Nav2. If the active destination lies inside a new zone, cancel its goal, wait for cancellation, mark that command failed with the zone/reason, then continue. Mark future destinations inside the zone failed before selecting a new unordered order. Ordered routes retain their remaining order.
3. **Unreachable destination / recovery period:** allow the configured Nav2 behavior-tree recoveries to complete, then make one mission-layer retry of the same waypoint after the configured 2.0 s delay (`waypoint_retry_limit: 1`). If that attempt also fails, record the command ID, outcome, and failure reason; mark it failed and continue to the next eligible positional command when `continue_after_waypoint_failure` is true. If no eligible destination remains, fail the mission rather than silently treating the route as complete. The approved procedure must confirm that this retry window is the intended recovery period.
4. **Restoration:** after operator takeover or process restart, keep autonomy suspended. Resume only after explicit operator request, valid/fresh localization, navigation readiness, and a fresh successful path to the current destination. Do not automatically resume or navigate home while localization is invalid.

**Approval record:** `Pending — obtain project-owner approval before marking MPR-09 or MIS-15 complete.` Run and retain tests for each rule, then conduct scripted simulation checks and the required assessed-trial validation.

**MIS-06 simulation evidence (2026-10-23):** In `runs/20261023T083815Z-sim-record.1/recordings/terrascout1/rosbag2_2026_10_23-08_38_21/`, the first recorded invalid localization fix was at 55.300 s. Ground-truth odometry fell below 0.02 m/s by 55.588 s, approximately 0.29 s after the invalid fix, within the 2 s simulation limit. This does not establish physical-robot compliance or complete the required assessed fault-trial recording; retain those as outstanding verification.

For simulation fault injection, call `/terrascout1/gnss_shim/set_localization_fault` with `std_srvs/srv/SetBool` (`data: true` to inject, `data: false` to clear). Localization loss suspends the mission; resuming requires restored valid localization, navigation readiness, a fresh path, and an explicit operator request.

## 6. Quick Checklist for a New Development PC

Before running the simulation on another computer on the same network:

1. Choose a unique ROS domain ID.

2. Set it in:

   ```python
   @property
   def domain_id(self) -> int:
       return <unique_domain_id>
   ```

3. Choose a unique machine name for the Gazebo partition.

4. Set:

   ```python
   @property
   def gz_partition(self) -> str:
       return f"<machine_name>_sage_{self.index}"
   ```

5. Build the project if necessary:

   ```bash
   make build
   ```

6. Start the simulation:

   ```bash
   make up
   ```

7. Verify the actual Gazebo partition:

   ```bash
   docker exec sage1-world-1 sh -c 'pid=$(pgrep -f "gz sim" | head -1); tr "\0" "\n" < /proc/$pid/environ | grep GZ_PARTITION'
   ```

8. Verify that controlling the TerraScout on one development computer does not move the simulation on another computer.

---

## Important

The two settings that must be unique between development computers are:

```python
@property
def domain_id(self) -> int:
    return 133
```

and:

```python
@property
def gz_partition(self) -> str:
    return f"dator10_sage_{self.index}"
```

This keeps the ROS 2 and Gazebo simulations separated between development computers.

---

## 7. Keepout-Zone Implementation Map

Files changed for the keepout-zone implementation:

```text
src/fleet_common/CMakeLists.txt
src/fleet_common/fleet_common/keepout_zone.py

src/fleet_interfaces/CMakeLists.txt
src/fleet_interfaces/srv/GetKeepoutZone.srv
src/fleet_interfaces/srv/SetKeepoutZone.srv

src/fleet_config/fleet_config/classes/terrascout.py

src/terrascout_mission/CMakeLists.txt
src/terrascout_mission/package.xml
src/terrascout_mission/terrascout_mission/keepout_geometry.py
src/terrascout_mission/terrascout_mission/keepout_server.py
src/terrascout_mission/test/test_keepout_geometry.py
src/terrascout_mission/config/simulation/foxglove_bridge.yaml
src/terrascout_mission/config/physical/foxglove_bridge.yaml

src/terrascout_navigation/config/nav2.yaml
src/terrascout_navigation/config/navigate_to_pose.xml
```

### fleet_common

#### `src/fleet_common/CMakeLists.txt`

Installs keepout CLI scripts into the ROS package runtime path (`lib/fleet_common`) so they can be run through `ros2 run`.

#### `src/fleet_common/fleet_common/keepout_zone.py`

User-facing CLI and service client for keepout CRUD-style operations:

- create/update (upsert): call `set_keepout_zone`
- read: call `get_keepout_zone`
- delete: call `set_keepout_zone` with `--clear`

It validates numeric inputs for non-read, non-delete operations.

---

### fleet_interfaces

#### `src/fleet_interfaces/CMakeLists.txt`

Registers keepout interfaces for ROSIDL generation.

#### `src/fleet_interfaces/srv/SetKeepoutZone.srv`

Service contract for create/update/delete behavior:

- request contains label + geometry fields + `clear`
- response returns `accepted` and `reason`

#### `src/fleet_interfaces/srv/GetKeepoutZone.srv`

Service contract for read behavior:

- request contains `label`
- response returns `found`, `reason`, zone geometry, and `active`

---

### fleet_config

#### `src/fleet_config/fleet_config/classes/terrascout.py`

Adds and configures keepout server process startup for TerraScout.

This is what wires keepout server execution into the normal unit lifecycle.

---

### terrascout_mission

#### `src/terrascout_mission/CMakeLists.txt`

Installs keepout runtime node and keepout unit tests as part of package build/testing.

#### `src/terrascout_mission/package.xml`

Declares dependencies required by keepout publication and server runtime (for example geometry message support).

#### `src/terrascout_mission/terrascout_mission/keepout_server.py`

Runtime keepout server node. Responsibilities:

- lock map datum from GNSS shim
- serve `set_keepout_zone` and `get_keepout_zone`
- manage in-memory zones by label
- publish zone polygon to collision-monitor topic
- publish sampled zone points to costmap obstacle `PointCloud2` topic
- periodically republish active zone points for late subscribers

Startup contract:

- starts empty (no active keepout zone)
- publishes an empty keepout cloud by default, and non-empty keepout data after a set/update request

#### `src/terrascout_mission/terrascout_mission/keepout_geometry.py`

Pure geometry helpers used by server logic and tests.

#### `src/terrascout_mission/test/test_keepout_geometry.py`

Regression tests for deterministic rectangle generation and geometry validation.

#### `src/terrascout_mission/config/simulation/foxglove_bridge.yaml`

Whitelists keepout topics so simulation Foxglove can visualize keepout polygon/point data.

#### `src/terrascout_mission/config/physical/foxglove_bridge.yaml`

Physical deployment equivalent of simulation whitelist so the same keepout topics are exposed.

---

### terrascout_navigation

#### `src/terrascout_navigation/config/nav2.yaml`

Adds keepout obstacle source to global and local costmap obstacle layers.

Important integration points:

- include keepout source in `observation_sources`
- use `PointCloud2`
- point topic to `/<robot_namespace>/keepout_obstacle_points`

Without this, Nav2 does not consume keepout points even if server publishing works.

---

## 7. TerraScout Forest Navigation Tuning

Current experimental navigation settings are in:

```text
src/terrascout_navigation/config/ground_seg.yaml
src/terrascout_navigation/config/nav2.yaml
src/terrascout_navigation/config/navigate_to_pose.xml
patches/ground_segmentation_ros2/tf-failure-stops-node.patch
```

### Robot-relative obstacle heights

The segmentation node transforms LiDAR points into `base_footprint` and publishes the obstacle cloud in that frame. This frame is the ground-level robot origin; `base_link` is elevated 0.33 m above it. The obstacle filter keeps non-ground Z values from **0.0 m to +1.5 m** relative to `base_footprint`. The segmentation input crop remains wider (`minZ: -1.5 m`) so it does not discard valid returns first. The raw cloud is retained as the separate costmap clearing/raytracing source. Points classified as ground are removed before this height filter and cannot be recovered by lowering its minimum.

Nav2's obstacle-layer min/max height checks happen after observations are transformed into each costmap's global frame (`map` or `odom`). Those source limits are intentionally wide; do not use them to implement a robot-relative height band. Collision Monitor uses `base_footprint` as its base frame to match the segmentation cloud and applies the matching 0.0 m / +1.5 m filter.

Ground plane fitting uses the baseline `groundInlierThreshold` of 0.15 m; this tolerance can still classify near-plane root returns as ground. Validate `ground_points` and `obstacle_points` at a root and test low rocks/curbs with the `base_footprint` height reference before relying on this configuration.

### Costmap inflation and replanning

- Both inflation radii are 0.6 m, the smallest value previously accepted by the footprint validator (circumscribed footprint about 0.596 m).
- Global cost scaling is 3.0; local cost scaling is 5.0. The lower global value makes costs decay more gradually through the inflation band, while the local layer keeps the steeper 5.0 decay for tighter tracking.
- The behavior tree checks path validity at 1 Hz and computes a new path when the current path is invalid or the goal changes.
- Follow failure recovery attempts a 1 m reverse at 0.15 m/s with a 12 s allowance, clears both costmaps, attempts one 90-degree turn, then replans even if the spin times out. The fallback has a wait and a slow 0.25 m forward nudge; Collision Monitor can still block motion.
- The latest stuck run timed out on backup/spin and Collision Monitor blocked the nudge. It also reported controller rates as low as about 7 Hz against a 20 Hz target; extra retries alone are not a fix.

These are configuration choices to evaluate, not confirmed performance improvements. Validate clear, narrow, blocked, and root-obstacle routes with Collision Monitor enabled. Compare costmaps and obstacle-cloud data before changing several values at once.


## 7. Runtime Support and Environment Setup

These files are not keepout feature logic, but they must be valid for simulation/runtime verification:

### `src/fleet_config/fleet_config/model.py`

Defines machine/environment parameters such as ROS domain and Gazebo partitioning.

If these are wrong, simulation isolation breaks and keepout validation may occur in the wrong ROS/Gazebo context.

### `src/fleet_config/fleet_config/targets.py`

Defines runnable targets and composition used by `make use`, `make build`, and `make up`.

This selects which runtime graph is rendered and therefore whether keepout-enabled TerraScout stack is launched.

### `src/terrascout_description/config/gz_bridge.yaml`

Controls Gazebo↔ROS topic bridging for simulation sensors and robot interfaces.

Not keepout-specific, but incorrect bridge setup can block dependent runtime behavior and make system-level keepout verification misleading.

---

## 8. Recent Keepout and Navigation Tuning Changes

This section summarizes recent behavior changes made after the initial keepout integration.

### `src/terrascout_mission/terrascout_mission/keepout_server.py`

- continues publishing keepout data periodically, including empty keepout cloud when no zone is active
- publishes a dedicated Foxglove marker topic (`keepout_zone_marker`) for a persistent colored keepout overlay
- on keepout delete, requests immediate local and global Nav2 costmap clear services

Result:

- keepout deletion is reflected faster in Nav2 and in Foxglove costmap views
- keepout visualization is available as an explicit overlay topic, independent of costmap windowing

### `src/terrascout_navigation/config/nav2.yaml`

- keepout obstacle source enabled in both global and local obstacle layers
- keepout obstacle range increased for farther zones:
    - global `keepout_mark.obstacle_max_range: 1000.0`
    - local `keepout_mark.obstacle_max_range: 200.0`
- obstacle layer height filters are currently:
    - `max_obstacle_height: 2.5`
    - `min_obstacle_height: -0.3`

Result:

- keepout zones are consumed at longer distances than before, but costmap display is still bounded by rolling-window behavior

### `src/terrascout_navigation/config/navigate_to_pose.xml`

- uses `PipelineSequence` with `RateController hz="0.2"`
- replanning path check is done through `IsPathValid` and `GlobalUpdatedGoal`
- keeps standard local/global costmap clearing recovery actions

Result:

- replanning is conservative (low frequency) and depends on path validity/goal updates, which reduces churn but can delay reaction to fast-changing blockage

### `src/fleet_config/fleet_config/classes/terrascout.py`

- keepout server launch params now include:
    - `marker_topic: keepout_zone_marker`
    - `publish_period_s: 0.2`

Result:

- keepout geometry and marker updates are republished faster for Foxglove and late subscribers

### `src/terrascout_mission/config/simulation/foxglove_bridge.yaml`
### `src/terrascout_mission/config/physical/foxglove_bridge.yaml`

- both simulation and physical Foxglove bridges whitelist `/<robot_namespace>/keepout_zone_marker`

Result:

- Foxglove can render a dedicated colored keepout overlay layer directly from marker messages

---

## 9. Mission and Foxglove Waypoint Changes

### Mission server Home behavior and crash fix

#### `src/terrascout_mission/terrascout_mission/mission_server.py`

- Renamed the navigation `GetParameters` service client field from `_parameters` to `_navigation_parameters_client`. `_parameters` is reserved internally by the ROS 2 `Node`; overwriting it caused `get_parameter("saved_home")` to crash the server when `--home` was used.
- Captures the first valid GPS fix as the dynamic spawn Home fallback. An explicitly configured `saved_home` from `--sethome` takes priority; otherwise the first valid startup position is returned by `get_home_point`.
- Hosts the Foxglove click collector in-process so it uses the existing mission server executable and cannot cause startup failure because of separate executable discovery.

#### `src/fleet_common/fleet_common/send_mission.py`

- `--home` asks the mission server for its saved Home and sends a home-only mission.
- `--sethome LAT,LON` updates the mission server's `saved_home` parameter without sending a mission.
- Invalid Home response text is handled as a clean client error rather than an uncaught coordinate parsing exception.

### Foxglove click-to-mission workflow

Files involved:

- `src/terrascout_mission/terrascout_mission/foxglove_mission_input.py`
- `src/terrascout_mission/terrascout_mission/mission_server.py`
- `src/fleet_common/fleet_common/geodesy.py`
- `src/terrascout_mission/config/simulation/foxglove_bridge.yaml`
- `src/terrascout_mission/config/physical/foxglove_bridge.yaml`
- `src/terrascout_mission/layouts/terrascout.json`
- `src/terrascout_mission/package.xml`
- `README.md`

The collector subscribes to `/<robot_namespace>/clicked_point` (`geometry_msgs/PointStamped`) from the Foxglove 3D Scene panel. It only accepts clicks in the configured `map` frame and converts map-frame ENU coordinates to GPS using the GNSS datum. Each accepted click is retained as a draft waypoint and visualized as a numbered marker on `mission_waypoint_markers`.

The inverse geodesy helper returns coordinates in `(latitude, longitude, altitude)` order. `foxglove_mission_input.py` must preserve that order when storing click coordinates; swapping latitude and longitude creates invalid points thousands of kilometres away. The mission server validates leg lengths and will reject such a mission.

Foxglove Trigger services operate on the draft:

- `/<robot_namespace>/mission_waypoints/clear` clears the draft.
- `/<robot_namespace>/mission_waypoints/send_ordered` submits all points in click order.
- `/<robot_namespace>/mission_waypoints/send_unordered` submits all points after nearest-next ordering.

Both send services submit one `ExecuteMission` goal with `TAKEOFF -> WAYPOINT(s) -> HOME -> LAND`; the return Home in that mission is the robot's GPS position when submitted. Draft points remain visible if the mission server rejects the goal. The existing `send_mission.py` plan and waypoint-ordering helpers are reused. The 2D geographic map panel is display-only; waypoint clicks are made in the 3D Scene with its fixed frame set to `map`.

The Trigger service response is asynchronous with respect to the action: `success: true` means the goal was queued for `ExecuteMission`, not that the mission server accepted it or that navigation started. Check `mission_server` logs and `/terrascout1/mission_state` for acceptance/progress. A rejected goal is reported in the mission-server log; its click draft remains available to retry or clear.

### Build/startup validation

- `make build` passes all workspace tests (139 tests, 0 failures).
- The Foxglove collector is attached to `mission_server` rather than launched as a separate executable. This avoids ROS executable lookup failure under `--symlink-install` when a newly added Python source file lacks executable permissions.
- After rebuilding, `make up` was verified with all four simulation containers remaining up and `mission_server` reporting `ready for missions`.
- During a live Foxglove test, the mission server rejected a draft because the earlier click-conversion code had latitude and longitude reversed; the log showed a first leg of about 4,939,886 m against the 45 m limit. The current collector unpacks `enu_to_llh()` as `lat, lon, _` and therefore fixes that specific rejection. Rebuild and restart after source edits before testing.

### Other mission tester and launch changes

- `src/terrascout_mission_tester/terrascout_menu.py` sends all configured JSON waypoints for Ordered/Unordered missions instead of asking the operator to select a subset. Single-waypoint selection remains for adding a waypoint or setting saved Home. The menu no longer includes the old return-to-start option 6 or y/N confirmations.
- `Makefile` adds `make exec COMMAND=...` to run a command in the active unit container with the ROS workspace overlay sourced.
- `terrascout_menu.py` checks whether host ROS can resolve `fleet_common` and falls back to the unit-container execution path when needed. The stack must be running for that fallback.

---

## 10. Nav2 Progress and Recovery Tuning

The 2026-10-13 simulation log showed repeated `controller_server: Failed to make progress`, intermittent missed 20 Hz control-loop deadlines, optimizer failures, and eventual navigation failure at mission command 3. A single failed mission waypoint must not be silently skipped: it may be an obstacle, a bad map/route, or a real safety issue. The mission should either reach that waypoint, fail with its identity, or allow an operator to choose an alternate point.

### `src/terrascout_navigation/config/nav2.yaml`

- Relaxed `SimpleProgressChecker` from a 5 s / 1.0 m requirement to 8 s / 0.25 m, with 0.25 rad angular progress. This detects a stalled robot sooner while allowing cautious movement.
- Enabled bounded reverse velocity in MPPI (`vx_min: -0.25`) and the velocity smoother (`min_velocity[0]: -0.25`). The existing `PreferForwardCritic` still favors forward motion; reverse is available when useful rather than being forced.

### `src/terrascout_navigation/config/navigate_to_pose.xml`

- After `FollowPath` fails, the first contextual recovery action backs straight up 1 m (0.15 m/s, 12 s allowance), then clears both costmaps and computes a fresh path to the same goal.
- The collision monitor remains active during reverse. If it blocks the maneuver or the backup action times out, Nav2 proceeds to its bounded recovery cycle; it does not force motion through an obstacle.

### `src/terrascout_mission/terrascout_mission/mission_server.py`

- Nav2 remains responsible for geometric obstacle avoidance and its behavior-tree recovery cycle. If Nav2 ultimately aborts a `WAYPOINT`, the mission server waits 2 seconds and retries the exact same target once by default (`waypoint_retry_limit: 1`, `waypoint_retry_delay_s: 2.0`). Both parameters are configurable at startup; retries are bounded to 0–3 and 0–30 seconds.
- A successful retry finishes the original command normally. If both attempts are aborted and a later positional command exists, the mission marks the failed command, returns to the last reached point, then continues to the next planned positional command. It does not invent a substitute coordinate. If backtracking fails, or there is no later target, the mission ends as failed.
- `continue_after_waypoint_failure` enables that bounded fallback. It runs only after Nav2 reports `STATUS_ABORTED` and the same-target retry is exhausted; cancellation does not trigger it. The current recovery uses a single contextual spin; the outer fallback uses backup, costmap clearing, wait, and a slow forward nudge without repeated spins.
- `map_yaw_offset_deg` is a single fleet setting shared by simulation `map→odom` TF and mission/Foxglove GPS conversions. It defaults to 0 (the world is ENU with zero heading); only change it after measuring +X/+Y against known east/north motion. Do not compensate by rotating only the Foxglove camera.
- The global costmap explicitly uses `map` and `base_link`; the rolling local costmap uses `odom` and `base_link`.

### Validation required

- The policy/configuration changes require a fresh build and simulation run before relying on them operationally.
- Verify parameter/XML loading; test an unobstructed route, temporary obstacle, 1 m backup, same-target retry, return to last reached point, continuation, and a final blocked waypoint that ends as mission failure.
- Monitor actual command velocities, progress-checker resets, planner/controller failures, and controller loop-rate warnings. The loop-rate warnings may indicate simulation/CPU scheduling load; do not further loosen stall detection or increase MPPI workload without measurement.