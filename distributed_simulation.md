# Distributed Simulation — Progress Summary

**Project:** TerraScout / SAGE  
**Branch:** `feature/distributed-simulation`  
**Last updated:** 27 October 2026  
**Goal:** Run Gazebo on the Ubuntu PC and FAST-LIO2, ground segmentation, STVL, and Nav2 on the ASUS NUC, while preserving the original single-PC simulation.

> **Current status: Operational and tested.** Gazebo runs on the PC, perception and navigation run on the NUC, simulated sensor data travels from PC to NUC, and navigation commands travel back to Gazebo. Autonomous goals have succeeded, and initial NUC performance measurements show stable 20 Hz command publication. Longer-term robustness and mapping quality remain to be evaluated.

## 1. Purpose of the main SAGE configuration files

- **`targets.py`:** Defines available deployment targets (`sim`, `sim-pc`, `sim-nuc`) and selects robot, simulation, and network settings.
- **`model.py`:** Defines the configuration data structures used for fleets, robots, simulations, deployments, and network topology.
- **`unit.py`:** Creates unit contexts (robot, world, station) and defines each unit's allowed cross-router topic imports and exports.
- **`terrascout.py`:** Selects TerraScout's ROS 2 processes and assigns Gazebo-facing or navigation/perception processes to the relevant deployment.
- **`zenoh.py`:** Generates Zenoh router configuration, transport connections, and access-control policies.
- **`compose.py`:** Generates Docker Compose services, container settings, and startup dependencies. **Not modified** for this deployment.
- **`render.py`:** Combines configurations into generated manifests, router files, topic descriptions, and Compose files.

### Simplified configuration workflow

```text
targets.py + model.py
          |
          v
       unit.py --------> Unit contexts and topic boundaries
          |
          v
     terrascout.py -----> Process lists for each robot deployment
          |
          v
       render.py
          |----> zenoh.py   -> Zenoh routing / ACL configuration
          `----> compose.py -> Docker Compose configuration
          |
          v
 Generated .sage/render/<target>.1/ files
          |
          v
       make up
```

*This illustrates how the files contribute to rendering; it is not a literal Python call graph.*

## 2. Modified files

### `terrascout.py` — Process allocation

**Path:** `src/fleet_config/fleet_config/classes/terrascout.py`

- **Added `simulation_interface_processes()`** — Collects Gazebo-facing robot processes for the PC: world readiness, robot spawning, entity monitoring, simulation clock bridge, and ROS–Gazebo Bridge.
- **Added `simulation_navigation_processes()`** — Collects NUC-side perception, FAST-LIO2, odometry conversion, Nav2/STVL, mission services, teleoperation support, and Foxglove Bridge.
- **Updated `processes()`** — Selects the PC-only, NUC-only, or original combined process set according to `deployment`.

### `model.py` — Deployment data structures

**Path:** `src/fleet_config/fleet_config/model.py`

- **Added `deployment` to `Fleet` and `SimTarget`** — Distinguishes `combined`, `pc`, and `nuc` deployments.
- **Added `topology` to `SimTarget`** — Enables target-specific network settings.
- **Updated `SimTarget.fleet()`** — Propagates deployment and topology into the rendered fleet configuration.

### `targets.py` — Deployment targets

**Path:** `src/fleet_config/fleet_config/targets.py`

- **Added `sim-pc` and `sim-nuc`** — Introduces two-machine deployment targets while keeping the existing `sim` target.
- **Imported/configured `Topology`** — Sets the PC router address and host-specific network interfaces.

### `unit.py` — Unit contexts and Zenoh topic boundaries

**Path:** `src/fleet_config/fleet_config/unit.py`

- **Added `deployment` to `UnitContext`** — Makes the selected deployment available to unit generation.
- **Updated `contexts()`** — Includes the Gazebo world and station on the PC, but not on the NUC.
- **Updated `on_fleet_host` property** — Makes the NUC's robot router connect remotely to the PC fleet router instead of starting its own fleet router.
- **Added `distributed_simulation_boundary()`** — Defines allowed PC-to-NUC sensor/time topics and the NUC-to-PC velocity command; also retains robot status exports on the NUC.
- **Added `/terrascout1/odom_gt` to the sensor boundary** — Allows the NUC-side GNSS shim to consume Gazebo ground-truth odometry.

### `zenoh.py` — Central router address

**Path:** `src/fleet_config/fleet_config/zenoh.py`

- **Updated `fleet_router_address()`** — Exposes the PC's central fleet router on its LAN address for distributed simulation, while keeping the existing single-PC `sim` target on loopback.

### `render.py` — Headless NUC configuration

**Path:** `src/fleet_config/fleet_config/render.py`

- **Updated `fleet_summary()`** — Sets the GUI flag according to whether the generated deployment contains a Gazebo `world` unit. As a result, `sim-nuc` is headless while `sim` and `sim-pc` remain graphical.

### Files not modified for this stage

- `src/fleet_config/fleet_config/compose.py`
- `src/terrascout_navigation/config/nav2.yaml`
- FAST-LIO2 and STVL algorithm source code

## 3. Deployment architecture

### Original single-computer target: `sim`

```text
PC
├── Gazebo + ROS–Gazebo Bridge
├── FAST-LIO2 + ground segmentation
├── STVL costmaps + Nav2 (Smac + MPPI)
└── Foxglove Bridge
```

### Implemented distributed targets: `sim-pc` and `sim-nuc`

```text
PC — sim-pc                              NUC — sim-nuc
130.243.66.211                          10.132.134.193
+-----------------------------+          +----------------------------+
| Gazebo + robot spawning     |          | FAST-LIO2 + odom adapter   |
| ROS–Gazebo Bridge           | --data-> | Ground segmentation        |
| Simulation clock bridge     |          | STVL costmaps              |
| Fleet Zenoh router :7447    | <-cmd--- | Nav2: Smac + MPPI          |
| Station services            |          | Mission services           |
| Foxglove Desktop            |          | Foxglove Bridge            |
+-----------------------------+          +----------------------------+
                            Ethernet / Zenoh
```

The NUC's unit router connects to **`tcp/130.243.66.211:7447`**. There is **one central fleet router**, running on the PC; the NUC does not start another fleet router.

Foxglove Desktop runs on the PC and has successfully connected to the NUC's Foxglove Bridge. An SSH tunnel is one working access method.

## 4. Cross-host ROS 2 topic permissions

| PC → NUC: simulated inputs          | NUC → PC: commands / published robot status |
|                                     |                                             |
| `/clock`                            | `/terrascout1/cmd_vel`                      |
| `/terrascout1/lidar_points_raw`     | `/terrascout1/unit/description`             |
| `/terrascout1/microstrain/imu/data` | `/terrascout1/unit/health`                  |
| `/terrascout1/navsat`               | `/terrascout1/mission_state`                |
| `/terrascout1/joint_states`         | `/terrascout1/fixposition/odometry_llh`     |
| `/terrascout1/odom_gt`              |                                             |

**Evidence level:** The topics above are *configured* as Zenoh boundary permissions. Actual receipt was directly demonstrated on the NUC for `/clock`, LiDAR, and IMU; the unique `NUC_ZENOH_TEST` velocity command was directly observed on the PC. `/navsat`, `/joint_states`, and `/odom_gt` appeared in the NUC ROS graph, but their individual message contents were not separately checked in the recorded tests. Other exported robot-status topics were not individually verified across the host boundary.

## 5. Verification and integration results

### Configuration, synchronization, and startup

- [x] PC and NUC repositories synchronized on `feature/distributed-simulation`, with matching commit IDs.
- [x] SSH, IP routing, and TCP connectivity established between the machines.
- [x] `make render TARGET=sim`, `sim-pc`, and `sim-nuc` completed successfully.
- [x] Original `sim` target preserved; its original combined simulation was previously launched successfully after the initial refactor.
- [x] `sim-pc` generated only Gazebo-facing TerraScout processes.
- [x] `sim-nuc` generated only the TerraScout perception/navigation processes; no `world`, `unit-station`, or `fleet-router` Compose service.
- [x] The NUC unit router connected to the PC's fleet router at port `7447`.
- [x] GUI settings rendered as `true` for `sim` and `sim-pc`, and `false` for `sim-nuc`.
- [x] `make build TARGET=sim-nuc` completed without errors on the NUC.
- [x] `make up TARGET=sim-pc` and `make up TARGET=sim-nuc` launched the distributed system.

### ROS 2 data flow and navigation

- [x] NUC received `/clock` messages from the PC.
- [x] NUC received `/terrascout1/lidar_points_raw` messages from the PC.
- [x] NUC received `/terrascout1/microstrain/imu/data` messages from the PC.
- [x] NUC published FAST-LIO2 odometry (`/terrascout1/fast_lio/odometry`).
- [x] NUC published Nav2 odometry (`/terrascout1/odom`) and ground-segmented obstacle clouds (`/terrascout1/obstacle_points`).
- [x] Nav2 `controller_server` and `bt_navigator` reported `active [3]`.
- [x] The `NavigateToPose` action was available on the NUC.
- [x] The uniquely tagged zero-velocity command `NUC_ZENOH_TEST` was received on the PC over Zenoh.
- [x] TerraScout moved in Gazebo in response to navigation goals sent from the NUC.
- [x] Three recorded navigation attempts ended with `Goal succeeded`.
- [x] Foxglove Desktop connected successfully to the NUC's bridge and could send navigation goals that moved TerraScout in Gazebo.

**Interpretation:** The complete distributed sensor-processing, planning, control, and Gazebo-actuation path has been demonstrated. This is a functional integration result, not yet a full robustness or obstacle-avoidance validation.

## 6. Initial NUC performance baseline

**Hardware:** ASUS NUC, Intel Core i7-1360P, 16 logical CPU threads, approximately 31 GiB RAM.  
**Workload:** Distributed simulation with FAST-LIO2, ground segmentation, STVL/Nav2, and Foxglove Bridge running on the NUC.

| Measurement | Observed value | Interpretation |
|---|---:|---|
| TerraScout Docker container CPU | `383.92%` | About 3.84 logical CPUs' worth of usage (roughly 24% of 16 logical threads). |
| Host CPU idle (`top`) | `73.5%` | Substantial aggregate CPU headroom during the sample. |
| TerraScout container memory | `814.5 MiB / 30.9 GiB` | Low memory usage; approximately 2.57% of the reported limit. |
| Host memory available (`top`) | `29,737.8 MiB` | No observable host memory pressure in the sample. |
| Swap used | `0 MiB` | No swapping in the sample. |
| `/terrascout1/cmd_vel_nav` publication rate | Approximately `20.000 Hz` | Matches the configured 20 Hz controller rate during the sampled period. |
| Command interval (`ros2 topic hz`) | `47–53 ms` | Small interval variation around the nominal 50 ms period. |
| Command interval std. deviation | Approximately `0.72 ms` | Low publication jitter in the sampled period. |
| Recorded navigation outcomes | 3 × `Goal succeeded` | Successful completion of the tested navigation goals. |

The `ros2 topic hz` output covered more than 1,100 received command messages and repeatedly reported approximately 20 Hz. This demonstrates stable **command publication timing** during that sample; it does not, by itself, measure every MPPI computation or guarantee that no internal deadlines were missed.

### High-CPU processes observed with `top`

| Process (as displayed) | CPU % at sample | Function |
|---|---:|---|
| `fastlio_mapping` | `54.2%` | FAST-LIO2 localization |
| `ground_segmenta` | `35.5%` | Ground segmentation |
| `component_conta` | `23.6%` | Nav2 component container (includes MPPI and STVL) |
| `zenohd` | `19.9%` | ROS 2 / Zenoh communication |
| `foxglove_bridge` | `14.0%` | Visualization bridge |

Several `python3` processes also consumed CPU. Their specific node identities were not captured in the `top` excerpt, so the table is **not** a complete per-component CPU breakdown. `top` and Docker percentages are snapshots, not long-term averages.

### Observed warnings and limitations

- Nav2 logged some **controller-loop deadline misses**, including reported rates below 20 Hz, primarily near the beginnings of the supplied navigation log excerpts. Despite this, the longer command-frequency sample remained near 20 Hz and the tested goals succeeded.
- Nav2 occasionally logged **TF extrapolation into the future**, with differences of roughly 24–37 ms in the shown examples. Navigation recovered, but the underlying timing should be monitored in more demanding scenarios.
- Little difference in total CPU use was observed between stationary and moving states, plausibly because LiDAR, localization, and segmentation continue to run while stationary.
- The results establish **headroom for the current workload**, not maximum system capacity or worst-case real-time guarantees. Individual busy threads may still experience scheduling delays.

**Baseline conclusion:** The NUC ran the current distributed perception/navigation workload successfully without evidence of sustained aggregate CPU or memory saturation. No performance-parameter changes are justified by the recorded baseline alone.

## 7. Next steps

- [ ] Evaluate **STVL obstacle-map accuracy** and navigation in denser/changed obstacle environments.
- [ ] Measure end-to-end **sensor latency, network bandwidth, packet/message drops**, and responsiveness under sustained load.
- [ ] Repeat CPU and RAM profiling across longer runs; identify the high-CPU `python3` processes and inspect per-core bottlenecks.
- [ ] Track controller deadline misses and TF extrapolation frequency during difficult navigation tasks.
- [ ] Stress-test higher LiDAR loads, long missions, recovery behaviors, and restart scenarios.
- [ ] Document a repeatable startup/shutdown and troubleshooting procedure for both hosts.
- [ ] Later, add Jetson-based elevation mapping and connect traversability information to Nav2.

---

**Overall status:** **Distributed simulation implemented and functionally validated.** PC↔NUC ROS 2 communication, autonomous robot motion, Foxglove goal control, and a first performance baseline have all been demonstrated. Advanced performance characterization, robustness, mapping quality, and Jetson integration remain future work.
