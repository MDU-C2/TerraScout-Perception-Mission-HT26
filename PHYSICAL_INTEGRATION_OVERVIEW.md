# TerraScout physical integration — engineering overview

**Status:** prepared in source; not validated on the physical robot<br>
**Documentation snapshot:** 24 November 2026 (after Step 23)<br>
**Development branch:** `integration/full-system-nav-base`<br>
**Experimental target:** `terrascout-physical`<br>
**Related reference:** [GNSS alignment node and parameter reference](GNSS_ALIGNMENT_REFERENCE.md)

> **Source-of-truth note.** This document was assembled from the upstream branch, the actual patches produced during Steps 12–23, and the successful render/build/test results reported during development. The latest local files were **not pushed**, and the assistant cannot inspect the developer's current working tree directly. Treat the descriptions of local-only additions as a documented design snapshot; review them against the checked-out source before committing. Do not treat a passing build as proof of hardware readiness.

## 1. Goal and non-goals

Prepare a **separate, hardware-oriented TerraScout deployment** that ultimately provides the functionality of the current distributed `sim-elevation-pc` arrangement, but uses actual LiDAR, IMU, Fixposition GNSS, motor controller and camera rather than Gazebo.

- **Robot / NUC:** physical sensor interfaces, FAST-LIO2, GNSS alignment, ROS 2 TF, Nav2, control and network export.
- **Jetson:** elevation mapping / GPU perception and the results needed by Nav2; the final physical NUC–Jetson deployment and network paths still require implementation/verification.
- **No robot access yet:** prepare the code and configuration, pass automated tests, preserve safe defaults, and record an explicit physical validation plan.
- **Existing targets must remain unchanged:** `terrascout1`, `sim`, `sim-record`, `sim-madum`, `sim-elevation-pc`, and `station` must not gain experimental localization nodes or changed live TF owners.

**Out of scope of the completed steps:** establishing reliable on-robot localization, proving sensor extrinsics/timestamps, enabling the live handover, validating real driving or confirming distributed Jetson operation. Those are future activities.

## 2. Targets and current isolation boundary

- **`terrascout1`**
  - **Role:** Existing physical deployment
  - **Experimental GNSS + FAST-LIO2 localization:** **Off**; keep its original Fixposition ownership
- **`sim`**
  - **Role:** Existing Gazebo simulation
  - **Experimental GNSS + FAST-LIO2 localization:** **Off**; preserve the existing sim FAST-LIO/adapter paths
- **`sim-record`**
  - **Role:** Simulation with recording
  - **Experimental GNSS + FAST-LIO2 localization:** **Off**
- **`sim-madum`**
  - **Role:** Simulation with recording/MADUM
  - **Experimental GNSS + FAST-LIO2 localization:** **Off**
- **`sim-elevation-pc`**
  - **Role:** Existing PC–Jetson distributed simulation
  - **Experimental GNSS + FAST-LIO2 localization:** **Off**; remain the reference architecture
- **`station`**
  - **Role:** Existing station deployment
  - **Experimental GNSS + FAST-LIO2 localization:** **Off**
- **`terrascout-physical`**
  - **Role:** New local-only experimental physical target
  - **Experimental GNSS + FAST-LIO2 localization:** FAST-LIO + alignment **present in diagnostic mode**; live handover **off**

`src/fleet_config/fleet_config/targets.py` defines target names. `src/fleet_config/fleet_config/model.py` defines the `Features` dataclass. `src/fleet_config/fleet_config/classes/terrascout.py` renders the robot's processes and topic remappings. `src/fleet_config/test/test_render_facts.py` checks publisher ownership and experimental-target boundaries.

**Checks already reported successful:** rendering each of the six original targets, searching their rendered manifests for experimental process/topic names (no matches), and `make build TARGET=terrascout-physical` after Step 23. These checks do **not** prove behavioral equivalence of shared C++ code at runtime.

## 3. System architecture and TF ownership

### 3.1 Current prepared configuration — diagnostic mode

```text
Livox MID-360 + MicroStrain IMU ──> FAST-LIO2 ──> fast_lio/odometry
                                         │               │
                                         │               ├──> GNSS alignment shadow
Fixposition GNSS/status ─────────────────┘               │       │
                                                                 ├── map→odom candidate
                                                                 ├── smoothed alignment
                                                                 └── timestamped TF preview

Fixposition driver (nav2_mode: true) ── live TF ──> map → odom → vrtk_link
Robot-state-publisher (URDF) ───────────────────────────────> static robot links
Differential-drive controller ── live `odom` ROS topic (TF broadcast disabled)
FAST-LIO odom adapter ──────── private odometry and TF preview topics
Nav2 ──────────────────────── consumes the existing live topics/TF tree
```

**Critical distinction:** a `nav_msgs/Odometry` topic called `odom` is not the same as a TF edge `odom → vrtk_link`. The drive controller currently supplies an `odom` topic; Fixposition currently owns the live localization TF. The `fast_lio_odom.py` adapter is independently producing **private** preview messages on the experimental target.

### 3.2 Prepared but NOT activated coordinated handover

```text
Fixposition GNSS measurements/status ─────┐
FAST-LIO2 odometry ───────────────────────┼──> GNSS alignment ── map → odom (live TF)
                                         │
                                         └──> FAST-LIO adapter ── odom → vrtk_link (live TF)
                                                │
                                                └── /odom (Nav2 odometry)
Robot-state-publisher ── unchanged robot-link TF
Drive controller ──────── /drive/odom (not /odom)
Fixposition TF/TF_STATIC ─ isolated private topics (measurement topics retained)
```

The *single* coordinated feature flag selects all these changes in the rendered experimental physical target. It is **false by default**, and no on-robot handover has occurred. There are important open questions around the robot's TF tree, startup before GNSS initialization, freshness handling, and safety responses; see §10.

### 3.3 Frames and transform calculation

- FAST-LIO2 odometry input is expected to report `header.frame_id = camera_init` and `child_frame_id = body` (the latter represents the IMU/body pose from this FAST-LIO build).
- Fixposition ENU odometry is expected to report `map → vrtk_link` in `nav2_mode`.
- URDF provides the fixed transform between `microstrain_imu_link` and `vrtk_link`, as well as robot links below `vrtk_link` (including `base_footprint` and `base_link`).
- For each matched timestamp, the alignment node computes

  ```text
  T_odom_vrtk = T_odom_imu_from_FAST_LIO × T_imu_vrtk_from_URDF
  T_map_odom  = T_map_vrtk_from_Fixposition × inverse(T_odom_vrtk)
  ```

- `fast_lio_odom.py` converts FAST-LIO2's IMU-centric pose into odometry for `base_link` and TF `odom → vrtk_link`, using the URDF extrinsics. In the experimental target those outputs remain preview-only until the handover.

**TF review requirement:** Fixposition can also publish `FP_*` frame transforms and the static `FP_ENU0 → map` transform. Isolating **all** Fixposition `/tf` and `/tf_static` output must not silently remove a static transform another component requires. Validate the complete TF graph before enabling the handover.

## 4. Feature flags: location, effect and safety

The following are **Python `Features` flags**, not ROS parameters and not `site.yaml` keys. They are declared in `src/fleet_config/fleet_config/model.py` and selected per target in `src/fleet_config/fleet_config/targets.py`. The robot's process builder accesses them through `unit.resolved_robot().features` in `terrascout.py`.

- **`recording_enabled`**
  - **Default:** `False`
  - **What it does:** Existing recording feature
  - **Operational rule:** Existing feature; do not repurpose
- **`madum_enabled`**
  - **Default:** `False`
  - **What it does:** Existing MADUM integration feature
  - **Operational rule:** Existing feature; do not repurpose
- **`isolate_drive_odometry`**
  - **Default:** `False`
  - **What it does:** Routes `diff_drive_controller/odom` from `odom` to `drive/odom`
  - **Operational rule:** Partial isolation alone is rejected by Step 23 validation
- **`isolate_fixposition_tf`**
  - **Default:** `False`
  - **What it does:** Redirects Fixposition `/tf` and `/tf_static` to private topics
  - **Operational rule:** Partial isolation alone is rejected by Step 23 validation
- **`experimental_localization_handover`**
  - **Default:** **`False`**
  - **What it does:** Atomically opts into the four localization/odometry ownership changes
  - **Operational rule:** **Keep false until real-robot validation**; physical TerraScout only

With `experimental_localization_handover=True`, the process builder *derives* the following effective settings together:

- **Fixposition `/tf`**
  - **OFF (current default):** Namespaced live `tf`
  - **ON (future, unvalidated):** `fixposition/tf_isolated`
- **Fixposition `/tf_static`**
  - **OFF (current default):** Namespaced live `tf_static`
  - **ON (future, unvalidated):** `fixposition/tf_static_isolated`
- **Alignment ROS parameter `publish_live_tf`**
  - **OFF (current default):** `False`
  - **ON (future, unvalidated):** `True`
- **FAST-LIO adapter process**
  - **OFF (current default):** `fast_lio_odom_preview`
  - **ON (future, unvalidated):** `fast_lio_odom_live`
- **FAST-LIO adapter `/tf`**
  - **OFF (current default):** `fast_lio/odom_tf_preview`
  - **ON (future, unvalidated):** `tf`
- **FAST-LIO adapter `odom`**
  - **OFF (current default):** `fast_lio/odom_preview`
  - **ON (future, unvalidated):** `odom`
- **Drive controller's `odom` mapping**
  - **OFF (current default):** `odom`
  - **ON (future, unvalidated):** `drive/odom`

The preview/live adapter selection and mapping live in `fast_lio_odom_preview(...)` in `terrascout.py` (despite that function's historical name). `fixposition_tf_remaps(...)`, `drive_controller_remaps(...)` and `validate_localization_handover(...)` coordinate the rest.

**Guardrails:** `validate_localization_handover(...)` rejects a handover in simulation and rejects standalone `isolate_fixposition_tf=True` or `isolate_drive_odometry=True` while `experimental_localization_handover=False`. Tests also render an **in-memory test copy** of the experimental target with the handover flag enabled to confirm one publisher per live localization TF edge; the actual repository target remains disabled.

> Do **not** enable a single component manually by editing a ROS parameter or remapping without the coordinated target flag. A syntactically valid configuration may still produce inconsistent TFs, missing odometry or unsafe control behavior on the robot.

## 5. `site.yaml` versus `Features` versus ROS node parameters

These are distinct configuration layers:

1. **`site.yaml`: hardware/site-specific values**, still required by physical deployments. Examples: the Fixposition `gnss_stream`, `drive_can_interface`, network `uplink_interface`, etc. The original physical and experimental physical targets can share robot site information. Avoid documenting actual addresses, credentials or serial details here.
2. **`Features` in `model.py` + target entries in `targets.py`: target-specific behavior**, including experimental localization ownership. Since `terrascout1` and `terrascout-physical` refer to the same robot, `site.yaml` must *not* control Fixposition TF isolation. The old `isolate_fixposition_tf` site setting was superseded by the target-scoped flag in Step 22.
3. **ROS parameters:** runtime node settings, such as `publish_live_tf`, the GNSS alignment thresholds, and `velocity_smoothing_alpha`. Defaults and legal ranges are declared with `fleet_common.params.declare` within nodes. Concrete launch values are supplied in the process `param_values` within `terrascout.py`. See the [full alignment parameter reference](GNSS_ALIGNMENT_REFERENCE.md).
4. **Device ROS parameter files:** e.g., `src/terrascout_hardware/config/gnss.yaml`, `src/terrascout_hardware/config/controllers.yaml`, and `src/terrascout_navigation/config/fast_lio_physical.yaml` (local experimental file). These are not the location of the per-target `Features` toggles.

**Fixposition:** keep `nav2_mode: true` in `gnss.yaml`. Setting it to false is **not** a safe shortcut for TF isolation: the pinned driver changes odometry frame IDs and also stops publishing its Nav2 datum topic. Our approach remaps its TF output instead, while preserving GNSS measurements and datum.

**Drive controller:** `enable_odom_tf: false` is configured in `controllers.yaml`; it still produces an odometry **topic**. `position_feedback: true` is configured, but this does **not** establish that wheel encoders are physically available or correctly integrated. Verify hardware feedback once the robot is available.

## 6. Changed or newly introduced files

- **`src/fleet_config/fleet_config/targets.py`** — Local experimental `terrascout-physical` target; existing targets preserved
- **`src/fleet_config/fleet_config/model.py`** — Target-scoped feature switches, notably coordinated handover
- **`src/fleet_config/fleet_config/classes/terrascout.py`** — Physical FAST-LIO processes; GNSS alignment; preview/live odometry switching; Fixposition and drive remaps; handover validation
- **`src/fleet_config/fleet_config/unit.py`** — Existing distributed simulation topic-boundary reference; physical boundary needs separate work
- **`src/fleet_config/test/test_render_facts.py`** — Live TF publisher uniqueness and target isolation tests; tests enabled handover on a temporary render
- **`src/terrascout_navigation/terrascout_navigation/gnss_alignment_shadow.py`** — GNSS–FAST-LIO alignment, quality gating, reacquisition, smoothing, preview, watchdog and gated live-TF option
- **`src/terrascout_navigation/test/test_gnss_alignment_shadow.py`** — Algorithm, freshness, reacquisition, smoothing and optional broadcast tests
- **`src/terrascout_navigation/terrascout_navigation/fast_lio_odom.py`** — Existing FAST-LIO2 odometry/TF adapter reused in physical preview mode
- **`src/terrascout_navigation/config/fast_lio_physical.yaml`** — Local FAST-LIO2 physical sensor configuration; inspect actual values in checkout
- **`src/terrascout_navigation/CMakeLists.txt` / `package.xml`** — Register GNSS alignment executable/tests and dependencies
- **`src/terrascout_hardware/config/gnss.yaml`** — Fixposition driver configuration (`nav2_mode: true`)
- **`src/terrascout_hardware/config/controllers.yaml`** — Drive controller odometry/TF settings (existing configuration)
- **`docker/Dockerfile`** — Added physical-image `ros-${ROS_DISTRO}-pcl-ros` needed by FAST-LIO2; simulation image already had it
- **`patches/FAST_LIO_ROS2/mid360_point_timing.patch`** — MID-360 per-point timestamps interpreted from FLOAT64 `timestamp` (nanoseconds) relative to cloud start, rather than estimated by scan angle
- **`patches/livox_ros_driver2/system-time-stamps.patch`** — Pinned LiDAR driver's system-time stamping integration; inspect patch and actual sensor clock behavior

Other FAST-LIO2 and driver patches were already present in the repository or were applied during the earlier simulation work. This table distinguishes the **new integration-related pieces** but is **not** an authoritative full `git diff` of the unpushed checkout.

### Why the timestamp patch matters

The `mid360_point_timing.patch` uses the expected Livox PointCloud2 `timestamp` field (FLOAT64 absolute nanoseconds), computes each point's offset from the cloud header in milliseconds, bounds the offset and rejects invalid points. FAST-LIO2 stores per-point relative timing in its point `curvature` field for deskew/processing. This code is shared by builds using the same FAST-LIO source; a successful render alone does not rule out a simulation regression.

## 7. Key ROS topics (names shown for `terrascout1` namespace)

- **`/terrascout1/lidar_points_raw`**
  - **Type:** `sensor_msgs/PointCloud2`
  - **Producer / use:** Livox physical LiDAR output
  - **Mode:** Physical
- **`/terrascout1/fast_lio/odometry`**
  - **Type:** `nav_msgs/Odometry`
  - **Producer / use:** FAST-LIO2 `camera_init → body` estimate
  - **Mode:** Both preview and future live
- **`/terrascout1/fixposition/odometry_enu`**
  - **Type:** `nav_msgs/Odometry`
  - **Producer / use:** Fixposition `map → vrtk_link` GNSS/fusion pose
  - **Mode:** Both
- **`/terrascout1/fixposition/fpa/odomstatus`**
  - **Type:** `fixposition_driver_msgs/FpaOdomstatus`
  - **Producer / use:** Fixposition GNSS/fusion status
  - **Mode:** Both
- **`/terrascout1/fixposition/datum`**
  - **Type:** `sensor_msgs/NavSatFix`
  - **Producer / use:** Datum needed by navigation/geospatial consumers
  - **Mode:** Preserve
- **`/terrascout1/fast_lio/gnss_alignment_candidate`**
  - **Type:** `geometry_msgs/TransformStamped`
  - **Producer / use:** Accepted unsmoothed `map → odom` candidate
  - **Mode:** Diagnostic
- **`/terrascout1/fast_lio/gnss_alignment_smoothed`**
  - **Type:** `geometry_msgs/TransformStamped`
  - **Producer / use:** Smoothed accepted correction
  - **Mode:** Diagnostic
- **`/terrascout1/fast_lio/gnss_alignment_tf_preview`**
  - **Type:** `geometry_msgs/TransformStamped`
  - **Producer / use:** 20 Hz timestamp-refreshed correction while FAST-LIO is fresh
  - **Mode:** Diagnostic
- **`/terrascout1/fast_lio/odom_preview`**
  - **Type:** `nav_msgs/Odometry`
  - **Producer / use:** FAST-LIO adapter's private Nav2 odometry preview
  - **Mode:** Diagnostic
- **`/terrascout1/fast_lio/odom_tf_preview`**
  - **Type:** `tf2_msgs/TFMessage`
  - **Producer / use:** FAST-LIO adapter's private `odom → vrtk_link` preview
  - **Mode:** Diagnostic
- **`/terrascout1/tf` and `/terrascout1/tf_static`**
  - **Type:** `tf2_msgs/TFMessage`
  - **Producer / use:** Live namespaced TF tree
  - **Mode:** Current Fixposition + URDF; ownership changes only in enabled handover
- **`/terrascout1/odom`**
  - **Type:** `nav_msgs/Odometry`
  - **Producer / use:** Live odometry topic
  - **Mode:** Current drive controller; future FAST-LIO adapter
- **`/terrascout1/drive/odom`**
  - **Type:** `nav_msgs/Odometry`
  - **Producer / use:** Drive-controller odometry after handover
  - **Mode:** Future only
- **`/terrascout1/fixposition/tf_isolated`**
  - **Type:** `tf2_msgs/TFMessage`
  - **Producer / use:** Fixposition dynamic TF when isolated
  - **Mode:** Future only
- **`/terrascout1/fixposition/tf_static_isolated`**
  - **Type:** `tf2_msgs/TFMessage`
  - **Producer / use:** Fixposition static TF when isolated
  - **Mode:** Future only

In the physical FAST-LIO configuration, check the exact MicroStrain IMU source, QoS and cloud remappings in `terrascout.py` and `fast_lio_physical.yaml` before hardware integration; those local-only settings have not been independently read from the final unpushed checkout. Namespaced `/tf` topics are the repository's existing convention, **not** necessarily the global ROS `/tf` topic.

## 8. Completed implementation and verification timeline

- **Earlier FAST-LIO integration**
  - **Main result:** FAST-LIO2 physical configuration, Livox point timing and sensor/IMU setup preparation
  - **Verification boundary:** Compilation/tests; hardware timing and extrinsics not checked
- **12**
  - **Main result:** Timestamp-matched FAST-LIO interpolation; no extrapolation
  - **Verification boundary:** Automated tests
- **13**
  - **Main result:** GNSS reacquisition after gaps and residual failures
  - **Verification boundary:** Automated tests
- **14**
  - **Main result:** Smoothed/rate-limited global alignment
  - **Verification boundary:** Automated tests
- **15**
  - **Main result:** Added GNSS alignment unit tests
  - **Verification boundary:** Physical Docker build required Fixposition message package
- **15.1/15.2**
  - **Main result:** Added `pcl_ros` to physical image; conformed node parameters to `fleet_common.params.declare`
  - **Verification boundary:** Physical build/test passed
- **16**
  - **Main result:** Continuous diagnostic TF preview (20 Hz)
  - **Verification boundary:** Build/test passed
- **17**
  - **Main result:** Monotonic arrival-time watchdog for advancing FAST-LIO messages
  - **Verification boundary:** Build/test passed
- **18**
  - **Main result:** FAST-LIO odometry/TF preview with isolated topic remaps
  - **Verification boundary:** Build/test passed after TF-owner test understood remapping
- **19**
  - **Main result:** Optional Fixposition dynamic/static TF isolation
  - **Verification boundary:** Prepared, disabled
- **20**
  - **Main result:** Optional live map→odom broadcaster behind `publish_live_tf=False`
  - **Verification boundary:** Prepared, disabled
- **21**
  - **Main result:** Target-scoped drive-odometry routing option
  - **Verification boundary:** Prepared, disabled
- **22**
  - **Main result:** Moved Fixposition TF isolation out of shared site values into `Features`
  - **Verification boundary:** Build/test passed
- **23**
  - **Main result:** One coordinated opt-in handover and regression tests
  - **Verification boundary:** Developer reports successful build; handover remains disabled

No physical sensor/robot driving test has yet taken place. The count of past tests varied as more tests were added; the current authoritative count is the one emitted by the developer's latest local `make build`.

## 9. Verification commands (do not activate live TF)

Run from the repository root. These commands are non-actuating build/render checks:

```bash
# Experimental physical render and complete physical build/test phase
make render TARGET=terrascout-physical
make build TARGET=terrascout-physical

# All six old targets must still render
for target in terrascout1 sim sim-record sim-madum sim-elevation-pc station; do
  make render TARGET="$target" || exit 1
done

# Should produce no matching lines in existing target renders
grep -R -nE \
  'gnss_alignment_shadow|fast_lio_physical|fast_lio/odom_preview|fast_lio/odom_tf_preview|fixposition/tf_isolated|fixposition/tf_static_isolated' \
  .sage/render/terrascout1.1 \
  .sage/render/sim.1 \
  .sage/render/sim-record.1 \
  .sage/render/sim-madum.1 \
  .sage/render/sim-elevation-pc.1 \
  .sage/render/station.1 || true
```

If switching from a simulation-built to a physical-built workspace, the Makefile tracks build mode and may refuse mixed build/install artifacts. **Do not erase a working simulation build without planning to preserve it.** The Docker build/test environment is the authoritative ROS environment; sourcing Docker-generated `install/setup.bash` on the Ubuntu host can produce broken install-path warnings.

Check intended ownership in the **rendered manifest**, not just source code, before any live run. Automated publisher counting is a useful safety test but cannot verify dynamic runtime discovery, TF timestamp validity or absence of a third-party unexpected broadcaster.

## 10. Outstanding physical validation and safety items

These are **open**; they have not been solved by the Step 23 build:

1. **Sensor clock domains and synchronization:** prove Livox cloud header/per-point timestamps, MicroStrain IMU, Fixposition odometry/status, and ROS system time are mutually consistent. The physical process waits for chrony synchronization (`chronyc waitsync`, configured offset gate), but deployment on NUC/Jetson and real sensor time must be checked.
2. **Extrinsics and frame conventions:** verify the actual MID-360 ↔ IMU rotation/translation, `microstrain_imu_link → vrtk_link`, `vrtk_link → base_link`, ENU orientation and handedness; confirm FAST-LIO output remains `camera_init/body`.
3. **GNSS availability and reacquisition:** test GNSS startup, outages, RTK float/fixed transitions, outlier clusters, heading jumps, drift and long interruptions with recorded bags before actuating Nav2.
4. **TF owners and static dependencies:** verify who publishes every live frame edge; inspect what is lost when Fixposition's **both** TF topics are isolated, especially any useful `FP_*` or datum-related static links.
5. **Odometry consumers:** ensure Nav2, safety monitoring and any secondary consumers use the intended `/odom`; determine whether physical drive feedback really exists. `position_feedback: true` in YAML does not prove encoder feedback.
6. **Watchdog and safety:** the 1 s watchdog suppresses the alignment preview/live map correction if FAST-LIO2 stops. It is **not** a verified robot stop or Nav2 safety interlock. Define a separate stale-localization behavior and test it.
7. **Startup and loss of GNSS:** the alignment node emits no initial `map → odom` until sufficiently good and consistent GNSS and FAST-LIO samples have arrived. Decide how Nav2 startup/lifecycle should wait and recover.
8. **NIC, ROS namespace and distributed transport:** establish physical NUC ↔ Jetson topology, topic export/import rules and DDS/Zenoh details, without depending on simulated `/clock`.
9. **Jetson GPU pipeline:** configure and validate elevation-mapping-cupy, filtered cloud, TF transforms, semantic perception, and return traversability outputs to Nav2 on the physical target.
10. **End-to-end regression:** preserve current `terrascout1`/`sim*` functionality, including runtime simulation with the shared FAST-LIO2 timing patch, not just render tests.

## 11. Relationship to `sim-elevation-pc`: what to implement next

The existing `sim-elevation-pc` topology is defined through `src/fleet_config/fleet_config/targets.py` and `src/fleet_config/fleet_config/unit.py`. Its distributed simulation boundary exports clock, filtered point clouds, selected camera data and robot TF/TF_STATIC, and imports elevation mapping / perception products. **This is a reference, not yet the actual physical deployment.**

- **Sensor origins**
  - **Existing `sim-elevation-pc`:** Gazebo simulation
  - **Desired `terrascout-physical`:** Physical Livox, IMU, GNSS, camera
- **Clock**
  - **Existing `sim-elevation-pc`:** Simulation `/clock` exported
  - **Desired `terrascout-physical`:** Physical synchronized system/sensor time; **no Gazebo `/clock` dependency**
- **Local localization**
  - **Existing `sim-elevation-pc`:** Existing simulation pipeline
  - **Desired `terrascout-physical`:** FAST-LIO2 + optional GNSS alignment handover
- **Point cloud to Jetson**
  - **Existing `sim-elevation-pc`:** Export filtered LiDAR
  - **Desired `terrascout-physical`:** Export physical filtered LiDAR with correct timestamps/frames/QoS
- **TF to Jetson**
  - **Existing `sim-elevation-pc`:** Export robot TF and TF_STATIC
  - **Desired `terrascout-physical`:** Export the **physical** namespaced TF tree, with one owner per edge
- **Camera to Jetson**
  - **Existing `sim-elevation-pc`:** Simulation camera image
  - **Desired `terrascout-physical`:** Hardware camera stream/topic bridging
- **Elevation result to Nav2**
  - **Existing `sim-elevation-pc`:** Import Jetson elevation map
  - **Desired `terrascout-physical`:** Same API/consumption semantics where compatible
- **Semantic result to Nav2**
  - **Existing `sim-elevation-pc`:** Import detector outputs
  - **Desired `terrascout-physical`:** Physical perception stream and message mapping

**Recommended next phase:** define the physical topic boundary and deployment processes for NUC and Jetson, compare them with the existing simulated boundary, and add render-level tests ensuring (a) no simulated `/clock` input, (b) correct cloud/TF export, (c) elevation-map import, and (d) unchanged old-target configurations. Do this before enabling live TF or commanding physical motion.

## 12. Change management and source review

Because the modifications are unpushed, capture the exact implementation before committing these documents:

```bash
git status --short
git diff --stat
git diff -- src/fleet_config/fleet_config/model.py \
  src/fleet_config/fleet_config/targets.py \
  src/fleet_config/fleet_config/classes/terrascout.py \
  src/fleet_config/test/test_render_facts.py \
  docker/Dockerfile
```

`git diff` omits **untracked** files unless added to the index; separately inspect the new alignment node, its tests, physical FAST-LIO YAML and patches. Keep historical patch files outside the tracked tree unless they are part of reproducible third-party patch application. Compare the actual local target entries and ROS parameter mappings against this reference, then commit the documentation alongside the tested implementation.

**Do not activate `experimental_localization_handover` as a documentation or build-verification step.** It is an unfinished physical-system integration option.
