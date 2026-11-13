# FAST-LIO2 Integration – Changes

## 1. Purpose

Improve TerraScout localization using FAST-LIO2 (LiDAR + IMU) and remove hardcoded sensor offsets by obtaining transformations from the robot's URDF.

## 2. Added Files

### `src/fleet_unit/fleet_unit/urdf_extrinsics.py`
- Calculates the transformation between two sensor frames using the URDF.
- Extracts translation (`extrinsic_T`) and rotation (`extrinsic_R`).
- Supports fixed-joint transformations.
- **Reason:** Avoid hardcoded LiDAR–IMU sensor offset positions in FAST-LIO2.

## 3. Modified Files

### `src/fleet_unit/fleet_unit/manifest.py`
- Added the `UrdfExtrinsics` dataclass.
- Added the optional `urdf_extrinsics` field to `Process`.
- **Reason:** Allow processes to request sensor transformations from the URDF.

### `src/fleet_config/fleet_config/common.py`
- Extended the `node()` function to accept `urdf_extrinsics`.
- **Reason:** Pass URDF transformation requirements into the process configuration.

### `src/fleet_unit/fleet_unit/commands.py`
- Added automatic URDF/Xacro processing during node startup.
- Calculates LiDAR–IMU extrinsics using `urdf_extrinsics.py`.
- Passes the calculated values to FAST-LIO2 as ROS 2 parameter overrides:
  - `mapping.extrinsic_T`
  - `mapping.extrinsic_R`
- **Reason:** Ensure FAST-LIO2 uses sensor offsets defined in the robot model.

### `src/fleet_config/fleet_config/classes/terrascout.py - in def fast_lio_simulation()`
- Configured FAST-LIO2 to calculate extrinsics between:
  - Target: `microstrain_imu_link`
  - Source: `lidar_link`
- Uses the same robot description as `robot_state_publisher`.
- **Reason:** Keep FAST-LIO2 sensor calibration consistent with the URDF.

### `src/terrascout_navigation/terrascout_navigation/fast_lio_odom.py`
- Replaced hardcoded IMU-to-robot offsets with TF lookups.
- Obtains transformations between:
  - `microstrain_imu_link` and `base_link`
  - `microstrain_imu_link` and `vrtk_link`
- Applies translation and rotation offsets to FAST-LIO2 odometry.
- Publishes:
  - `/odom` for Nav2.
  - `odom → vrtk_link` TF.
- **Reason:** Keep robot localization consistent with the URDF and avoid manually maintained offsets.

## 4. FAST-LIO2 Configuration

### `src/terrascout_navigation/config/fast_lio_sim.yaml`

Existing parameters:

```yaml
extrinsic_T: [0.10, 0.0, 0.32]
extrinsic_R: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
```

- These values remain as defaults.
- When launched through the updated TerraScout configuration, URDF-derived parameters override them.
- Standalone FAST-LIO2 launches without these overrides use the YAML values.

## 5. Verification

- **Verified:** Xacro expansion and URDF transformation calculation.
- **Calculated LiDAR–IMU translation:** `[0.10, 0.0, 0.32]` m.
- **Calculated rotation:** Identity matrix.
- **Remaining:** Verify FAST-LIO2 receives the overrides during simulation and validate the TF-based odometry output.