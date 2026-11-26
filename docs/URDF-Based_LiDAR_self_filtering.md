# URDF-Based LiDAR Self-Filtering

## 1. Purpose

Replace hardcoded robot self-filtering coordinates with geometry automatically extracted from the URDF.

This ensures the LiDAR filter adapts when robot dimensions or sensor positions change.

## 2. Modified Files

### `src/fleet_config/fleet_config/classes/terrascout.py`

- Added `robot_description=robot_description(unit)` to the `lidar_filter()` configuration.
- Passes the expanded URDF to the LiDAR filter as a ROS 2 parameter.
- **Reason:** Make robot geometry available to the filter.

### `src/terrascout_navigation/terrascout_navigation/lidar_filter.py`

- Added `_geometry_from_urdf()` to extract:
  - Chassis collision-box dimensions and position.
  - Wheel collision-cylinder dimensions and positions.
  - LiDAR position relative to `base_link`.
- Calculates chassis and wheel filtering regions in `lidar_link` coordinates.
- Replaced hardcoded exclusion regions with URDF-derived geometry.
- Retained configurable safety margins based on the original experimental filter.
- Geometry is calculated once at node startup, not for every LiDAR scan.
- **Reason:** Avoid duplicating robot geometry in Python.

## 3. Safety Margins

The filtering regions are calculated as:

**Filtering region = URDF collision geometry + safety margins**

Existing margins are retained to account for the robot's visual mesh and measurement uncertainty.

| Parameter | Margin |
|---|---:|
| `CHASSIS_X_MARGIN` | 0.20 m |
| `CHASSIS_Y_MARGIN` | 0.105 m |
| `CHASSIS_Z_LOWER_MARGIN` | 0.155 m |
| `CHASSIS_Z_UPPER_MARGIN` | 0.295 m |
| `WHEEL_RADIUS_MARGIN` | 0.025 m |
| `WHEEL_WIDTH_MARGIN` | 0.0175 m |

## 4. Unchanged Functionality

- Minimum LiDAR range filtering (0.1 m).
- Removal of invalid points (NaN, infinity, zero).
- Livox noise-tag filtering.
- NumPy-based point filtering.
- Output topic: `lidar_points_filtered`.
- FAST-LIO2 continues to receive `lidar_points_raw`.

## 5. Limitations and Verification

- Supports the current TerraScout URDF structure (chassis box and four wheel cylinders).
- Assumes the current axis alignment of the collision geometry.
- Requires the `robot_description` parameter at startup.
