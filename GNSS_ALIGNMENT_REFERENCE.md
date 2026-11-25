# GNSS alignment node — algorithm and configuration reference

**Component:** `gnss_alignment_shadow.py`<br>
**Status:** implemented and tested without physical sensors; diagnostic mode by default<br>
**Last documented:** 24 November 2026 (Steps 12–23)<br>
**Related reference:** [Physical integration overview](PHYSICAL_INTEGRATION_OVERVIEW.md)

> **Source note:** This reference was checked against the reconstructed node after Step 20 and subsequent Step 23 feature-routing patches. Its parameter declarations and algorithms are inspectable from the retained patch/source artifacts. Your implementation is unpushed; compare the actual local source and the rendered experimental target before treating this as a release specification.

## 1. Purpose — exactly what the node does

The node estimates the **global correction between the locally drifting FAST-LIO2 odometry frame and the Fixposition ENU map frame**. It does **not** replace FAST-LIO2, run a state estimator or fuse raw GNSS with raw IMU measurements. It uses the **two existing pose estimates**:

1. FAST-LIO2: local LiDAR–IMU odometry, expressed as `camera_init → body`.
2. Fixposition: global/fused ENU pose, expressed as `map → vrtk_link` when the pinned driver is in `nav2_mode`.

After matching timestamps and applying URDF extrinsics, it calculates a candidate:

```text
T_map_odom = T_map_vrtk(Fixposition) × inverse(T_odom_vrtk(FAST-LIO2))
```

It rejects low-quality or inconsistent candidates, requires multiple consistent samples for initial acquisition and re-acquisition, and applies smoothing and correction speed limits. It publishes diagnostic `TransformStamped` messages showing raw accepted, smoothed and regularly refreshed alignments. An **explicitly disabled-by-default** mode can additionally broadcast the correction to the live TF tree as part of the coordinated handover.

**Resulting intended division of responsibility:** FAST-LIO2 gives smooth local motion even if GNSS becomes unreliable; GNSS updates the *slow global alignment* when it is trusted. During GNSS outages the most recently accepted global alignment is held, while FAST-LIO2 continues locally.

## 2. Where to find and change things

- **Node implementation, thresholds, ROS parameters and internal buffers** — `src/terrascout_navigation/terrascout_navigation/gnss_alignment_shadow.py`
- **Concrete launched parameter values, ROS topics/remaps, dependencies** — `src/fleet_config/fleet_config/classes/terrascout.py`, `gnss_alignment_shadow(...)` process definition
- **Parameter declaration policy** — `src/fleet_common/fleet_common/params.py`, function `declare(...)`
- **Target-scoped handover flag** — `src/fleet_config/fleet_config/model.py`, `Features.experimental_localization_handover`
- **Actual target selection** — `src/fleet_config/fleet_config/targets.py`, `terrascout-physical` target entry
- **Algorithm unit tests** — `src/terrascout_navigation/test/test_gnss_alignment_shadow.py`
- **Rendered ownership and regression tests** — `src/fleet_config/test/test_render_facts.py`
- **Sensor config and related frame definitions** — `src/terrascout_navigation/config/fast_lio_physical.yaml` and TerraScout URDF
- **Fixposition driver mode** — `src/terrascout_hardware/config/gnss.yaml`, `nav2_mode: true`

Parameter **declaration** in the node states the name, type, default and legal range. The **launch process** in `terrascout.py` passes the effective values through `param_values`. The `fleet_config` test suite checks that parameter declarations and rendered values match. **Editing only the declared default may not change the running process if `param_values` overrides it.**

`fleet_common.params.declare(...)` also creates read-only ROS parameter descriptors with explanations and numeric bounds. These are configuration-time tuning parameters, not intended for live reconfiguration while driving.

**Do not configure `publish_live_tf` alone.** Step 23 derives it from the target-specific handover flag so it changes atomically with Fixposition isolation, FAST-LIO output remapping and drive-odometry remapping.

## 3. Input topics, expected frames and output topics

All examples below are relative to the robot's namespace (`/terrascout1/` in the current example deployment).

### Inputs

- **`fast_lio/odometry`**
  - **ROS type:** `nav_msgs/Odometry`
  - **Expected content:** `camera_init → body`; valid stamped FAST-LIO2 IMU/body pose
- **`fixposition/odometry_enu`**
  - **ROS type:** `nav_msgs/Odometry`
  - **Expected content:** `map → vrtk_link`; global ENU pose plus covariance
- **`fixposition/fpa/odomstatus`**
  - **ROS type:** `fixposition_driver_msgs/FpaOdomstatus`
  - **Expected content:** Fixposition initialization, RTK fix and sensor-fusion usage status
- **TF buffer (`microstrain_imu_link`, `vrtk_link`)**
  - **ROS type:** TF / static URDF
  - **Expected content:** Fixed transform from IMU link to TerraScout root link

Input subscriptions use the ROS sensor-data QoS profile. These topics and frame names are **assumptions enforced by code**; an unexpected frame ID means the corresponding message is ignored, not silently reinterpreted.

### Outputs

- **`fast_lio/gnss_alignment_candidate`**
  - **ROS type:** `geometry_msgs/TransformStamped`
  - **When produced:** An accepted raw candidate, after first acquisition or normal validation
- **`fast_lio/gnss_alignment_smoothed`**
  - **ROS type:** `geometry_msgs/TransformStamped`
  - **When produced:** An accepted correction after smoothing/rate limiting
- **`fast_lio/gnss_alignment_tf_preview`**
  - **ROS type:** `geometry_msgs/TransformStamped`
  - **When produced:** Timer at `alignment_tf_preview_rate_hz`, while an alignment exists and LIO watchdog is fresh
- **`tf` (namespaced live topic)**
  - **ROS type:** `tf2_msgs/TFMessage` via TF broadcaster
  - **When produced:** **Only if** `publish_live_tf=True`, and subject to the same preview/watchdog gate

The first three outputs are **ordinary diagnostics**, not inserts into the TF buffer. Publishing a `TransformStamped` to a diagnostic topic does not make it a ROS TF edge. The live output is opt-in and disabled in the tested physical target.

## 4. Alignment algorithm: data flow in order

### A. Ingest time-ordered FAST-LIO2 odometry (`_on_lio`)

- Require `camera_init/body` frame IDs and a positive timestamp.
- Reject a duplicate or out-of-order timestamp; keep the newest **100** increasing FAST-LIO2 samples.
- Record arrival time from `time.monotonic_ns()` **only for advancing valid samples**. This is used for the watchdog; it is not derived from sensor clock offset.
- After receiving another LIO sample, attempt to process queued Fixposition measurements that can now be bracketed.

### B. Cache the latest Fixposition status (`_on_status`)

Cache status only if its header timestamp is positive. A GNSS odometry sample will later be compared with this latest status using `max_status_dt_s`.

### C. Check GNSS quality (`_on_fixposition`)

Before a pose can become an alignment candidate:

1. Require `map/vrtk_link` frame IDs and a positive timestamp.
2. Require a recent status message: `abs(gnss_time - status_time) <= max_status_dt_s`.
3. Require Fixposition to be **globally initialized** (`INIT_STATUS_GLOBAL_INIT`).
4. Require at least **one** GNSS receiver to be `GNSS_STATUS_RTK_FLOAT` or `GNSS_STATUS_RTK_FIXED` **and** its fusion measurement status to be `MEAS_STATUS_USED`.
5. Require positive finite X/Y pose covariance diagonal entries (`covariance[0]` and `covariance[7]`). Their standard deviation is evaluated as `sqrt(max(variance_x, variance_y))` and must not exceed `max_horizontal_std_m`.
6. Reject out-of-order qualified Fixposition samples.

A GNSS pose passing these gates is **queued**, not necessarily used immediately. This is important because the next FAST-LIO2 sample may not yet have arrived.

**Interpretation:** The node accepts *qualified Fixposition global/fused poses*, not raw satellite ranges. GNSS data alone are insufficient; the quality/status and synchronization gates must also pass.

### D. Synchronize LIO pose to GNSS timestamp (`_drain_pending_fixposition`, `_interpolate_lio`)

The node retains up to **50** pending GNSS poses. For each:

- If its timestamp is earlier than the first retained LIO sample, discard it.
- If it is newer than the latest LIO sample, wait for a later sample.
- Once both LIO samples exist, interpolate position linearly and orientation with shortest-path quaternion SLERP.
- Reject the pair when either bracketing sample lies more than `max_pair_dt_s` from the GNSS stamp.
- **Never extrapolate** FAST-LIO2 odometry into a timestamp for which no valid bracket exists.

The resulting synchronized LIO pose corresponds to the GNSS measurement time, which reduces motion-induced alignment errors.

### E. Apply IMU ↔ root extrinsics and calculate `map → odom`

The node looks up the fixed transform:

```text
lookup_transform("microstrain_imu_link", "vrtk_link", Time())
```

It caches that transform on first successful lookup. At each matched timestamp:

```text
T_odom_root = T_odom_imu(LIO) × T_imu_root(URDF)
T_map_root  = T_map_vrtk(Fixposition)
T_map_odom = T_map_root × inverse(T_odom_root)
```

Translation and rotation are composed in full 3D with quaternion operations. The output frame IDs are `map` (header) and `odom` (child).

> **Terminology caveat:** The FAST-LIO2 incoming `camera_init` reference frame is treated as the local odometry coordinate system for this calculation. This alignment is a mathematical convention in this project, not a claim that FAST-LIO2 directly broadcasts a ROS frame named `odom`.

### F. Initial alignment and reacquisition

The node starts in `_reacquiring=True`. It does **not** accept the very first qualified GNSS pose as the initial global reference.

- Gather at least `reacquire_min_samples` consistent candidates spanning at least `reacquire_min_duration_s`.
- Compare candidate translations and yaw against the first candidate of the current verification sequence.
- If spread exceeds `reacquire_max_position_spread_m` or `reacquire_max_yaw_spread_deg`, clear that sequence and start over.
- Only after enough consistent samples are collected is the current candidate accepted and published.
- Return to reacquisition if the interval between qualified GNSS samples exceeds `reacquire_after_s`.
- Also return to reacquisition after sustained ordinary alignment outliers exceed both `reacquire_after_rejections` and `reacquire_min_rejection_duration_s`.

This allows the system to reject isolated GNSS jumps while later accepting a persistent, internally consistent shifted global reference after a substantial outage or drift. It does **not** guarantee the new reference is physically correct; that requires hardware measurements and validation.

### G. Ordinary residual rejection

After initial acquisition, compare the newly measured global robot pose against the pose predicted by the previously accepted correction and current LIO pose.

- Reject if horizontal position residual exceeds `max_position_residual_m`.
- Reject if wrapped yaw residual exceeds `max_yaw_residual_deg`.
- Count consecutive rejected candidates and their duration to decide whether reacquisition should begin.
- Qualified GNSS samples with unacceptable residuals do **not** immediately jump the global frame.

### H. Smooth the correction (`_smooth_alignment`)

Once an alignment is accepted, smoothing is applied **at the robot's current pose**, not by naively rotating a distant `odom` frame origin. For the previous and desired alignment, compute `map → root` at the same LIO `odom → root` pose, then interpolate and rate-limit position/orientation before recovering `map → odom`.

The translation and full quaternion orientation use first-order smoothing with

```text
alpha = 1 - exp(-dt / alignment_smoothing_tau_s)
```

and independent hard limits on robot translation correction speed and rotation correction speed. The effective smoothing timestep is capped by `alignment_max_update_dt_s` so a long GNSS outage does not allow one enormous correction. On *first* accepted alignment the candidate establishes the initial anchor without a prior correction to smooth from.

### I. Periodic preview and FAST-LIO watchdog (`_publish_tf_preview`)

- Before any accepted/smoothed alignment: **publish nothing**.
- While FAST-LIO2 continues delivering advancing odometry: republish the latest alignment at `alignment_tf_preview_rate_hz`, refreshing the message header stamp using the node's ROS clock.
- During a GNSS outage: **retain the last accepted correction** and keep publishing it while FAST-LIO2 remains fresh.
- If no valid advancing LIO message arrives for `fast_lio_watchdog_timeout_s`: **withhold** the preview and, if enabled, the live alignment TF broadcast.
- After advancing LIO resumes: preview/live broadcast resumes using the stored alignment; a new accepted GNSS update is not required for resuming the held correction.

This watchdog checks **LIO arrival liveness**, not correct localization, GNSS freshness or physical robot safety. It does not send a stop command to Nav2 or the drive controller.

## 5. Complete ROS parameter reference (19 parameters)

The values below are **initial development settings**, not tuned or validated physical thresholds. They were read from the node's `declare(...)` calls and corresponding development launch changes. Ranges are enforced by the parameter declaration helper; the rendered target's `param_values` are the effective launch values.

### 5.1 Timestamp pairing and GNSS quality

- **`max_pair_dt_s`**
  - **Type:** double
  - **Default / development value:** `0.15` s
  - **Allowed range:** `0.01–1.0`
  - **Purpose:** Max allowable gap **to each LIO interpolation bracket** from GNSS timestamp
- **`max_horizontal_std_m`**
  - **Type:** double
  - **Default / development value:** `5.0` m
  - **Allowed range:** `0.01–50.0`
  - **Purpose:** Max permitted X/Y GNSS position standard deviation (from covariance)
- **`max_status_dt_s`**
  - **Type:** double
  - **Default / development value:** `1.0` s
  - **Allowed range:** `0.01–10.0`
  - **Purpose:** Max time difference between GNSS pose and its latest status

### 5.2 Ordinary outlier rejection

- **`max_position_residual_m`**
  - **Type:** double
  - **Default / development value:** `1.0` m
  - **Allowed range:** `0.01–100.0`
  - **Purpose:** Max predicted-versus-measured horizontal robot-position discrepancy
- **`max_yaw_residual_deg`**
  - **Type:** double
  - **Default / development value:** `5.0` deg
  - **Allowed range:** `0.1–180.0`
  - **Purpose:** Max predicted-versus-measured yaw discrepancy (wrapped angle)

### 5.3 Acquisition and reacquisition

- **`reacquire_after_s`**
  - **Type:** double
  - **Default / development value:** `2.0` s
  - **Allowed range:** `0.1–120.0`
  - **Purpose:** Qualified-GNSS sample gap that forces reacquisition
- **`reacquire_min_samples`**
  - **Type:** integer
  - **Default / development value:** `5`
  - **Allowed range:** `2–200`
  - **Purpose:** Minimum consistent candidates before accepting an initial/reacquired global alignment
- **`reacquire_min_duration_s`**
  - **Type:** double
  - **Default / development value:** `0.5` s
  - **Allowed range:** `0.05–120.0`
  - **Purpose:** Minimum timespan covered by these candidates
- **`reacquire_after_rejections`**
  - **Type:** integer
  - **Default / development value:** `5`
  - **Allowed range:** `2–200`
  - **Purpose:** Count of consecutive ordinary alignment residual failures required before reacquisition
- **`reacquire_min_rejection_duration_s`**
  - **Type:** double
  - **Default / development value:** `1.0` s
  - **Allowed range:** `0.1–120.0`
  - **Purpose:** Duration of consecutive residual failures also required for reacquisition
- **`reacquire_max_position_spread_m`**
  - **Type:** double
  - **Default / development value:** `0.75` m
  - **Allowed range:** `0.01–50.0`
  - **Purpose:** Max horizontal translation spread within a verification sequence
- **`reacquire_max_yaw_spread_deg`**
  - **Type:** double
  - **Default / development value:** `5.0` deg
  - **Allowed range:** `0.1–180.0`
  - **Purpose:** Max yaw spread within a verification sequence

### 5.4 Smoothing and rate limiting

- **`alignment_smoothing_tau_s`**
  - **Type:** double
  - **Default / development value:** `2.0` s
  - **Allowed range:** `0.01–60.0`
  - **Purpose:** Time constant for global correction smoothing; larger means slower correction
- **`alignment_max_translation_speed_m_s`**
  - **Type:** double
  - **Default / development value:** `0.2` m/s
  - **Allowed range:** `0.001–5.0`
  - **Purpose:** Hard maximum rate of robot-position correction in map frame
- **`alignment_max_rotation_speed_deg_s`**
  - **Type:** double
  - **Default / development value:** `5.0` deg/s
  - **Allowed range:** `0.1–180.0`
  - **Purpose:** Hard maximum rate of global orientation correction
- **`alignment_max_update_dt_s`**
  - **Type:** double
  - **Default / development value:** `0.2` s
  - **Allowed range:** `0.01–5.0`
  - **Purpose:** Upper bound on smoothing step after long gaps

### 5.5 Preview, liveness and TF handover

- **`alignment_tf_preview_rate_hz`**
  - **Type:** double
  - **Default / development value:** `20.0` Hz
  - **Allowed range:** `1.0–100.0`
  - **Purpose:** Rate of regularly timestamp-refreshed map→odom preview
- **`fast_lio_watchdog_timeout_s`**
  - **Type:** double
  - **Default / development value:** `1.0` s
  - **Allowed range:** `0.1–10.0`
  - **Purpose:** Max monotonic elapsed time since an **advancing** LIO message
- **`publish_live_tf`**
  - **Type:** bool
  - **Default / development value:** **`False`**
  - **Allowed range:** `True/False`
  - **Purpose:** Whether to mirror the same watchdog-gated correction to live namespaced TF

**Count:** 3 quality + 2 residual + 7 reacquisition + 4 smoothing + 3 preview/TF = **19 parameters**.

### 5.6 Other related tuning (outside this node)

- **`velocity_smoothing_alpha`**
  - **Default / current known value:** `0.35` in physical preview launch
  - **Where / why:** `fast_lio_odom.py` adapter; smooths inferred odometry twist, not global alignment
- **`nav2_mode`**
  - **Default / current known value:** `true`
  - **Where / why:** `src/terrascout_hardware/config/gnss.yaml`; must remain true for correct Fixposition frames/datum
- **`enable_odom_tf`**
  - **Default / current known value:** `false`
  - **Where / why:** `src/terrascout_hardware/config/controllers.yaml`; drive controller does not broadcast its odom TF
- **`experimental_localization_handover`**
  - **Default / current known value:** `False`
  - **Where / why:** `Features` in model/target config; controls all live localization owner switches together

FAST-LIO2 has **its own** settings in `fast_lio_physical.yaml` (sensor inputs/extrinsics/filtering/mapper parameters). These are not parameters of `gnss_alignment_shadow.py`; inspect the unpushed YAML file to document their exact values.

## 6. Internal state variables (not ROS parameters)

The following names may look like “experimental variables” while reading the Python file, but they are **mutable implementation state** and should not be edited as user-facing tuning knobs.

- **`_fast_lio`**
  - **Initial value or capacity:** `deque(maxlen=100)`
  - **Meaning:** Ordered buffer of accepted LIO odometry samples
- **`_pending_fixposition`**
  - **Initial value or capacity:** `deque(maxlen=50)`
  - **Meaning:** Qualified GNSS poses waiting for an interpolation bracket
- **`_latest_status`**
  - **Initial value or capacity:** `None`
  - **Meaning:** Latest stamped Fixposition status
- **`_imu_to_root`**
  - **Initial value or capacity:** `None`
  - **Meaning:** Cached URDF IMU-to-`vrtk_link` transform
- **`_last_qualified_stamp_ns`**
  - **Initial value or capacity:** `None`
  - **Meaning:** Most recent GNSS pose that passed quality and timing checks and reached candidate evaluation
- **`_last_accepted_alignment`**
  - **Initial value or capacity:** `None`
  - **Meaning:** Last raw, accepted `map → odom` candidate (used for residual comparison)
- **`_last_accepted_stamp_ns`**
  - **Initial value or capacity:** `None`
  - **Meaning:** Timestamp of last accepted raw candidate
- **`_reacquiring`**
  - **Initial value or capacity:** `True`
  - **Meaning:** Initial/reacquisition consistency-verification state
- **`_reacquire_candidates`**
  - **Initial value or capacity:** `deque(maxlen=200)`
  - **Meaning:** Candidate transforms in current verification sequence
- **`_rejected_count`**
  - **Initial value or capacity:** `0`
  - **Meaning:** Consecutive normal residual failures
- **`_first_rejected_stamp_ns`**
  - **Initial value or capacity:** `None`
  - **Meaning:** First timestamp in current failure sequence
- **`_smoothed_alignment`**
  - **Initial value or capacity:** `None`
  - **Meaning:** Last smoothed map→odom correction, retained through GNSS outages
- **`_last_smoothed_stamp_ns`**
  - **Initial value or capacity:** `None`
  - **Meaning:** Timestamp of last accepted smoothing update
- **`_last_lio_arrival_monotonic_ns`**
  - **Initial value or capacity:** `None`
  - **Meaning:** Last valid, advancing LIO message **arrival** time
- **`_tf_preview_was_stale`**
  - **Initial value or capacity:** `False`
  - **Meaning:** Edge-triggered logging state of LIO watchdog
- **`_live_tf_broadcaster`**
  - **Initial value or capacity:** `None` by default
  - **Meaning:** `tf2_ros.TransformBroadcaster` only when `publish_live_tf` is enabled

These buffer limits are literal constants in the code, **not configurable ROS parameters**. Increasing them may change CPU/memory characteristics and time-history behavior; do so only with tests and a clear reason.

## 7. State-machine summary and failure responses

```text
     [START: no alignment]
               |
               v
  [GNSS status and covariance qualify?] -- no --> ignore
               |
              yes
               v
  [LIO interpolation bracket exists?] ---- no --> wait/drop as appropriate
               |
              yes
               v
  [Accumulate consistent candidate samples]
               |
        count and duration met
               v
         [TRACKING]
           /      \
          /        \ ordinary residual > gate
         /          \ repeated for sufficient duration
   qualified          v
   candidate      [REACQUIRING]  <--- GNSS qualified-gap > threshold
      |                |
      v                | after consistency checks
 [SMOOTH CORRECTION] <--+
      |
      v
 [STORE LAST ALIGNMENT] -- GNSS outage --> retain last correction
      |
      v
 [20 Hz PREVIEW / OPTIONAL LIVE TF]
      |
      +--- FAST-LIO watchdog expired --> suppress broadcasts until fresh LIO
```

- **GNSS poor fix / no global init**
  - **Code behavior:** Reject incoming GNSS pose
  - **What is **not** guaranteed:** No fallback global map anchoring before first accepted fix
- **GNSS covariance too high or invalid**
  - **Code behavior:** Reject incoming GNSS pose
  - **What is **not** guaranteed:** No sensor-level diagnosis of the cause
- **GNSS pose ahead of latest LIO**
  - **Code behavior:** Queue until a later LIO message arrives
  - **What is **not** guaranteed:** If later LIO never comes, no alignment update
- **LIO time moves backwards or repeats**
  - **Code behavior:** Ignore offending LIO message; watchdog deadline not refreshed
  - **What is **not** guaranteed:** Duplicate stream may still exist elsewhere
- **Single GNSS global-position jump**
  - **Code behavior:** Reject if beyond residual gates
  - **What is **not** guaranteed:** Does not prove accepted GNSS samples are physically correct
- **Sustained global disagreement**
  - **Code behavior:** Enter reacquisition verification
  - **What is **not** guaranteed:** No immediate zero-error snap to GNSS
- **GNSS outage after prior alignment**
  - **Code behavior:** Hold the stored correction, continue preview if LIO fresh
  - **What is **not** guaranteed:** Global drift increases without new GNSS corrections
- **FAST-LIO stalls for >1 s**
  - **Code behavior:** Stop publishing preview (and optional live alignment TF)
  - **What is **not** guaranteed:** **No direct Nav2 stop command**; TF cache/consumers need explicit safety validation
- **Initial startup without accepted GNSS**
  - **Code behavior:** No `map → odom` candidate or preview
  - **What is **not** guaranteed:** Nav2 may lack a usable map transform if live handover were enabled prematurely

## 8. Tests and verification

The existing test suite, `src/terrascout_navigation/test/test_gnss_alignment_shadow.py`, exercises: timestamp interpolation/no extrapolation; wide sample gaps; quaternion sign flips; GNSS quality gating; translation speed limits; smoothing at a distant odometry origin; queued GNSS samples; initial multi-sample acquisition; isolated GNSS jumps; bad fixes; prolonged disagreement; long GNSS outages; preview before/after alignment; LIO staleness/recovery; default-disabled live TF; and watchdog-gated opt-in forwarding.

These tests inject ROS messages/recorders and call the node's methods. **They are deterministic unit tests, not physical telemetry or an end-to-end ROS TF/network test.**

From the repository root:

```bash
make render TARGET=terrascout-physical
make build TARGET=terrascout-physical
```

The Makefile's physical Docker build/test environment includes the Fixposition message interfaces; using host Python with a simulation-only `install/` previously raised `ModuleNotFoundError: fixposition_driver_msgs` and should not be mistaken for an algorithm failure.

For future tuning, record a ROS bag containing synchronized FAST-LIO odometry, Fixposition pose/status, URDF/static TF, the three alignment diagnostic topics, and timing information. Compare GNSS rejection, smoothing lag, LIO drift and recovery before touching the handover switch.

## 9. Physical tuning and validation plan (future work)

**Do not enable the handover just because the code compiles.** Suggested validation sequence when the robot becomes available:

1. Confirm physical LiDAR and MicroStrain IMU connectivity, physical frames, QoS and timestamp origins; inspect per-point MID-360 time fields.
2. Verify FAST-LIO's expected `camera_init/body` pose behavior while stationary, under manual motion and on uneven ground.
3. Verify Fixposition `map/vrtk_link`, RTK status fields, covariance and datum publication at known locations, including GNSS outages.
4. Compare measured inter-sensor timestamp offsets against `max_pair_dt_s` and `max_status_dt_s`; tune based on distributions, not guesses.
5. Check the URDF transforms and physical offsets against actual antenna, IMU, LiDAR and body geometry.
6. Run the alignment **diagnostics only** while moving. Plot candidate versus smoothed translation/yaw over time; characterize actual residuals, delay and drift.
7. Inject or record GNSS dropout/outlier conditions and FAST-LIO stalls; verify the warning and watchdog behavior.
8. Inspect the entire live TF graph, especially Fixposition `FP_*` static frames, and verify all consumers still resolve transforms when the driver is isolated.
9. In a controlled non-actuating test, verify the rendered **opt-in** ownership configuration end to end, with one source each for `map→odom`, `odom→vrtk_link` and `/odom`.
10. Only after a safe independent stop/interlock, bounded motion tests and a rollback plan should live handover be considered.

### What has *not* been validated yet

Actual RTK status quality, sensor timestamp consistency, LiDAR–IMU extrinsics, global ENU frame convention, odometry continuity, wheel/drive feedback, NUC–Jetson message transport, Nav2 recovery on stale localization, loss of datum-relative static TF, and any physical motion safety behavior.

---

**Maintenance rule:** Whenever a parameter changes, update (a) its `declare(...)` default/range/description as needed, (b) the rendered `param_values`, (c) associated unit tests, and (d) this reference. Whenever any TF publisher changes, also review the TF-owner regression tests and the physical integration overview.
