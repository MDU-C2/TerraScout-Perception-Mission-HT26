#!/usr/bin/env python3
"""Observe candidate map->odom alignment; live TF is disabled by default.

Inputs:
  fast_lio/odometry           nav_msgs/Odometry, camera_init -> body (IMU)
  fixposition/odometry_enu    nav_msgs/Odometry, map -> vrtk_link
  TF (static URDF)             microstrain_imu_link -> vrtk_link
Outputs (diagnostic messages by default, no /tf broadcast):
  fast_lio/gnss_alignment_candidate  raw accepted map->odom candidate
  fast_lio/gnss_alignment_smoothed   rate-limited map->odom candidate
  fast_lio/gnss_alignment_tf_preview  periodic timestamp-refreshed diagnostic

GNSS quality gates, timing, outlier rejection, and correction smoothing
require hardware validation before switching TF ownership.
"""

import math
import time
from collections import deque

import rclpy
from fixposition_driver_msgs.msg import FpaOdomstatus
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener

from fleet_common.params import declare


def _time_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def _quat(rotation):
    q = (rotation.x, rotation.y, rotation.z, rotation.w)
    norm = math.sqrt(sum(x * x for x in q))
    if not math.isfinite(norm) or norm < 1e-8:
        raise ValueError("invalid orientation quaternion")
    return tuple(x / norm for x in q)


def _multiply(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def _rotate(q, v):
    x, y, z, w = q
    vx, vy, vz = v
    tx, ty, tz = (2 * (y * vz - z * vy),
                  2 * (z * vx - x * vz),
                  2 * (x * vy - y * vx))
    return (vx + w * tx + y * tz - z * ty,
            vy + w * ty + z * tx - x * tz,
            vz + w * tz + x * ty - y * tx)


def _xyz(vector):
    return (vector.x, vector.y, vector.z)

def _yaw(q):
    """Extract yaw (radians) from quaternion (x, y, z, w)."""
    x, y, z, w = q

    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )

def _slerp(q0, q1, fraction):
    """Shortest-path quaternion interpolation for normalized (x, y, z, w)."""
    dot = sum(a * b for a, b in zip(q0, q1))
    if dot < 0.0:
        q1 = tuple(-component for component in q1)
        dot = -dot

    dot = max(-1.0, min(1.0, dot))
    if dot > 0.9995:
        result = tuple(
            (1.0 - fraction) * a + fraction * b
            for a, b in zip(q0, q1)
        )
    else:
        angle = math.acos(dot)
        sin_angle = math.sin(angle)
        a_weight = math.sin((1.0 - fraction) * angle) / sin_angle
        b_weight = math.sin(fraction * angle) / sin_angle
        result = tuple(
            a_weight * a + b_weight * b
            for a, b in zip(q0, q1)
        )

    magnitude = math.sqrt(sum(v * v for v in result))
    if not math.isfinite(magnitude) or magnitude < 1e-8:
        raise ValueError("invalid interpolated quaternion")
    return tuple(v / magnitude for v in result)


def _interpolate_lio(messages, stamp_ns, max_pair_dt_s):
    """Interpolate FAST-LIO pose at a GNSS stamp; never extrapolate.

    Both bracketing samples must be within max_pair_dt_s of the GNSS stamp.
    The input is expected to be ordered by increasing timestamp.
    Returns (position, quaternion) or None when a valid bracket is absent.
    """
    max_dt_ns = int(max_pair_dt_s * 1e9)
    previous = None

    for sample in messages:
        sample_ns = _time_ns(sample.header.stamp)
        if sample_ns == stamp_ns:
            return (
                _xyz(sample.pose.pose.position),
                _quat(sample.pose.pose.orientation),
            )
        if sample_ns > stamp_ns:
            if previous is None:
                return None
            previous_ns = _time_ns(previous.header.stamp)
            if (stamp_ns - previous_ns > max_dt_ns
                    or sample_ns - stamp_ns > max_dt_ns):
                return None
            fraction = (stamp_ns - previous_ns) / (sample_ns - previous_ns)
            p0 = _xyz(previous.pose.pose.position)
            p1 = _xyz(sample.pose.pose.position)
            q0 = _quat(previous.pose.pose.orientation)
            q1 = _quat(sample.pose.pose.orientation)
            position = tuple(
                (1.0 - fraction) * a + fraction * b
                for a, b in zip(p0, p1)
            )
            return position, _slerp(q0, q1, fraction)
        previous = sample

    return None


def _smooth_alignment(previous, target, p_odom_root, q_odom_root,
                      dt_s, tau_s, max_linear_speed_m_s,
                      max_angular_speed_deg_s):
    """Smooth a map->odom correction at the *current robot pose*.

    Interpolate the two map->root poses at the same odom->root pose, rate-limit
    both translation and rotation, then recover map->odom. This avoids the
    unintended position jump caused by rotating around a distant odom origin.
    Returns (map->odom translation, map->odom quaternion).
    """
    p_prev = _xyz(previous.transform.translation)
    q_prev = _quat(previous.transform.rotation)
    p_target = _xyz(target.transform.translation)
    q_target = _quat(target.transform.rotation)

    rotated_prev = _rotate(q_prev, p_odom_root)
    rotated_target = _rotate(q_target, p_odom_root)
    p_prev_root = tuple(p_prev[i] + rotated_prev[i] for i in range(3))
    p_target_root = tuple(p_target[i] + rotated_target[i] for i in range(3))
    q_prev_root = _multiply(q_prev, q_odom_root)
    q_target_root = _multiply(q_target, q_odom_root)

    # First-order smoothing, with an independent hard speed limit.
    alpha = -math.expm1(-dt_s / tau_s) if tau_s > 0.0 else 1.0
    delta = tuple(p_target_root[i] - p_prev_root[i] for i in range(3))
    distance = math.sqrt(sum(d * d for d in delta))
    linear_fraction = min(
        alpha, (max_linear_speed_m_s * dt_s / distance)
    ) if distance > 1e-9 else 1.0
    p_new_root = tuple(
        p_prev_root[i] + linear_fraction * delta[i] for i in range(3)
    )

    # SLERP over full 3D orientation, limited in degrees per second.
    dot = abs(sum(a * b for a, b in zip(q_prev_root, q_target_root)))
    angle = 2.0 * math.acos(max(-1.0, min(1.0, dot)))
    angular_fraction = min(
        alpha, math.radians(max_angular_speed_deg_s) * dt_s / angle
    ) if angle > 1e-9 else 1.0
    q_new_root = _slerp(q_prev_root, q_target_root, angular_fraction)

    # map->odom = (smoothed map->root) * inverse(odom->root)
    inverse_q_odom_root = (
        -q_odom_root[0], -q_odom_root[1],
        -q_odom_root[2], q_odom_root[3],
    )
    q_new = _multiply(q_new_root, inverse_q_odom_root)
    offset = _rotate(q_new, p_odom_root)
    p_new = tuple(p_new_root[i] - offset[i] for i in range(3))
    return p_new, q_new


def _candidate_difference(a, b):
    """Compare two candidate map -> odom transforms."""

    pa = _xyz(a.transform.translation)
    pb = _xyz(b.transform.translation)

    qa = _quat(a.transform.rotation)
    qb = _quat(b.transform.rotation)

    horizontal_error = math.hypot(
        pa[0] - pb[0],
        pa[1] - pb[1],
    )

    yaw_delta = _yaw(qa) - _yaw(qb)

    yaw_error_deg = abs(
        math.degrees(
            math.atan2(
                math.sin(yaw_delta),
                math.cos(yaw_delta),
            )
        )
    )

    return horizontal_error, yaw_error_deg

def _gnss_status_usable(status: FpaOdomstatus) -> bool:
    """Accept globally initialized fusion using RTK GNSS measurements."""

    consts = status.consts

    # Fixposition must be globally initialized.
    if status.init_status != consts.INIT_STATUS_GLOBAL_INIT:
        return False

    acceptable_fixes = {
        consts.GNSS_STATUS_RTK_FLOAT,
        consts.GNSS_STATUS_RTK_FIXED,
    }

    gnss1_usable = (
        status.gnss1_status in acceptable_fixes
        and status.fusion_gnss1 == consts.MEAS_STATUS_USED
    )

    gnss2_usable = (
        status.gnss2_status in acceptable_fixes
        and status.fusion_gnss2 == consts.MEAS_STATUS_USED
    )

    return gnss1_usable or gnss2_usable


class GnssAlignmentShadow(Node):
    def __init__(self):
        super().__init__("gnss_alignment_shadow")
        declare(
            self, "max_pair_dt_s", rclpy.Parameter.Type.DOUBLE,
            "Maximum allowed time gap to each FAST-LIO2 interpolation sample (s)",
            0.01, 1.0, default=0.15,
        )
        declare(
            self, "max_horizontal_std_m", rclpy.Parameter.Type.DOUBLE,
            "Maximum permitted Fixposition horizontal position standard deviation (m)",
            0.01, 50.0, default=5.0,
        )
        self._fast_lio = deque(maxlen=100)
        # Hold GNSS poses until a *later* FAST-LIO pose is available.
        # Bounded to avoid accumulating messages during a LiDAR outage.
        self._pending_fixposition = deque(maxlen=50)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._imu_to_root = None
        self._candidate_pub = self.create_publisher(
            TransformStamped, "fast_lio/gnss_alignment_candidate", 10
        )
        self._smoothed_pub = self.create_publisher(
            TransformStamped, "fast_lio/gnss_alignment_smoothed", 10
        )
        # Preview the future map->odom broadcaster without writing to /tf.
        self._tf_preview_pub = self.create_publisher(
            TransformStamped, "fast_lio/gnss_alignment_tf_preview", 10
        )
        # Prepared for a future atomic TF handover. The deployment explicitly
        # sets this to False; leave Fixposition as the live TF owner for now.
        live_tf = declare(
            self, "publish_live_tf", rclpy.Parameter.Type.BOOL,
            "Broadcast the accepted map-to-odom correction to live TF (unsafe until handover)",
            default=False,
        )
        self._live_tf_broadcaster = TransformBroadcaster(self) if live_tf else None
        preview_rate_hz = declare(
            self, "alignment_tf_preview_rate_hz", rclpy.Parameter.Type.DOUBLE,
            "Diagnostic map-to-odom preview publication frequency (Hz)",
            1.0, 100.0, default=20.0,
        )
        declare(
            self, "fast_lio_watchdog_timeout_s", rclpy.Parameter.Type.DOUBLE,
            "Maximum elapsed time since advancing FAST-LIO2 odometry (s)",
            0.1, 10.0, default=1.0,
        )
        self._last_lio_arrival_monotonic_ns = None
        self._tf_preview_was_stale = False
        self._tf_preview_timer = self.create_timer(
            1.0 / preview_rate_hz, self._publish_tf_preview
        )
        # Rate limits apply to the *robot pose* and not the odom origin.
        # Diagnostic starting values; calibrate from physical logs.
        declare(
            self, "alignment_smoothing_tau_s", rclpy.Parameter.Type.DOUBLE,
            "Time constant for diagnostic global-alignment smoothing (s)",
            0.01, 60.0, default=2.0,
        )
        declare(
            self, "alignment_max_translation_speed_m_s", rclpy.Parameter.Type.DOUBLE,
            "Maximum diagnostic global-position correction speed (m/s)",
            0.001, 5.0, default=0.2,
        )
        declare(
            self, "alignment_max_rotation_speed_deg_s", rclpy.Parameter.Type.DOUBLE,
            "Maximum diagnostic heading correction rate (deg/s)",
            0.1, 180.0, default=5.0,
        )
        # Prevent a large update after a long GNSS dropout/reacquisition.
        declare(
            self, "alignment_max_update_dt_s", rclpy.Parameter.Type.DOUBLE,
            "Maximum elapsed time per smoothing update (s)",
            0.01, 5.0, default=0.2,
        )
        self._smoothed_alignment = None
        self._last_smoothed_stamp_ns = None
        self._lio_sub = self.create_subscription(
            Odometry, "fast_lio/odometry", self._on_lio, qos_profile_sensor_data
        )
        self._fixposition_sub = self.create_subscription(
            Odometry, "fixposition/odometry_enu", self._on_fixposition,
            qos_profile_sensor_data,
        )
        if live_tf:
            self.get_logger().warning(
                "EXPERIMENTAL: GNSS alignment now broadcasts map->odom on live TF; "
                "Fixposition TF must be isolated and FAST-LIO odometry must own odom->vrtk_link"
            )
        else:
            self.get_logger().info(
                "Shadow GNSS alignment: diagnostic topic only; does not publish TF"
            )

        declare(
            self, "max_status_dt_s", rclpy.Parameter.Type.DOUBLE,
            "Maximum GNSS odometry versus status timestamp difference (s)",
            0.01, 10.0, default=1.0,
        )

        self._latest_status = None

        self._status_sub = self.create_subscription(
            FpaOdomstatus,
            "fixposition/fpa/odomstatus",
            self._on_status,
            qos_profile_sensor_data,
        )

        # Initial outlier thresholds.
        # These must be validated on physical hardware.
        declare(
            self, "max_position_residual_m", rclpy.Parameter.Type.DOUBLE,
            "Maximum ordinary horizontal alignment innovation (m)",
            0.01, 100.0, default=1.0,
        )
        declare(
            self, "max_yaw_residual_deg", rclpy.Parameter.Type.DOUBLE,
            "Maximum ordinary heading alignment innovation (deg)",
            0.1, 180.0, default=5.0,
        )

        # Last accepted alignment.
        self._last_accepted_alignment = None
        self._last_accepted_stamp_ns = None

        # Consecutive alignment residual failures (not GNSS quality failures).
        self._rejected_count = 0
        self._first_rejected_stamp_ns = None

        # GNSS reacquisition parameters.
        # Initial values; validate on physical hardware.
        declare(
            self, "reacquire_after_s", rclpy.Parameter.Type.DOUBLE,
            "Qualified GNSS interruption duration that triggers reacquisition (s)",
            0.1, 120.0, default=2.0,
        )
        declare(
            self, "reacquire_min_samples", rclpy.Parameter.Type.INTEGER,
            "Minimum consistent GNSS candidates for reacquisition",
            2, 200, default=5,
        )
        declare(
            self, "reacquire_min_duration_s", rclpy.Parameter.Type.DOUBLE,
            "Minimum duration covered by reacquisition candidates (s)",
            0.05, 120.0, default=0.5,
        )
        # Persistent residual failures can also indicate accumulated LIO drift
        # even if the stream of qualified GNSS measurements never stops.
        declare(
            self, "reacquire_after_rejections", rclpy.Parameter.Type.INTEGER,
            "Consecutive alignment outliers before reacquisition",
            2, 200, default=5,
        )
        declare(
            self, "reacquire_min_rejection_duration_s", rclpy.Parameter.Type.DOUBLE,
            "Minimum duration of consecutive outliers before reacquisition (s)",
            0.1, 120.0, default=1.0,
        )
        declare(
            self, "reacquire_max_position_spread_m", rclpy.Parameter.Type.DOUBLE,
            "Maximum candidate translation spread during reacquisition (m)",
            0.01, 50.0, default=0.75,
        )
        declare(
            self, "reacquire_max_yaw_spread_deg", rclpy.Parameter.Type.DOUBLE,
            "Maximum candidate yaw spread during reacquisition (deg)",
            0.1, 180.0, default=5.0,
        )

        # Timestamp of the most recent measurement that passed
        # GNSS quality, covariance and LiDAR timestamp pairing checks.
        self._last_qualified_stamp_ns = None

        # Also verify multiple measurements during initial alignment.
        self._reacquiring = True

        # Pending alignment candidates: (timestamp_ns, transform).
        self._reacquire_candidates = deque(maxlen=200)
        

    def _publish_tf_preview(self):
        """Publish a gated diagnostic preview; optionally mirror to live TF.

        This intentionally holds map->odom constant during GNSS outages.
        FAST-LIO2 odom->root must continue advancing independently. A future
        live TF broadcaster also needs local-odometry freshness monitoring.
        """
        if self._smoothed_alignment is None:
            return

        # Arrival-time watchdog: do not compare hardware stamps with ROS time.
        # Only advancing, correctly framed LIO messages refresh the deadline.
        last = self._last_lio_arrival_monotonic_ns
        timeout_ns = int(
            self.get_parameter("fast_lio_watchdog_timeout_s").value * 1e9
        )
        if last is None or time.monotonic_ns() - last > timeout_ns:
            if not self._tf_preview_was_stale:
                self.get_logger().warning(
                    "FAST-LIO2 odometry stale: withholding map->odom TF preview"
                )
                self._tf_preview_was_stale = True
            return
        if self._tf_preview_was_stale:
            self.get_logger().info("FAST-LIO2 odometry resumed; TF preview restored")
            self._tf_preview_was_stale = False

        preview = TransformStamped()
        preview.header.stamp = self.get_clock().now().to_msg()
        if _time_ns(preview.header.stamp) <= 0:
            return
        preview.header.frame_id = self._smoothed_alignment.header.frame_id
        preview.child_frame_id = self._smoothed_alignment.child_frame_id
        preview.transform = self._smoothed_alignment.transform
        self._tf_preview_pub.publish(preview)
        # Only the same accepted, timestamp-refreshed and watchdog-gated
        # transform may reach the live TF tree after explicit opt-in.
        if self._live_tf_broadcaster is not None:
            self._live_tf_broadcaster.sendTransform(preview)

    def _publish_smoothed_alignment(self, candidate, stamp_ns,
                                    p_odom_root, q_odom_root):
        """Publish a rate-limited diagnostic transform; never broadcast TF."""
        smoothed = candidate
        if self._smoothed_alignment is not None:
            elapsed_s = (stamp_ns - self._last_smoothed_stamp_ns) * 1e-9
            if elapsed_s <= 0.0:
                return
            dt_s = min(
                elapsed_s,
                self.get_parameter("alignment_max_update_dt_s").value,
            )
            try:
                translation, rotation = _smooth_alignment(
                    self._smoothed_alignment, candidate,
                    p_odom_root, q_odom_root,
                    dt_s,
                    self.get_parameter("alignment_smoothing_tau_s").value,
                    self.get_parameter("alignment_max_translation_speed_m_s").value,
                    self.get_parameter("alignment_max_rotation_speed_deg_s").value,
                )
            except ValueError:
                self.get_logger().warning("Ignoring invalid smoothing pose")
                return

            smoothed = TransformStamped()
            smoothed.header = candidate.header
            smoothed.child_frame_id = candidate.child_frame_id
            (smoothed.transform.translation.x,
             smoothed.transform.translation.y,
             smoothed.transform.translation.z) = translation
            (smoothed.transform.rotation.x,
             smoothed.transform.rotation.y,
             smoothed.transform.rotation.z,
             smoothed.transform.rotation.w) = rotation

        # The first quality-verified alignment establishes the initial anchor.
        # After GNSS outages, the anchor is retained and moved only gradually.
        self._smoothed_alignment = smoothed
        self._last_smoothed_stamp_ns = stamp_ns
        self._smoothed_pub.publish(smoothed)

    def _on_lio(self, message):
        if (message.header.frame_id, message.child_frame_id) != (
            "camera_init", "body"
        ):
            return
        if _time_ns(message.header.stamp) <= 0:
            return
        stamp_ns = _time_ns(message.header.stamp)
        if (self._fast_lio
                and stamp_ns <= _time_ns(self._fast_lio[-1].header.stamp)):
            # Interpolation requires time-ordered FAST-LIO samples.
            return
        self._fast_lio.append(message)
        self._last_lio_arrival_monotonic_ns = time.monotonic_ns()
        self._drain_pending_fixposition()

    def _on_status(self, message: FpaOdomstatus):
        if _time_ns(message.header.stamp) > 0:
            self._latest_status = message

    def _on_fixposition(self, gnss):
        if (gnss.header.frame_id, gnss.child_frame_id) != (
            "map", "vrtk_link"
        ):
            return
        stamp_ns = _time_ns(gnss.header.stamp)
        if stamp_ns <= 0:
            return

        # Require recent Fixposition GNSS status.
        status = self._latest_status
        if status is None:
            return
        status_dt_s = abs(
            stamp_ns - _time_ns(status.header.stamp)
        ) * 1e-9
        if status_dt_s > self.get_parameter("max_status_dt_s").value:
            return
        if not _gnss_status_usable(status):
            return

        covariance = gnss.pose.covariance
        vx, vy = covariance[0], covariance[7]
        if not all(math.isfinite(x) and x > 0 for x in (vx, vy)):
            return
        if math.sqrt(max(vx, vy)) > self.get_parameter(
            "max_horizontal_std_m"
        ).value:
            return

        # This measurement passed the quality checks. Defer alignment until
        # the FAST-LIO stream contains samples on both sides of its timestamp.
        if (self._pending_fixposition and stamp_ns <=
                _time_ns(self._pending_fixposition[-1].header.stamp)):
            return
        self._pending_fixposition.append(gnss)
        self._drain_pending_fixposition()

    def _drain_pending_fixposition(self):
        if not self._fast_lio:
            return

        earliest_ns = _time_ns(self._fast_lio[0].header.stamp)
        latest_ns = _time_ns(self._fast_lio[-1].header.stamp)

        while self._pending_fixposition:
            gnss = self._pending_fixposition[0]
            stamp_ns = _time_ns(gnss.header.stamp)
            if stamp_ns < earliest_ns:
                # Too old to interpolate from the retained FAST-LIO history.
                self._pending_fixposition.popleft()
                continue
            if stamp_ns > latest_ns:
                # A later FAST-LIO pose has not arrived yet. Never extrapolate.
                break
            self._pending_fixposition.popleft()
            self._process_matched_fixposition(gnss)

    def _process_matched_fixposition(self, gnss):
        stamp_ns = _time_ns(gnss.header.stamp)
        try:
            lio_pose = _interpolate_lio(
                self._fast_lio,
                stamp_ns,
                self.get_parameter("max_pair_dt_s").value,
            )
        except ValueError:
            return
        if lio_pose is None:
            return
        p_odom_imu, q_odom_imu = lio_pose

        if self._imu_to_root is None:
            try:
                self._imu_to_root = self._tf_buffer.lookup_transform(
                    "microstrain_imu_link", "vrtk_link", Time()
                )
            except TransformException:
                return

        try:
            p_map_root = _xyz(gnss.pose.pose.position)
            q_map_root = _quat(gnss.pose.pose.orientation)
            p_imu_root = _xyz(self._imu_to_root.transform.translation)
            q_imu_root = _quat(self._imu_to_root.transform.rotation)
        except ValueError:
            return
        if not all(math.isfinite(v) for v in (*p_map_root, *p_odom_imu, *p_imu_root)):
            return

        # T_odom_root = T_odom_imu * T_imu_root
        offset = _rotate(q_odom_imu, p_imu_root)
        p_odom_root = tuple(p_odom_imu[i] + offset[i] for i in range(3))
        q_odom_root = _multiply(q_odom_imu, q_imu_root)

        # T_map_odom = T_map_root(GNSS) * inverse(T_odom_root(LIO))
        inverse_q_odom_root = (
            -q_odom_root[0], -q_odom_root[1], -q_odom_root[2], q_odom_root[3]
        )
        q_map_odom = _multiply(q_map_root, inverse_q_odom_root)
        rotated = _rotate(q_map_odom, p_odom_root)
        p_map_odom = tuple(p_map_root[i] - rotated[i] for i in range(3))

        candidate = TransformStamped()
        candidate.header.stamp = gnss.header.stamp
        candidate.header.frame_id = "map"
        candidate.child_frame_id = "odom"  # Message only; not inserted into /tf.
        (candidate.transform.translation.x,
         candidate.transform.translation.y,
         candidate.transform.translation.z) = p_map_odom
        (candidate.transform.rotation.x,
         candidate.transform.rotation.y,
         candidate.transform.rotation.z,
         candidate.transform.rotation.w) = q_map_odom

        # Reject out-of-order or duplicate qualified GNSS measurements.
        if (
            self._last_qualified_stamp_ns is not None
            and stamp_ns <= self._last_qualified_stamp_ns
        ):
            return

        # Detect an interruption in qualified GNSS measurements.
        if self._last_qualified_stamp_ns is not None:
            gap_s = (
                stamp_ns - self._last_qualified_stamp_ns
            ) * 1e-9

            if gap_s > self.get_parameter("reacquire_after_s").value:
                self._reacquiring = True
                self._reacquire_candidates.clear()
                self._rejected_count = 0
                self._first_rejected_stamp_ns = None

                self.get_logger().warning(
                    f"GNSS alignment interrupted for {gap_s:.2f} s; "
                    "starting reacquisition verification"
                )

        self._last_qualified_stamp_ns = stamp_ns

        if self._reacquiring:

            # Compare each new candidate against the first candidate
            # in the current verification sequence.
            if self._reacquire_candidates:

                reference = self._reacquire_candidates[0][1]

                position_error, yaw_error = _candidate_difference(
                    candidate,
                    reference,
                )

                max_position = self.get_parameter(
                    "reacquire_max_position_spread_m"
                ).value

                max_yaw = self.get_parameter(
                    "reacquire_max_yaw_spread_deg"
                ).value

                if (
                    position_error > max_position
                    or yaw_error > max_yaw
                ):
                    # Inconsistent candidate: restart verification.
                    self._reacquire_candidates.clear()

            self._reacquire_candidates.append(
                (stamp_ns, candidate)
            )

            first_stamp_ns = self._reacquire_candidates[0][0]

            duration_s = (
                stamp_ns - first_stamp_ns
            ) * 1e-9

            min_samples = self.get_parameter(
                "reacquire_min_samples"
            ).value

            min_duration = self.get_parameter(
                "reacquire_min_duration_s"
            ).value

            if (
                len(self._reacquire_candidates) < min_samples
                or duration_s < min_duration
            ):
                return

            # Several consistent GNSS measurements have been observed.
            # Accept this candidate as the new diagnostic reference.
            self._last_accepted_alignment = candidate
            self._last_accepted_stamp_ns = stamp_ns
            self._rejected_count = 0
            self._first_rejected_stamp_ns = None

            self._reacquiring = False
            self._reacquire_candidates.clear()

            self.get_logger().info(
                "GNSS alignment reacquired after "
                f"{duration_s:.2f} s of consistent measurements"
            )

            # Keep the existing raw diagnostic and add a smoothed diagnostic.
            self._candidate_pub.publish(candidate)
            self._publish_smoothed_alignment(
                candidate, stamp_ns, p_odom_root, q_odom_root
            )

            return
        
        # Reject out-of-order GNSS measurements.
        if (
            self._last_accepted_stamp_ns is not None
            and stamp_ns <= self._last_accepted_stamp_ns
        ):
            return

        if self._last_accepted_alignment is not None:

            previous = self._last_accepted_alignment.transform

            p_previous = _xyz(previous.translation)
            q_previous = _quat(previous.rotation)

            # Predict the current robot pose in the map frame using
            # the previously accepted alignment and current FAST-LIO2 pose.
            rotated_position = _rotate(q_previous, p_odom_root)

            p_predicted = tuple(
                p_previous[i] + rotated_position[i]
                for i in range(3)
            )

            q_predicted = _multiply(q_previous, q_odom_root)

            # Horizontal position residual.
            position_residual = math.hypot(
                p_map_root[0] - p_predicted[0],
                p_map_root[1] - p_predicted[1],
            )

            # Yaw residual, accounting for angle wrapping.
            yaw_difference = _yaw(q_map_root) - _yaw(q_predicted)

            yaw_residual_deg = abs(
                math.degrees(
                    math.atan2(
                        math.sin(yaw_difference),
                        math.cos(yaw_difference),
                    )
                )
            )

            max_position = self.get_parameter(
                "max_position_residual_m"
            ).value

            max_yaw = self.get_parameter(
                "max_yaw_residual_deg"
            ).value

            # Reject inconsistent measurements.
            if (
                position_residual > max_position
                or yaw_residual_deg > max_yaw
            ):
                if self._first_rejected_stamp_ns is None:
                    self._first_rejected_stamp_ns = stamp_ns
                self._rejected_count += 1

                if (
                    self._rejected_count == 1
                    or self._rejected_count % 50 == 0
                ):
                    self.get_logger().warning(
                        "GNSS alignment rejected: "
                        f"position residual={position_residual:.2f} m, "
                        f"yaw residual={yaw_residual_deg:.2f} deg"
                    )

                rejected_duration_s = (
                    stamp_ns - self._first_rejected_stamp_ns
                ) * 1e-9
                if (
                    self._rejected_count >= self.get_parameter(
                        "reacquire_after_rejections"
                    ).value
                    and rejected_duration_s >= self.get_parameter(
                        "reacquire_min_rejection_duration_s"
                    ).value
                ):
                    # Do not immediately accept a large discrepancy. Instead,
                    # begin the same multi-sample consistency verification used
                    # after a gap in qualified GNSS measurements.
                    self._reacquiring = True
                    self._reacquire_candidates.clear()
                    self._reacquire_candidates.append((stamp_ns, candidate))
                    self.get_logger().warning(
                        "Sustained alignment residual failures for "
                        f"{rejected_duration_s:.2f} s; verifying a new "
                        "diagnostic alignment (no TF publication)"
                    )

                return

        # Accept candidate alignment; publish both diagnostic views.
        self._candidate_pub.publish(candidate)
        self._publish_smoothed_alignment(
            candidate, stamp_ns, p_odom_root, q_odom_root
        )

        self._last_accepted_alignment = candidate
        self._last_accepted_stamp_ns = stamp_ns
        self._rejected_count = 0
        self._first_rejected_stamp_ns = None


def main(args=None):
    rclpy.init(args=args)
    node = GnssAlignmentShadow()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
