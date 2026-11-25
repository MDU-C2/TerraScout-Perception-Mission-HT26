"""Deterministic checks for the FAST-LIO2/Fixposition shadow alignment.

These tests exercise callbacks directly; no sensors, DDS publishers, or TF
broadcaster are required. Run in a ROS 2 environment with pytest.
"""

import math
import time
from types import SimpleNamespace

import pytest
import rclpy
from fixposition_driver_msgs.msg import FpaOdomstatus
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry

from terrascout_navigation import gnss_alignment_shadow as alignment


def _stamp(seconds):
    ns = round(seconds * 1_000_000_000)
    return SimpleNamespace(sec=ns // 1_000_000_000, nanosec=ns % 1_000_000_000)


def _set_stamp(message, seconds):
    stamp = _stamp(seconds)
    message.header.stamp.sec = stamp.sec
    message.header.stamp.nanosec = stamp.nanosec


def _yaw_quat(degrees):
    angle = math.radians(degrees) / 2.0
    return (0.0, 0.0, math.sin(angle), math.cos(angle))


def _set_pose(pose, x=0.0, y=0.0, yaw_deg=0.0):
    pose.position.x = float(x)
    pose.position.y = float(y)
    q = _yaw_quat(yaw_deg)
    (pose.orientation.x, pose.orientation.y,
     pose.orientation.z, pose.orientation.w) = q


def _lio(t, x=0.0, y=0.0, yaw_deg=0.0):
    message = Odometry()
    message.header.frame_id = "camera_init"
    message.child_frame_id = "body"
    _set_stamp(message, t)
    _set_pose(message.pose.pose, x, y, yaw_deg)
    return message


def _gnss(t, x, y=0.0, yaw_deg=0.0, variance=0.01):
    message = Odometry()
    message.header.frame_id = "map"
    message.child_frame_id = "vrtk_link"
    _set_stamp(message, t)
    _set_pose(message.pose.pose, x, y, yaw_deg)
    message.pose.covariance[0] = variance
    message.pose.covariance[7] = variance
    return message


def _status(t, fix=True):
    message = FpaOdomstatus()
    _set_stamp(message, t)
    c = message.consts
    message.init_status = c.INIT_STATUS_GLOBAL_INIT
    message.gnss1_status = c.GNSS_STATUS_RTK_FIXED if fix else c.GNSS_STATUS_NO_FIX
    message.fusion_gnss1 = c.MEAS_STATUS_USED if fix else c.MEAS_STATUS_NOT_USED
    return message


def _transform(x=0.0, y=0.0, yaw_deg=0.0):
    transform = TransformStamped()
    transform.transform.translation.x = x
    transform.transform.translation.y = y
    q = _yaw_quat(yaw_deg)
    (transform.transform.rotation.x, transform.transform.rotation.y,
     transform.transform.rotation.z, transform.transform.rotation.w) = q
    return transform


class _Recorder:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


@pytest.fixture(scope="module", autouse=True)
def _ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def node():
    result = alignment.GnssAlignmentShadow()
    # Use a known static IMU -> vrtk_link transform (identity) in unit tests.
    # URDF frame calibration is a separate hardware validation concern.
    result._imu_to_root = _transform()
    result._candidate_pub = _Recorder()
    result._smoothed_pub = _Recorder()
    yield result
    result.destroy_node()


def _step(node, t, lio_x, gnss_x, fix=True):
    """Give the node two bracketing LIO poses and an epoch-matched GNSS pose."""
    node._on_lio(_lio(t - 0.02, lio_x - 0.02))
    node._on_lio(_lio(t + 0.02, lio_x + 0.02))
    node._on_status(_status(t, fix=fix))
    node._on_fixposition(_gnss(t, gnss_x))


def _initialize_alignment(node):
    # Five consistent candidates over 0.8s satisfy the initial gate.
    for i in range(5):
        t = 100.0 + i * 0.2
        _step(node, t, lio_x=i * 0.2, gnss_x=2.0 + i * 0.2)
    assert not node._reacquiring
    assert len(node._candidate_pub.messages) == 1
    assert len(node._smoothed_pub.messages) == 1


def test_interpolation_of_position_and_orientation():
    messages = [_lio(10.0, x=0, yaw_deg=0), _lio(10.1, x=10, yaw_deg=90)]
    result = alignment._interpolate_lio(messages, alignment._time_ns(_stamp(10.05)), 0.15)
    assert result is not None
    position, rotation = result
    assert position[0] == pytest.approx(5.0)
    assert math.degrees(alignment._yaw(rotation)) == pytest.approx(45.0)


def test_interpolation_never_extrapolates():
    messages = [_lio(10.0), _lio(10.1)]
    for t in (9.99, 10.11):
        assert alignment._interpolate_lio(
            messages, alignment._time_ns(_stamp(t)), 0.15
        ) is None


def test_interpolation_rejects_wide_time_gaps():
    messages = [_lio(10.0), _lio(11.0)]
    assert alignment._interpolate_lio(
        messages, alignment._time_ns(_stamp(10.5)), 0.15
    ) is None


def test_slerp_handles_quaternion_sign_flip():
    quat = _yaw_quat(20)
    opposite = tuple(-v for v in quat)
    result = alignment._slerp(quat, opposite, 0.5)
    assert math.degrees(alignment._yaw(result)) == pytest.approx(20)
    assert math.sqrt(sum(v * v for v in result)) == pytest.approx(1)


def test_gnss_quality_rejects_unusable_fix():
    valid = _status(10)
    assert alignment._gnss_status_usable(valid)
    valid.gnss1_status = valid.consts.GNSS_STATUS_NO_FIX
    assert not alignment._gnss_status_usable(valid)
    valid = _status(10)
    valid.fusion_gnss1 = valid.consts.MEAS_STATUS_NOT_USED
    assert not alignment._gnss_status_usable(valid)
    valid = _status(10)
    valid.init_status = valid.consts.INIT_STATUS_LOCAL_INIT
    assert not alignment._gnss_status_usable(valid)


def test_smoothing_translation_hard_speed_limit():
    p, q = alignment._smooth_alignment(
        _transform(x=0), _transform(x=4),
        (0.0, 0.0, 0.0), _yaw_quat(0),
        dt_s=0.1, tau_s=2.0,
        max_linear_speed_m_s=0.2, max_angular_speed_deg_s=5.0,
    )
    assert p[0] == pytest.approx(0.02)
    assert math.degrees(alignment._yaw(q)) == pytest.approx(0)


def test_smoothing_limits_robot_position_at_distant_odom_origin():
    p_root = (100.0, 0.0, 0.0)
    p, q = alignment._smooth_alignment(
        _transform(), _transform(yaw_deg=90),
        p_root, _yaw_quat(0),
        dt_s=0.1, tau_s=2.0,
        max_linear_speed_m_s=0.2, max_angular_speed_deg_s=5.0,
    )
    predicted = alignment._rotate(q, p_root)
    predicted = tuple(p[i] + predicted[i] for i in range(3))
    assert math.dist(predicted, p_root) <= 0.020001
    assert abs(math.degrees(alignment._yaw(q))) <= 0.500001


def test_queued_gnss_waits_for_later_lio_sample(node):
    t = 100.0
    node._on_lio(_lio(t - 0.02, 0.0))
    node._on_status(_status(t))
    node._on_fixposition(_gnss(t, x=2.02))
    assert len(node._pending_fixposition) == 1
    assert len(node._reacquire_candidates) == 0
    node._on_lio(_lio(t + 0.02, 0.04))
    assert len(node._pending_fixposition) == 0
    assert len(node._reacquire_candidates) == 1


def test_initial_alignment_needs_multiple_consistent_measurements(node):
    for i in range(4):
        _step(node, 100.0 + i * 0.2, i * 0.2, 2.0 + i * 0.2)
        assert len(node._candidate_pub.messages) == 0
    _step(node, 100.8, 0.8, 2.8)
    assert len(node._candidate_pub.messages) == 1
    assert node._candidate_pub.messages[-1].transform.translation.x == pytest.approx(2)
    assert len(node._smoothed_pub.messages) == 1


def test_single_gnss_jump_is_rejected(node):
    _initialize_alignment(node)
    _step(node, 101.0, lio_x=1.0, gnss_x=11.0)
    assert len(node._candidate_pub.messages) == 1
    assert len(node._smoothed_pub.messages) == 1
    assert node._rejected_count == 1
    assert not node._reacquiring


def test_bad_fix_is_not_used(node):
    _initialize_alignment(node)
    _step(node, 101.0, lio_x=1.0, gnss_x=3.0, fix=False)
    assert len(node._candidate_pub.messages) == 1
    assert len(node._smoothed_pub.messages) == 1


def test_sustained_disagreement_enters_reacquisition_and_rate_limits(node):
    _initialize_alignment(node)
    # Candidate offset changes from 2m to 6m but individual fixes agree.
    # Six 0.2s-spaced failures span 1.0s and start verification.
    for i in range(6):
        t = 101.0 + i * 0.2
        lio_x = t - 100.0
        _step(node, t, lio_x=lio_x, gnss_x=lio_x + 6.0)
    assert node._reacquiring
    assert len(node._candidate_pub.messages) == 1
    # Four further candidates make five consistent samples spanning 0.8s.
    for i in range(4):
        t = 102.2 + i * 0.2
        lio_x = t - 100.0
        _step(node, t, lio_x=lio_x, gnss_x=lio_x + 6.0)
    assert not node._reacquiring
    assert len(node._candidate_pub.messages) == 2
    assert len(node._smoothed_pub.messages) == 2
    # 0.2 m/s * capped dt (0.2s) = 0.04m maximum movement.
    previous_x = node._smoothed_pub.messages[0].transform.translation.x
    current_x = node._smoothed_pub.messages[1].transform.translation.x
    assert 0.0 < current_x - previous_x <= 0.040001


def test_long_gnss_outage_requires_reacquisition(node):
    _initialize_alignment(node)
    # No qualified GNSS for > 2s. The next accepted raw correction is
    # deferred until five consistent fixes have been observed.
    for i in range(4):
        t = 104.0 + i * 0.2
        lio_x = t - 100.0
        _step(node, t, lio_x, lio_x + 3.0)
        assert len(node._candidate_pub.messages) == 1
    _step(node, 104.8, 4.8, 7.8)
    assert len(node._candidate_pub.messages) == 2
    assert not node._reacquiring


def test_tf_preview_waits_for_initial_alignment(node):
    node._tf_preview_pub = _Recorder()
    node._publish_tf_preview()
    assert node._tf_preview_pub.messages == []


def test_tf_preview_retains_alignment_without_gnss(node):
    node._tf_preview_pub = _Recorder()
    _initialize_alignment(node)
    assert node._smoothed_alignment is not None
    original = node._smoothed_alignment
    # No further GNSS measurements arrive; the preview still updates.
    node._publish_tf_preview()
    node._publish_tf_preview()
    assert len(node._tf_preview_pub.messages) == 2
    for preview in node._tf_preview_pub.messages:
        assert preview.header.frame_id == "map"
        assert preview.child_frame_id == "odom"
        assert alignment._time_ns(preview.header.stamp) > 0
        assert preview.transform.translation.x == pytest.approx(
            original.transform.translation.x
        )
    assert len(node._smoothed_pub.messages) == 1


def test_tf_preview_stops_when_fast_lio_is_stale(node):
    node._tf_preview_pub = _Recorder()
    _initialize_alignment(node)
    node._publish_tf_preview()
    assert len(node._tf_preview_pub.messages) == 1

    # Simulate an odometry outage without sleeping or changing sensor time.
    timeout = node.get_parameter("fast_lio_watchdog_timeout_s").value
    node._last_lio_arrival_monotonic_ns = (
        time.monotonic_ns() - int((timeout + 0.2) * 1e9)
    )
    node._publish_tf_preview()
    node._publish_tf_preview()
    assert len(node._tf_preview_pub.messages) == 1
    assert node._tf_preview_was_stale


def test_tf_preview_resumes_only_after_advancing_fast_lio(node):
    node._tf_preview_pub = _Recorder()
    _initialize_alignment(node)
    timeout = node.get_parameter("fast_lio_watchdog_timeout_s").value
    expired = time.monotonic_ns() - int((timeout + 0.2) * 1e9)
    node._last_lio_arrival_monotonic_ns = expired

    # A duplicate timestamp must not extend the watchdog deadline.
    last_stamp_ns = alignment._time_ns(node._fast_lio[-1].header.stamp)
    node._on_lio(node._fast_lio[-1])
    assert node._last_lio_arrival_monotonic_ns == expired
    assert alignment._time_ns(node._fast_lio[-1].header.stamp) == last_stamp_ns
    node._publish_tf_preview()
    assert node._tf_preview_pub.messages == []

    # Newly advancing local odometry restores the preview without GNSS data.
    node._on_lio(_lio(101.0, 1.0))
    node._publish_tf_preview()
    assert len(node._tf_preview_pub.messages) == 1
    assert not node._tf_preview_was_stale


class _TfRecorder:
    def __init__(self):
        self.transforms = []

    def sendTransform(self, transform):
        self.transforms.append(transform)


def test_live_tf_is_disabled_by_default(node):
    # The original physical and simulation targets must never activate a new
    # map->odom publisher simply because this node exists in the workspace.
    assert node.get_parameter("publish_live_tf").value is False
    assert node._live_tf_broadcaster is None


def test_live_tf_uses_the_same_preview_and_watchdog_gate(node):
    node._tf_preview_pub = _Recorder()
    # Exercise the optional forwarding path without creating a real /tf writer.
    forwarded = _TfRecorder()
    node._live_tf_broadcaster = forwarded
    _initialize_alignment(node)
    node._publish_tf_preview()
    assert len(forwarded.transforms) == 1
    assert forwarded.transforms[-1] is node._tf_preview_pub.messages[-1]
    assert forwarded.transforms[-1].header.frame_id == "map"
    assert forwarded.transforms[-1].child_frame_id == "odom"

    timeout = node.get_parameter("fast_lio_watchdog_timeout_s").value
    node._last_lio_arrival_monotonic_ns = (
        time.monotonic_ns() - int((timeout + 0.2) * 1e9)
    )
    node._publish_tf_preview()
    assert len(forwarded.transforms) == 1

    # A genuinely advancing FAST-LIO measurement reopens the gate.
    node._on_lio(_lio(101.0, 1.0))
    node._publish_tf_preview()
    assert len(forwarded.transforms) == 2
