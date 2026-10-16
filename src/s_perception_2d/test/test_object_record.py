"""
Unit tests for ObjectRecord and ObjectRecordHandler.
"""

import pytest
from s_perception_2d.object_record import ObjectRecord, ObjectRecordHandler


def test_configured_class_assignment():
    handler = ObjectRecordHandler(
        configured_classes=["pedestrian_sign", "roadwork_sign", "stop-sign", "warning_bag"],
        confidence_threshold=0.35,
    )

    record = handler.validate_and_create_record(
        raw_class_name="warning_bag",
        confidence=0.88,
        xyxy=(100.0, 150.0, 300.0, 450.0),
        frame_id="oakd_rgb_camera_optical_frame",
    )

    assert record.is_valid is True
    assert record.class_name == "warning_bag"
    assert record.raw_class_name == "warning_bag"
    assert record.confidence == 0.88
    assert record.x1 == 100.0
    assert record.y1 == 150.0
    assert record.x2 == 300.0
    assert record.y2 == 450.0
    assert record.center_u == 200.0
    assert record.center_v == 300.0
    assert record.width == 200.0
    assert record.height == 300.0
    assert record.validation_status == "VALID"


def test_unknown_fallback_for_unsupported_class():
    handler = ObjectRecordHandler(
        configured_classes=["pedestrian_sign", "roadwork_sign", "stop-sign", "warning_bag"],
        confidence_threshold=0.35,
        allow_unknown_records=True,
    )

    # Class not in configured classes
    record = handler.validate_and_create_record(
        raw_class_name="random_obstacle",
        confidence=0.75,
        xyxy=(50.0, 50.0, 150.0, 150.0),
        frame_id="oakd_rgb_camera_optical_frame",
    )

    assert record.is_valid is True
    assert record.class_name == "unknown"
    assert record.raw_class_name == "random_obstacle"
    assert "ASSIGNED_UNKNOWN" in record.validation_status


def test_confidence_threshold_rejection():
    handler = ObjectRecordHandler(
        configured_classes=["warning_bag"],
        confidence_threshold=0.50,
    )

    record = handler.validate_and_create_record(
        raw_class_name="warning_bag",
        confidence=0.30,  # Below threshold
        xyxy=(10.0, 10.0, 100.0, 100.0),
    )

    assert record.is_valid is False
    assert "LOW_CONFIDENCE" in record.validation_status


def test_box_size_validation():
    handler = ObjectRecordHandler(
        configured_classes=["warning_bag"],
        min_box_size_px=10.0,
    )

    record = handler.validate_and_create_record(
        raw_class_name="warning_bag",
        confidence=0.90,
        xyxy=(10.0, 10.0, 12.0, 12.0),  # 2x2 box, smaller than 10px
    )

    assert record.is_valid is False
    assert "BOX_TOO_SMALL" in record.validation_status


def test_to_dict_serialization():
    record = ObjectRecord(
        class_name="warning_bag",
        raw_class_name="warning_bag",
        confidence=0.95,
        x1=10.0,
        y1=20.0,
        x2=50.0,
        y2=80.0,
        center_u=30.0,
        center_v=50.0,
        width=40.0,
        height=60.0,
        frame_id="camera_frame",
    )

    d = record.to_dict()
    assert d["class_name"] == "warning_bag"
    assert d["bbox"]["center_u"] == 30.0
    assert d["bbox"]["center_v"] == 50.0
    assert d["bbox"]["width"] == 40.0
    assert d["bbox"]["height"] == 60.0


def test_to_ros_detection2d():
    try:
        from vision_msgs.msg import Detection2D
    except ImportError:
        pytest.skip("vision_msgs not available")

    record = ObjectRecord(
        class_name="warning_bag",
        raw_class_name="warning_bag",
        confidence=0.92,
        x1=100.0,
        y1=120.0,
        x2=200.0,
        y2=260.0,
        center_u=150.0,
        center_v=190.0,
        width=100.0,
        height=140.0,
        frame_id="camera_optical_frame",
    )

    ros_det = record.to_ros_detection2d()
    assert ros_det is not None
    assert ros_det.header.frame_id == "camera_optical_frame"
    assert ros_det.bbox.center.position.x == 150.0
    assert ros_det.bbox.center.position.y == 190.0
    assert ros_det.bbox.size_x == 100.0
    assert ros_det.bbox.size_y == 140.0
    assert len(ros_det.results) == 1
    assert ros_det.results[0].hypothesis.class_id == "warning_bag"
    assert pytest.approx(ros_det.results[0].hypothesis.score, 0.001) == 0.92

