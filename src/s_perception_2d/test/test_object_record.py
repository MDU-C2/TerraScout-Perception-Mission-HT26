import pytest
from s_perception_2d.object_record import ObjectRecord, ObjectRecordHandler


def test_configured_class_assignment():
    handler = ObjectRecordHandler(
        configured_classes=["Apache friendly", "Ka-52 Enemy"],
        confidence_threshold=0.50,
        max_aspect_ratio_h_w=1.3,
    )

    record = handler.validate_and_create_record(
        raw_class_name="Apache friendly",
        confidence=0.88,
        xyxy=(100.0, 150.0, 400.0, 350.0),
        frame_id="oakd_rgb_camera_optical_frame",
    )

    assert record.is_valid is True
    assert record.class_name == "Apache friendly"
    assert record.raw_class_name == "Apache friendly"
    assert record.confidence == 0.88
    assert record.x1 == 100.0
    assert record.y1 == 150.0
    assert record.x2 == 400.0
    assert record.y2 == 350.0
    assert record.center_u == 250.0
    assert record.center_v == 250.0
    assert record.width == 300.0
    assert record.height == 200.0
    assert record.validation_status == "VALID"


def test_unknown_fallback_for_unsupported_class():
    handler = ObjectRecordHandler(
        configured_classes=["Apache friendly", "Ka-52 Enemy"],
        confidence_threshold=0.50,
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


def test_confidence_rejection():
    handler = ObjectRecordHandler(
        configured_classes=["Apache friendly", "Ka-52 Enemy"],
        confidence_threshold=0.50,
        max_aspect_ratio_h_w=1.3,
    )

    # 1. Low confidence (< 0.50) is rejected as noise
    rec_low = handler.validate_and_create_record(
        raw_class_name="Apache friendly",
        confidence=0.30,
        xyxy=(10.0, 10.0, 100.0, 80.0),
    )
    assert rec_low.is_valid is False
    assert "LOW_CONFIDENCE" in rec_low.validation_status

    # 2. Acceptable confidence
    rec_ok = handler.validate_and_create_record(
        raw_class_name="Apache friendly",
        confidence=0.55,
        xyxy=(10.0, 10.0, 100.0, 80.0),
    )
    assert rec_ok.is_valid is True
    assert rec_ok.class_name == "Apache friendly"
    assert rec_ok.validation_status == "VALID"


def test_box_size_validation():
    handler = ObjectRecordHandler(
        configured_classes=["Apache friendly", "Ka-52 Enemy"],
        min_box_size_px=10.0,
    )

    record = handler.validate_and_create_record(
        raw_class_name="Apache friendly",
        confidence=0.90,
        xyxy=(10.0, 10.0, 12.0, 12.0),  # 2x2 box, smaller than 10px
    )

    assert record.is_valid is False
    assert "BOX_TOO_SMALL" in record.validation_status


def test_to_dict_serialization():
    record = ObjectRecord(
        class_name="Apache friendly",
        raw_class_name="Apache friendly",
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
    assert d["class_name"] == "Apache friendly"
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
        class_name="Ka-52 Enemy",
        raw_class_name="Ka-52 Enemy",
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
    assert ros_det.results[0].hypothesis.class_id == "Ka-52 Enemy"
    assert pytest.approx(ros_det.results[0].hypothesis.score, 0.001) == 0.92


def test_aspect_ratio_rejection():
    handler = ObjectRecordHandler(
        configured_classes=["Apache friendly", "Ka-52 Enemy"],
        confidence_threshold=0.50,
        max_aspect_ratio_h_w=1.3,
    )

    # Vertical box (e.g. tree trunk: width=50, height=150 -> ratio=3.0 > 1.3)
    rec_vertical = handler.validate_and_create_record(
        raw_class_name="Ka-52 Enemy",
        confidence=0.85,
        xyxy=(100.0, 100.0, 150.0, 250.0),
    )
    assert rec_vertical.is_valid is False
    assert "ASPECT_RATIO_REJECTED" in rec_vertical.validation_status

    # Horizontal box
    rec_horizontal = handler.validate_and_create_record(
        raw_class_name="Ka-52 Enemy",
        confidence=0.85,
        xyxy=(100.0, 100.0, 250.0, 200.0),
    )
    assert rec_horizontal.is_valid is True
    assert rec_horizontal.class_name == "Ka-52 Enemy"
    assert rec_horizontal.validation_status == "VALID"
