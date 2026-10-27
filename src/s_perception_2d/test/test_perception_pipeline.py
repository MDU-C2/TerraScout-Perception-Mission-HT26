"""
Unit tests for PerceptionLogger and end-to-end record logging.
"""

import json
from pathlib import Path
import pytest

from s_perception_2d.object_record import ObjectRecord, ObjectRecordHandler
from s_perception_2d.perception_logger import PerceptionLogger


def test_perception_logger(tmp_path: Path):
    logger = PerceptionLogger(log_dir=str(tmp_path), enable_file_logging=True)

    handler = ObjectRecordHandler(
        configured_classes=["Apache friendly", "Ka-52 Enemy"],
        confidence_threshold=0.50,
        min_noise_threshold=0.15,
    )

    rec1 = handler.validate_and_create_record("Apache friendly", 0.90, (10, 10, 50, 50), "frame_1")
    rec2 = handler.validate_and_create_record("unknown_thing", 0.80, (20, 20, 60, 60), "frame_1")
    rec3 = handler.validate_and_create_record("Apache friendly", 0.10, (5, 5, 15, 15), "frame_1")  # Invalid (noise floor)

    logger.log_frame_detections(
        frame_id="frame_1",
        timestamp=1000.0,
        records=[rec1, rec2, rec3],
        image_shape=(480, 640),
    )

    stats = logger.get_stats()
    assert stats["total_frames"] == 1
    assert stats["total_detections"] == 3
    assert stats["configured_classes"]["Apache friendly"] == 1
    assert stats["unknown_records"] == 1
    assert stats["invalid_records"] == 1

    # Check written JSONL file
    assert logger.jsonl_path.exists()
    with open(logger.jsonl_path, "r", encoding="utf-8") as f:
        lines = f.readlines()
        assert len(lines) == 1
        entry = json.loads(lines[0])
        assert entry["frame_id"] == "frame_1"
        assert entry["num_detections"] == 3
        assert len(entry["records"]) == 3
        assert entry["records"][0]["class_name"] == "Apache friendly"
        assert entry["records"][1]["class_name"] == "unknown"
