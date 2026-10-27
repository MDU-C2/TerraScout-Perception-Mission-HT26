"""
Perception detection record logger.
Logs ObjectRecord detections in JSONL and CSV formats for perception-system validation,
debugging, and mission record keeping.
"""

import json
import os
from pathlib import Path
from typing import List, Dict, Any, Optional
from datetime import datetime
try:
    from .object_record import ObjectRecord
except (ImportError, ValueError):
    try:
        from s_perception_2d.object_record import ObjectRecord
    except ImportError:
        from object_record import ObjectRecord

class PerceptionLogger:
    """
    Handles file logging of perception ObjectRecords.
    """

    def __init__(self, log_dir: Optional[str] = None, enable_file_logging: bool = True):
        self.enable_file_logging = bool(enable_file_logging)
        if log_dir is None:
            self.log_dir = Path.home() / ".ros" / "s_perception_2d_logs"
        else:
            self.log_dir = Path(log_dir).expanduser()

        self.jsonl_path: Optional[Path] = None
        self.stats = {
            "total_frames": 0,
            "total_detections": 0,
            "configured_classes": {},
            "unknown_records": 0,
            "invalid_records": 0,
        }

        if self.enable_file_logging:
            try:
                self.log_dir.mkdir(parents=True, exist_ok=True)
            except (PermissionError, OSError):
                self.log_dir = Path.home() / ".ros" / "s_perception_2d_logs"
                self.log_dir.mkdir(parents=True, exist_ok=True)
            timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
            self.jsonl_path = self.log_dir / f"detections_{timestamp_str}.jsonl"

    def log_frame_detections(
        self,
        frame_id: str,
        timestamp: float,
        records: List[ObjectRecord],
        image_shape: Optional[tuple] = None,
    ) -> None:
        """
        Record a frame and all its validated ObjectRecords.
        """
        self.stats["total_frames"] += 1
        record_dicts = []

        for r in records:
            self.stats["total_detections"] += 1
            if not r.is_valid:
                self.stats["invalid_records"] += 1
            elif r.class_name == "unknown":
                self.stats["unknown_records"] += 1
            else:
                self.stats["configured_classes"][r.class_name] = (
                    self.stats["configured_classes"].get(r.class_name, 0) + 1
                )
            record_dicts.append(r.to_dict())

        if self.enable_file_logging and self.jsonl_path is not None:
            entry = {
                "frame_id": frame_id,
                "timestamp": timestamp,
                "num_detections": len(records),
                "image_shape": image_shape,
                "records": record_dicts,
            }
            try:
                with open(self.jsonl_path, "a", encoding="utf-8") as f:
                    f.write(json.dumps(entry) + "\n")
            except IOError as e:
                # Avoid crashing node if log disk write fails
                pass

    def get_stats(self) -> Dict[str, Any]:
        """Return runtime perception statistics."""
        return dict(self.stats)
