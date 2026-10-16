"""
ObjectRecord representation and perception validation handler.

Pipeline stage:
RGB image -> YOLOv8n inference -> Detection (raw bbox, class, conf)
          -> Perception-system validation and record handling
          -> ObjectRecord:
             ├── Configured class, when supported
             └── "unknown", when the record cannot be assigned to a configured class
"""

from dataclasses import dataclass, field, asdict
from typing import List, Tuple, Dict, Any, Optional
import time

try:
    from vision_msgs.msg import Detection2D, ObjectHypothesisWithPose
except ImportError:
    # Allows usage outside ROS environment during pure unit tests
    Detection2D = None
    ObjectHypothesisWithPose = None


@dataclass
class ObjectRecord:
    """
    Validated detection record produced by the perception system.
    """
    # Assigned class: either one of the configured supported classes or "unknown"
    class_name: str
    raw_class_name: str
    confidence: float

    # Bounding box coordinates: [x1, y1, x2, y2]
    x1: float
    y1: float
    x2: float
    y2: float

    # Center coordinates (u, v)
    center_u: float
    center_v: float

    # Dimensions (width, height)
    width: float
    height: float

    # Metadata
    frame_id: str = ""
    timestamp: float = field(default_factory=time.time)
    is_valid: bool = True
    validation_status: str = "VALID"

    def to_dict(self) -> Dict[str, Any]:
        """Convert record to dictionary for structured JSON logging."""
        return {
            "timestamp": self.timestamp,
            "frame_id": self.frame_id,
            "class_name": self.class_name,
            "raw_class_name": self.raw_class_name,
            "confidence": round(float(self.confidence), 4),
            "bbox": {
                "x1": round(float(self.x1), 2),
                "y1": round(float(self.y1), 2),
                "x2": round(float(self.x2), 2),
                "y2": round(float(self.y2), 2),
                "center_u": round(float(self.center_u), 2),
                "center_v": round(float(self.center_v), 2),
                "width": round(float(self.width), 2),
                "height": round(float(self.height), 2),
            },
            "is_valid": self.is_valid,
            "validation_status": self.validation_status,
        }

    def to_ros_detection2d(self, header=None) -> Optional[Any]:
        """
        Convert to standard ROS 2 vision_msgs/msg/Detection2D.
        """
        if Detection2D is None:
            raise RuntimeError("vision_msgs is not available in the current environment.")

        detection = Detection2D()
        if header is not None:
            detection.header = header
        else:
            detection.header.frame_id = self.frame_id

        # Fill 2D Bounding Box
        detection.bbox.center.position.x = float(self.center_u)
        detection.bbox.center.position.y = float(self.center_v)
        detection.bbox.center.theta = 0.0
        detection.bbox.size_x = float(self.width)
        detection.bbox.size_y = float(self.height)

        # Fill Hypothesis
        hypothesis = ObjectHypothesisWithPose()
        hypothesis.hypothesis.class_id = str(self.class_name)
        hypothesis.hypothesis.score = float(self.confidence)
        detection.results.append(hypothesis)

        return detection


class ObjectRecordHandler:
    """
    Perception-system validation and record handling.
    Enforces confidence thresholding, geometric bounding box validation,
    and class assignment (configured classes vs unknown fallback).
    """

    DEFAULT_CONFIGURED_CLASSES = [
        "pedestrian_sign",
        "roadwork_sign",
        "stop-sign",
        "warning_bag",
    ]

    def __init__(
        self,
        configured_classes: Optional[List[str]] = None,
        confidence_threshold: float = 0.35,
        min_box_size_px: float = 8.0,
        allow_unknown_records: bool = True,
    ):
        """
        :param configured_classes: List of supported class names.
        :param confidence_threshold: Minimum acceptable confidence score.
        :param min_box_size_px: Minimum width and height for a valid box.
        :param allow_unknown_records: If True, unsupported classes become 'unknown'.
                                      If False, unsupported classes are marked invalid.
        """
        if configured_classes is None:
            self.configured_classes = set(self.DEFAULT_CONFIGURED_CLASSES)
        else:
            self.configured_classes = set(configured_classes)

        self.confidence_threshold = float(confidence_threshold)
        self.min_box_size_px = float(min_box_size_px)
        self.allow_unknown_records = bool(allow_unknown_records)

    def validate_and_create_record(
        self,
        raw_class_name: str,
        confidence: float,
        xyxy: Tuple[float, float, float, float],
        frame_id: str = "",
        timestamp: Optional[float] = None,
        img_width: Optional[int] = None,
        img_height: Optional[int] = None,
    ) -> ObjectRecord:
        """
        Validates raw detection and creates an ObjectRecord.
        """
        x1, y1, x2, y2 = float(xyxy[0]), float(xyxy[1]), float(xyxy[2]), float(xyxy[3])
        conf = float(confidence)
        t = timestamp if timestamp is not None else time.time()

        width = max(0.0, x2 - x1)
        height = max(0.0, y2 - y1)
        center_u = x1 + width / 2.0
        center_v = y1 + height / 2.0

        is_valid = True
        status_reasons = []

        # 1. Confidence validation
        if conf < self.confidence_threshold:
            is_valid = False
            status_reasons.append("LOW_CONFIDENCE")

        # 2. Geometric box validation
        if width < self.min_box_size_px or height < self.min_box_size_px:
            is_valid = False
            status_reasons.append("BOX_TOO_SMALL")

        if img_width is not None and img_height is not None:
            if x2 <= 0 or y2 <= 0 or x1 >= img_width or y1 >= img_height:
                is_valid = False
                status_reasons.append("OUT_OF_BOUNDS")

        # 3. Class assignment: Configured class vs 'unknown'
        assigned_class: str
        if raw_class_name in self.configured_classes:
            assigned_class = raw_class_name
        else:
            assigned_class = "unknown"
            if not self.allow_unknown_records:
                is_valid = False
                status_reasons.append("UNCONFIGURED_CLASS_REJECTED")
            else:
                status_reasons.append("ASSIGNED_UNKNOWN")

        if is_valid and not status_reasons:
            status = "VALID"
        else:
            status = ";".join(status_reasons) if status_reasons else "VALID"

        return ObjectRecord(
            class_name=assigned_class,
            raw_class_name=raw_class_name,
            confidence=conf,
            x1=x1,
            y1=y1,
            x2=x2,
            y2=y2,
            center_u=center_u,
            center_v=center_v,
            width=width,
            height=height,
            frame_id=frame_id,
            timestamp=t,
            is_valid=is_valid,
            validation_status=status,
        )
