"""
s_perception_2d package.
Provides YOLOv8 2D object detection, validation, ObjectRecord handling, and ROS 2 publishing.
"""

from .object_record import ObjectRecord, ObjectRecordHandler

__all__ = ["ObjectRecord", "ObjectRecordHandler"]
