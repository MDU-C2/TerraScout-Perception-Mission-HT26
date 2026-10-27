#!/usr/bin/env python3
"""
ROS 2 YOLOv8 2D Object Detector Node.

Subscribes:
  - Input RGB camera image (sensor_msgs/msg/Image) on /oakd/rgb/image_raw

Inference:
  - YOLOv8 object detection on incoming frames using trained model (best.pt)

Validation & Record Handling:
  - Perception-system validation & ObjectRecord pipeline:
    * Confidence threshold check
    * Geometric bounding box validation (center u, v, size_x, size_y)
    * Configured class assignment vs 'unknown' fallback

Publishes:
  - 2D Bounding boxes & hypotheses (vision_msgs/msg/Detection2DArray) on /oakd/detections_2d
  - (Optional) Annotated debug image (sensor_msgs/msg/Image) on /oakd/detections_image

Logging:
  - ObjectRecord detection logs to JSONL files
"""
import os
import sys
import time
from pathlib import Path
from typing import List, Optional

# Ensure package root is in sys.path when executed directly
_pkg_root = Path(__file__).resolve().parent.parent
if str(_pkg_root) not in sys.path:
    sys.path.insert(0, str(_pkg_root))
_mod_dir = Path(__file__).resolve().parent
if str(_mod_dir) not in sys.path:
    sys.path.insert(0, str(_mod_dir))

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

try:
    from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
except ImportError:
    Detection2DArray = None
    Detection2D = None
    ObjectHypothesisWithPose = None

from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from ultralytics import YOLO
import torch

try:
    from s_perception_2d.object_record import ObjectRecord, ObjectRecordHandler
    from s_perception_2d.perception_logger import PerceptionLogger
except (ImportError, ModuleNotFoundError):
    try:
        from .object_record import ObjectRecord, ObjectRecordHandler
        from .perception_logger import PerceptionLogger
    except (ImportError, ModuleNotFoundError):
        from object_record import ObjectRecord, ObjectRecordHandler
        from perception_logger import PerceptionLogger



class YoloDetectorNode(Node):
    """
    ROS 2 Node performing YOLOv8 2D object detection and ObjectRecord validation.
    """

    def _declare_param(self, name, default):
        return getattr(self, "declare_parameter")(name, default)

    def __init__(self):
        super().__init__("yolo_detector_node")

        # ---------------- Parameters Declaration ----------------
        self._declare_param("model_path", "")
        self._declare_param("confidence_threshold", 0.50)
        self._declare_param("min_noise_threshold", 0.15)
        self._declare_param("iou_threshold", 0.45)
        self._declare_param("input_image_topic", "oakd/rgb/image_raw")
        self._declare_param("output_detections_topic", "oakd/detections_2d")
        self._declare_param("publish_annotated_image", True)
        self._declare_param("annotated_image_topic", "oakd/detections_image")
        self._declare_param(
            "configured_classes",
            ["Apache friendly", "Ka-52 Enemy"],
        )
        self._declare_param("allow_unknown_records", True)
        self._declare_param("enable_file_logging", True)
        self._declare_param("log_dir", "~/.ros/s_perception_2d_logs")
        self._declare_param("device", "auto")  # "auto", "cuda:0", "cpu"
        self._declare_param("inference_imgsz", 640)
        self._declare_param("qos_reliability", "best_effort")  # "best_effort" or "reliable"
        self._declare_param("target_fps", 5.0)  # Processing rate limit in Hz to prevent CPU starvation

        # ---------------- Retrieve Parameters ----------------
        model_path_param = self.get_parameter("model_path").get_parameter_value().string_value
        self.conf_threshold = float(self.get_parameter("confidence_threshold").value)
        self.min_noise_threshold = float(self.get_parameter("min_noise_threshold").value)
        self.iou_threshold = float(self.get_parameter("iou_threshold").value)
        self.input_image_topic = self.get_parameter("input_image_topic").value
        self.output_detections_topic = self.get_parameter("output_detections_topic").value
        self.publish_annotated_image = bool(self.get_parameter("publish_annotated_image").value)
        self.annotated_image_topic = self.get_parameter("annotated_image_topic").value
        self.configured_classes = list(self.get_parameter("configured_classes").value)
        self.allow_unknown_records = bool(self.get_parameter("allow_unknown_records").value)
        self.enable_file_logging = bool(self.get_parameter("enable_file_logging").value)
        self.log_dir = self.get_parameter("log_dir").value
        device_param = self.get_parameter("device").value
        self.inference_imgsz = int(self.get_parameter("inference_imgsz").value)
        qos_rel_param = self.get_parameter("qos_reliability").value.lower()
        self.target_fps = float(self.get_parameter("target_fps").value)
        self._min_frame_interval = 1.0 / self.target_fps if self.target_fps > 0 else 0.0
        self._last_processed_time = 0.0
        self._is_processing = False

        # Resolve compute device
        if device_param == "auto":
            self.device = 0 if torch.cuda.is_available() else "cpu"
        else:
            self.device = device_param

        # Resolve model path
        self.model_path = self._resolve_model_path(model_path_param)
        self.get_logger().info(f"Loading YOLOv8 model from: {self.model_path} (device={self.device})")
        self.model = YOLO(str(self.model_path))

        # Model class dictionary
        self.model_classes = self.model.names if hasattr(self.model, "names") else {}
        self.get_logger().info(f"Model classes: {self.model_classes}")
        self.get_logger().info(f"Configured supported classes: {self.configured_classes}")
        self.get_logger().info(f"Confidence threshold: {self.conf_threshold}, Min noise threshold: {self.min_noise_threshold}")

        # Initialize ObjectRecordHandler & PerceptionLogger
        self.record_handler = ObjectRecordHandler(
            configured_classes=self.configured_classes,
            confidence_threshold=self.conf_threshold,
            min_noise_threshold=self.min_noise_threshold,
            allow_unknown_records=self.allow_unknown_records,
        )
        self.logger_handler = PerceptionLogger(
            log_dir=self.log_dir,
            enable_file_logging=self.enable_file_logging,
        )

        self.bridge = CvBridge()
        self.frame_count = 0

        # Configure QoS profile: Best-effort subscriber receives from both
        # best-effort (e.g. Gazebo ROS bridge) and reliable publishers.
        if qos_rel_param == "reliable":
            reliability = ReliabilityPolicy.RELIABLE
        else:
            reliability = ReliabilityPolicy.BEST_EFFORT

        qos_profile = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=reliability,
            durability=DurabilityPolicy.VOLATILE,
        )

        # ---------------- Subscriptions & Publishers ----------------
        self.sub_image = self.create_subscription(
            Image,
            self.input_image_topic,
            self.image_callback,
            qos_profile,
        )

        self.pub_detections = self.create_publisher(
            Detection2DArray,
            self.output_detections_topic,
            10,
        )

        if self.publish_annotated_image:
            self.pub_annotated = self.create_publisher(
                Image,
                self.annotated_image_topic,
                10,
            )
        else:
            self.pub_annotated = None

        self.get_logger().info(
            f"YoloDetectorNode initialized! Subscribing to '{self.input_image_topic}', "
            f"Publishing to '{self.output_detections_topic}'"
        )

    def _resolve_model_path(self, model_path_str: str) -> Path:
        """Resolve model weights path with fallback locations."""
        if model_path_str:
            p = Path(model_path_str).expanduser()
            if p.exists():
                return p

        # 1. Look in package share directory
        try:
            share_dir = Path(get_package_share_directory("s_perception_2d"))
            p_share = share_dir / "models" / "best.pt"
            if p_share.exists():
                return p_share
        except (PackageNotFoundError, Exception):
            pass

        # 2. Look in workspace source tree
        pkg_src = Path(__file__).resolve().parent.parent / "models" / "best.pt"
        if pkg_src.exists():
            return pkg_src

        # 3. Look in runs_yolo training output
        runs_path = (
            Path(__file__).resolve().parents[3]
            / "runs_yolo"
            / "train"
            / "yolov8n_trained"
            / "weights"
            / "best.pt"
        )
        if runs_path.exists():
            return runs_path

        # 4. Fallback to default yolov8n.pt if best.pt is not yet ready
        return Path("yolov8n.pt")

    def image_callback(self, msg: Image) -> None:
        """
        Callback processing incoming camera image frame.
        Throttled to target_fps to prevent CPU starvation and message queue backpressure.
        """
        now = time.time()
        if self._min_frame_interval > 0:
            if (now - self._last_processed_time) < self._min_frame_interval:
                return  # Throttle: drop frame to maintain target processing rate
        if self._is_processing:
            return  # Drop frame if previous inference is still executing

        self._is_processing = True
        self._last_processed_time = now
        try:
            self._process_image(msg)
        finally:
            self._is_processing = False

    def _process_image(self, msg: Image) -> None:
        self.frame_count += 1
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge conversion error: {e}")
            return

        h, w, _ = cv_image.shape

        # ---------------- Inference ----------------
        try:
            results = self.model.predict(
                source=cv_image,
                conf=self.min_noise_threshold,
                iou=self.iou_threshold,
                imgsz=self.inference_imgsz,
                verbose=False,
                device=self.device,
            )
        except torch.cuda.OutOfMemoryError:
            self.get_logger().warn("CUDA Out of Memory during inference! Falling back to CPU.")
            self.device = "cpu"
            torch.cuda.empty_cache()
            try:
                results = self.model.predict(
                    source=cv_image,
                    conf=self.min_noise_threshold,
                    iou=self.iou_threshold,
                    imgsz=self.inference_imgsz,
                    verbose=False,
                    device="cpu",
                )
            except Exception as e:
                self.get_logger().error(f"Inference error on CPU fallback: {e}")
                return
        except Exception as e:
            self.get_logger().error(f"Inference error: {e}")
            return

        # ---------------- Validation & Record Handling ----------------
        object_records: List[ObjectRecord] = []
        detection_array_msg = Detection2DArray()
        detection_array_msg.header = msg.header

        if results and len(results) > 0:
            result = results[0]
            boxes = result.boxes

            for box in boxes:
                xyxy = box.xyxy[0].cpu().numpy().tolist()
                conf = float(box.conf[0].item())
                cls_id = int(box.cls[0].item())
                raw_class_name = self.model_classes.get(cls_id, str(cls_id))

                record = self.record_handler.validate_and_create_record(
                    raw_class_name=raw_class_name,
                    confidence=conf,
                    xyxy=(xyxy[0], xyxy[1], xyxy[2], xyxy[3]),
                    frame_id=msg.header.frame_id,
                    timestamp=msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                    img_width=w,
                    img_height=h,
                )

                if record.is_valid:
                    object_records.append(record)
                    det_msg = record.to_ros_detection2d(header=msg.header)
                    if det_msg is not None:
                        detection_array_msg.detections.append(det_msg)
                    self.get_logger().info(
                        f"🎯 Detected: '{record.class_name}' | conf={record.confidence:.2f} | "
                        f"center=({record.center_u:.1f}, {record.center_v:.1f}) | "
                        f"bbox=[{record.x1:.0f}, {record.y1:.0f}, {record.x2:.0f}, {record.y2:.0f}]"
                    )

        # ---------------- Publish Detections ----------------
        self.pub_detections.publish(detection_array_msg)

        # ---------------- Log Detection Records ----------------
        stamp_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.logger_handler.log_frame_detections(
            frame_id=msg.header.frame_id,
            timestamp=stamp_time,
            records=object_records,
            image_shape=(h, w),
        )

        # ---------------- Publish Annotated Image (Only if subscribers exist) ----------------
        if self.publish_annotated_image and self.pub_annotated is not None:
            if self.pub_annotated.get_subscription_count() > 0:
                self._publish_annotated_frame(cv_image, object_records, msg.header)

        # Periodic log summary every 100 frames
        if self.frame_count % 100 == 0:
            stats = self.logger_handler.get_stats()
            self.get_logger().info(
                f"[Perception Status] Frames: {self.frame_count}, Total Detections: {stats['total_detections']}, "
                f"Class counts: {stats['configured_classes']}, Unknown: {stats['unknown_records']}"
            )

    def _publish_annotated_frame(
        self,
        cv_image: np.ndarray,
        records: List[ObjectRecord],
        header,
    ) -> None:
        """
        Draw bounding boxes and class labels onto the image and publish.
        """
        vis_img = cv_image.copy()

        # Class color palette (BGR)
        palette = {
            "Apache friendly": (0, 255, 0),   # Green
            "Ka-52 Enemy": (0, 0, 255),       # Red
            "unknown": (180, 180, 180),       # Grey
        }

        for rec in records:
            x1, y1, x2, y2 = int(rec.x1), int(rec.y1), int(rec.x2), int(rec.y2)
            color = palette.get(rec.class_name, (0, 255, 0))

            # Draw bounding box
            cv2.rectangle(vis_img, (x1, y1), (x2, y2), color, 2)

            # Draw center point (u, v)
            cu, cv = int(rec.center_u), int(rec.center_v)
            cv2.drawMarker(vis_img, (cu, cv), color, cv2.MARKER_CROSS, 10, 2)

            # Label text
            label = f"{rec.class_name}: {rec.confidence:.2f}"
            t_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)[0]
            c2 = x1 + t_size[0] + 4, y1 - t_size[1] - 4
            cv2.rectangle(vis_img, (x1, y1), c2, color, -1, cv2.LINE_AA)
            cv2.putText(
                vis_img,
                label,
                (x1 + 2, y1 - 2),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 255, 255),
                1,
                cv2.LINE_AA,
            )

        try:
            annotated_msg = self.bridge.cv2_to_imgmsg(vis_img, encoding="bgr8")
            annotated_msg.header = header
            self.pub_annotated.publish(annotated_msg)
        except CvBridgeError as e:
            self.get_logger().warn(f"Failed to publish annotated image: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, Exception):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
