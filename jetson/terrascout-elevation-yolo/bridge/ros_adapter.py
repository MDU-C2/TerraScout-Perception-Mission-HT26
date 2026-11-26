#!/usr/bin/env python3
"""ROS 2 Jazzy adapter for a Python 3.10 CUDA YOLO subprocess.

Runs in the mapping container (Python 3.12). Keeps the original Image
header, bounding-box conventions, and ObjectRecord validation logic.
Uses one queued frame, dropping older messages when inference is slower
than image arrivals. Camera transport throttling must be done upstream.
"""
import json
import os
import queue
import socket
import struct
import threading
import time

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
from s_perception_2d.object_record import ObjectRecordHandler
from s_perception_2d.perception_logger import PerceptionLogger

MAX_REPLY_BYTES = 2 * 1024 * 1024


def read_exact(conn, length):
    data = bytearray()
    while len(data) < length:
        chunk = conn.recv(length - len(data))
        if not chunk:
            raise EOFError('YOLO connection closed')
        data.extend(chunk)
    return bytes(data)


def env_bool(name, default=False):
    return os.environ.get(name, str(default)).strip().lower() in ('true', '1', 'yes')


class YoloRosAdapter(Node):
    def __init__(self):
        super().__init__('yolo_detector_node')
        self.bridge = CvBridge()
        self.handler = ObjectRecordHandler(
            configured_classes=['Apache friendly', 'Ka-52 Enemy'],
            confidence_threshold=float(
                os.environ.get('YOLO_CONFIDENCE_THRESHOLD', '0.35')
            ),
            max_aspect_ratio_h_w=float(
                os.environ.get('YOLO_MAX_ASPECT_RATIO_H_W', '1.5')
            ),
            allow_unknown_records=True,
        )
        self.perception_logger = PerceptionLogger(
            log_dir=os.environ.get('YOLO_LOG_DIR', '/data/perception-logs'),
            enable_file_logging=env_bool('YOLO_ENABLE_FILE_LOGGING', True),
        )
        self.port = int(os.environ.get('YOLO_PORT', '58751'))
        self.rate = float(os.environ.get('YOLO_TARGET_FPS', '5.0'))
        self.last_accepted = float('-inf')
        self.frames = queue.Queue(maxsize=1)
        self.stop = threading.Event()
        self.publish_annotated = env_bool('YOLO_PUBLISH_ANNOTATED', False)
        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                         reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
        self.sub = self.create_subscription(
            Image, os.environ.get('YOLO_INPUT_TOPIC', '/terrascout1/oakd/rgb/image_raw'),
            self.on_image, qos)
        # Reliable publisher is compatible with best-effort enemy_reader subscribers.
        self.pub = self.create_publisher(
            Detection2DArray,
            os.environ.get('YOLO_OUTPUT_TOPIC', '/terrascout1/oakd/detections_2d'), 10)
        self.debug_pub = None
        if self.publish_annotated:
            self.debug_pub = self.create_publisher(
                Image, os.environ.get('YOLO_ANNOTATED_TOPIC', '/terrascout1/oakd/detections_image'), 1)
        self.worker = threading.Thread(target=self.process_frames, daemon=True)
        self.worker.start()
        self.get_logger().info(f'YOLO ROS adapter started, server=127.0.0.1:{self.port}')

    def on_image(self, msg):
        now = time.monotonic()
        if self.rate > 0 and now - self.last_accepted < 1.0 / self.rate:
            return
        self.last_accepted = now
        try:
            if self.frames.full():
                self.frames.get_nowait()
            self.frames.put_nowait(msg)
        except queue.Empty:
            pass
        except queue.Full:
            pass

    def process_frames(self):
        while not self.stop.is_set():
            try:
                conn = socket.create_connection(('127.0.0.1', self.port), timeout=3.0)
                conn.settimeout(10.0)
                self.get_logger().info('Connected to CUDA YOLO inference service')
                with conn:
                    while not self.stop.is_set():
                        try:
                            msg = self.frames.get(timeout=0.5)
                        except queue.Empty:
                            continue
                        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                        h, w = image.shape[:2]
                        ok, jpeg = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, 80])
                        if not ok:
                            raise RuntimeError('JPEG encoding failed')
                        buf = jpeg.tobytes()
                        conn.sendall(struct.pack('!I', len(buf)) + buf)
                        length, = struct.unpack('!I', read_exact(conn, 4))
                        if not 0 < length < MAX_REPLY_BYTES:
                            raise RuntimeError(f'Bad YOLO reply size: {length}')
                        raw = json.loads(read_exact(conn, length))
                        out = Detection2DArray()
                        out.header = msg.header
                        valid = []
                        for item in raw:
                            record = self.handler.validate_and_create_record(
                                raw_class_name=item['class_name'],
                                confidence=item['confidence'],
                                xyxy=item['xyxy'],
                                frame_id=msg.header.frame_id,
                                timestamp=msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                                img_width=w, img_height=h,
                            )
                            if record.is_valid:
                                det = record.to_ros_detection2d(header=msg.header)
                                out.detections.append(det)
                                valid.append(record)
                        self.pub.publish(out)
                        self.perception_logger.log_frame_detections(
                            frame_id=msg.header.frame_id,
                            timestamp=msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                            records=valid, image_shape=(h, w),
                        )
                        if self.debug_pub is not None and self.debug_pub.get_subscription_count() > 0:
                            annotated = image.copy()
                            for record in valid:
                                p1 = (int(record.x1), int(record.y1))
                                p2 = (int(record.x2), int(record.y2))
                                cv2.rectangle(annotated, p1, p2, (0, 255, 0), 2)
                                cv2.putText(annotated, f'{record.class_name}: {record.confidence:.2f}',
                                            (p1[0], max(12, p1[1] - 5)), cv2.FONT_HERSHEY_SIMPLEX,
                                            0.5, (0, 255, 0), 1)
                            vis = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
                            vis.header = msg.header
                            self.debug_pub.publish(vis)
            except (OSError, EOFError, RuntimeError, ValueError, KeyError, TypeError) as exc:
                if not self.stop.is_set():
                    self.get_logger().warning(f'YOLO worker unavailable / processing error: {exc}; retrying')
                    self.stop.wait(2.0)
            except Exception as exc:
                if not self.stop.is_set():
                    self.get_logger().error(f'Unexpected inference-adapter failure: {exc}; retrying')
                    self.stop.wait(2.0)

    def close(self):
        self.stop.set()
        self.worker.join(timeout=2.0)


def main():
    rclpy.init()
    node = YoloRosAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
