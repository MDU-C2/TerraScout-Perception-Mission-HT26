#!/usr/bin/env python3

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from vision_msgs.msg import Detection2DArray

from message_filters import Subscriber, ApproximateTimeSynchronizer
from nav_msgs.msg import Odometry

from object_tracker import ObjectTracker


class EnemyReader(Node):

    def __init__(self):
        super().__init__('enemy_reader')

        #Parameters for depth estimation
        self.mad_multiplier = 3.5
        self.min_depth_tolerance = 0.05
        self.min_valid_points = 10

        #Camera pose
        self.camera_offset = np.array([0.36, 0.0, 0.55])

        self.tracker = ObjectTracker(
            max_distance=0.5,
            min_observations=5
        )
        self.robot_pose = None

        #Subscribe to YOLO
        self.detection_sub = Subscriber(
            self,
            Detection2DArray,
            '/terrascout1/oakd/detections_2d',
            qos_profile=qos_profile_sensor_data
        )

        #Point cloud
        self.cloud_sub = Subscriber(
            self,
            PointCloud2,
            '/terrascout1/oakd/points',
            qos_profile=qos_profile_sensor_data
        )

        #Pair yolo and point cloud, timewise
        self.sync = ApproximateTimeSynchronizer(
            [self.detection_sub, self.cloud_sub],
            queue_size=10,
            slop=0.1
        )

        #Our absolute position
        self.odom_sub = self.create_subscription(
            Odometry,
            '/terrascout1/fixposition/odometry_enu',
            self.odom_callback,
            10
        )

        self.sync.registerCallback(self.sensor_callback)
        self.get_logger().info('Enemy reader started')

    def odom_callback(self, msg):
        self.robot_pose = msg.pose.pose

    def estimate_camera_position(self, object_points):

        valid_points = object_points[np.isfinite(object_points).all(axis=2)]

        valid_points = valid_points[valid_points[:, 0] > 0.0]
        if len(valid_points) < self.min_valid_points:
            return None, 0

        depths = valid_points[:, 0]
        median_depth = np.median(depths)
        mad = np.median(np.abs(depths - median_depth))

        tolerance = max(
            self.mad_multiplier * mad,
            self.min_depth_tolerance
        )

        keep = np.abs(depths - median_depth) <= tolerance
        filtered_points = valid_points[keep]

        if len(filtered_points) < self.min_valid_points:
            return None, len(filtered_points)

        position = np.median(filtered_points, axis=0)
        return position, len(filtered_points)

    def calculate_global_position(self, x, y, z):
        if self.robot_pose is None:
            return None

        p_robot = np.array([
            self.robot_pose.position.x,
            self.robot_pose.position.y,
            self.robot_pose.position.z
        ])

        #x forward, y sideways, z up
        p_camera = np.array([x, y, z])

        q = self.robot_pose.orientation
        qx, qy, qz, qw = q.x, q.y, q.z, q.w

        R = np.array([
            [1 - 2 * (qy*qy + qz*qz), 2 * (qx*qy - qz*qw), 2 * (qx*qz + qy*qw)],
            [2 * (qx*qy + qz*qw), 1 - 2 * (qx*qx + qz*qz), 2 * (qy*qz - qx*qw)],
            [2 * (qx*qz - qy*qw), 2 * (qy*qz + qx*qw), 1 - 2 * (qx*qx + qy*qy)]
        ])

        return p_robot + R @ (p_camera + self.camera_offset)

    def sensor_callback(self, detections_msg, cloud_msg):
        if detections_msg.header.frame_id != cloud_msg.header.frame_id:
            self.get_logger().warn('Detection and point cloud frames do not match')
            return

        if cloud_msg.height != 720 or cloud_msg.width != 1280:
            self.get_logger().warn('Unexpected point cloud dimensions')
            return

        observations = []

        points = point_cloud2.read_points_numpy(
            cloud_msg,
            field_names=('x', 'y', 'z'),
            skip_nans=False
        )
        points = points.reshape(cloud_msg.height, cloud_msg.width, 3)

        for detection in detections_msg.detections:
            if not detection.results:
                continue

            obj_class = detection.results[0].hypothesis.class_id
            yolo_confidence = detection.results[0].hypothesis.score

            cx = detection.bbox.center.position.x
            cy = detection.bbox.center.position.y
            w = detection.bbox.size_x
            h = detection.bbox.size_y

            x1 = max(0, int(cx - w / 2))
            x2 = min(cloud_msg.width, int(cx + w / 2))
            y1 = max(0, int(cy - h / 2))
            y2 = min(cloud_msg.height, int(cy + h / 2))

            if x2 <= x1 or y2 <= y1:
                continue

            object_points = points[y1:y2, x1:x2]
            position, retained_points = self.estimate_camera_position(object_points)

            if position is None:
                self.get_logger().warn(
                    f'Not enough valid depth points for {obj_class} '
                    f'after MAD filtering ({retained_points})'
                )
                continue

            global_position = self.calculate_global_position(*position)
            if global_position is None:
                continue

            observations.append({
                'class_id': obj_class,
                'position': np.array(global_position, dtype=float),
                'yolo_confidence': yolo_confidence,
            })

        timestamp = (cloud_msg.header.stamp.sec + cloud_msg.header.stamp.nanosec * 1e-9)

        tracks = self.tracker.update(observations, timestamp)

        for track in tracks:
            if track.status != 'confirmed':
                continue

            tx, ty, tz = track.position

            self.get_logger().info(
                f'ID={track.id} | '
                f'{track.obj_class} | '
                f'Global: ({tx:.2f}, {ty:.2f}, {tz:.2f}) | '
                f'Observations={track.observations} | '
                f'Status={track.status}'
            )
        print(flush=True)
        

def main(args=None):
    rclpy.init(args=args)
    node = EnemyReader()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
