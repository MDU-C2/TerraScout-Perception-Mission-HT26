#!/usr/bin/env python3
"""YOLO + organized depth -> ENU static-object tracks with visibility evidence."""

from collections import deque
from fleet_interfaces.msg import Enemy, EnemyArray

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, CameraInfo
from sensor_msgs_py import point_cloud2
from vision_msgs.msg import Detection2DArray
from message_filters import Subscriber, ApproximateTimeSynchronizer
from nav_msgs.msg import Odometry
from image_geometry import PinholeCameraModel
from scipy.spatial.transform import Rotation, Slerp

from object_tracker import ObjectTracker

#TODO, yolo confidence is still not in use

#This toggles if we use noise or not for the oak-D distance and odometry data
#And it is specifically for noise generated in THIS file
simulating = True

#TODO, generate our own YOLO noise, misses and false positives?

class EnemyReader(Node):
    def __init__(self):
        super().__init__('enemy_reader')

        #Patameters:
        self.mad_multiplier = 3.5
        self.min_depth_tolerance = 0.05
        self.min_valid_points = 10

        self.camera_offset = np.array([0.36, 0.0, 0.55])
        self.tracker = ObjectTracker(min_observations=5)
        self.odom_history = deque(maxlen=150)
        self.camera_model = None
        self.camera_info_size = None
        self._warned_camera_info = False
        self._warned_odom = False

        self.detection_sub = Subscriber(
            self, Detection2DArray, '/terrascout1/oakd/detections_2d',
            qos_profile=qos_profile_sensor_data,
        )

        #Publishing enemies for Keep-out-zones for Navigation
        self.enemies_pub = self.create_publisher(
            EnemyArray,
            '/terrascout1/enemies',
            10
        )

        self.cloud_sub = Subscriber(
            self, PointCloud2, '/terrascout1/oakd/points',
            qos_profile=qos_profile_sensor_data,
        )

        self.sync = ApproximateTimeSynchronizer(
            [self.detection_sub, self.cloud_sub], queue_size=10, slop=0.1,
        )

        self.odom_sub = self.create_subscription(
            Odometry, '/terrascout1/fixposition/odometry_enu',
            self.odom_callback, 10,
        )

        self.camera_info_sub = self.create_subscription(
            CameraInfo, '/terrascout1/oakd/rgb/camera_info',
            self.camera_info_callback, qos_profile_sensor_data,
        )

        self.sync.registerCallback(self.sensor_callback)
        self.get_logger().info('Enemy reader started (KF + gated association)')

    @staticmethod
    def message_time(stamp):
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    #Getting robot pose and saving pose history
    def odom_callback(self, msg):

        #Position
        p = msg.pose.pose.position

        #Orientation
        q = msg.pose.pose.orientation
        quaternion = np.array([q.x, q.y, q.z, q.w], dtype=float)
        if not np.isfinite(quaternion).all() or np.linalg.norm(quaternion) < 0.5:
            return

        if simulating:
            x, y, z = np.random.normal(0, 0.05, size = 3) #TODO, this is just randomly chosen 5cm SD, adjust

        self.odom_history.append((
            self.message_time(msg.header.stamp),
            np.array([p.x, p.y, p.z], dtype=float),
            quaternion,
        ))

    #Get camera width, height and model
    def camera_info_callback(self, msg):
        if (msg.width <= 0 or msg.height <= 0
                or msg.k[0] <= 0 or msg.k[4] <= 0
                or msg.p[0] <= 0 or msg.p[5] <= 0):
            return
        model = PinholeCameraModel()
        model.fromCameraInfo(msg)
        self.camera_model = model
        self.camera_info_size = (msg.width, msg.height)

    #Robots pose at the time of cameras timestamp
    #Returns global robot position, rotation and if the reading was new enough
    #The YOLO detection will very likely lag behind, thus we can't return the most recent odom value
    def pose_for_timestamp(self, timestamp):
        if not self.odom_history:
            return None

        history = list(self.odom_history)

        #A is history starting at 0
        #B is history starting at 1
        for a, b in zip(history, history[1:]):
            t0, p0, q0 = a
            t1, p1, q1 = b

            #Is time stamp inbetween two close time points?
            if t0 <= timestamp <= t1 and 0.0 < t1 - t0 <= 0.35:
                fraction = (timestamp - t0) / (t1 - t0)
                p = (1.0 - fraction) * p0 + fraction * p1

                #Slerp is used to get "middle" rotation from two rotations
                rotation = Slerp([t0, t1], Rotation.from_quat([q0, q1]))([timestamp])[0]

                return p, rotation.as_matrix(), True

        #If there were no good pair, pick the closest in time recorded transform
        t, p, q  = min(history, key=lambda item: abs(item[0] - timestamp))

        age = abs(t - timestamp)
        if age > 0.30:
            return None

        return p, Rotation.from_quat(q).as_matrix(), age <= 0.20

    def get_position_from_point_cloud(self, object_points):

        #Ensure x, y, z are finite and x is positve
        valid = object_points[np.isfinite(object_points).all(axis=2)]
        valid = valid[valid[:, 0] > 0.0]

        #X is depth
        depths = valid[:, 0]

        #Get MAD, get tolerace, then filter
        median_depth = np.median(depths)
        mad = np.median(np.abs(depths - median_depth))
        tolerance = max(self.mad_multiplier * mad, self.min_depth_tolerance)
        filtered = valid[np.abs(depths - median_depth) <= tolerance]

        if len(filtered) < self.min_valid_points:
            return None, len(filtered)

        position = np.median(filtered, axis=0)

        if simulating:
            position[0] += np.random.normal(0, 0.01 * position[0]**2)#mean, std, in meters

        return position, len(filtered)

    #Cameras depth uncertainty increases with the distance to the target
    #TODO, modify according to the datasheet!
    def measurement_covariance(self, depth, R_world_robot):
        d = max(0.2, float(depth))

        #TODO, UNMOTIVATED
        std_depth = 0.15 + 0.010 * d**2
        std_side = np.hypot(0.15 + 0.020 * d, 0.030 * d)
        std_up = np.hypot(0.12 + 0.015 * d, 0.020 * d)
        #Hypotenuse

        #Diagonal covariance matrix
        R_camera = np.diag([std_depth**2, std_side**2, std_up**2])

        #Transform to global coordinates and return
        return R_world_robot @ R_camera @ R_world_robot.T + np.eye(3) * 0.08**2

    #Nested function, the inner project() will be sent to object_tracker
    #And we do not use this file as an include there yet
    def projector(self, robot_position, R_world_robot, width, height):

        #position (@ is matrix multiplication)
        camera_pos = robot_position + R_world_robot @ self.camera_offset

        camera_rot = R_world_robot.T

        #Guess 2D camera image pos from 3D objects position
        def project(world_point):

            x, y, z = camera_rot @ (world_point - camera_pos)

            if x <= 0.0:
                return None

            #Calling camera models function with its own coordinate system
            u, v = self.camera_model.project3dToPixel((-y, -z, x))

            if not np.isfinite([u, v]).all():
                return None

            return {'uv': (float(u), float(v)),
                    'forward_depth': float(x),
                    'image_size': (width, height)}

        return project

    #Updates estimate and publishes it
    def sensor_callback(self, detections_msg, cloud_msg):

        #Error handling
        if detections_msg.header.frame_id != cloud_msg.header.frame_id:
            self.get_logger().warn('Detection and point cloud frames do not match')
            return
        if cloud_msg.height != 720 or cloud_msg.width != 1280:
            self.get_logger().warn('Unexpected point cloud dimensions')
            return

        #Get time
        timestamp = self.message_time(cloud_msg.header.stamp)

        #Error handling
        pose = self.pose_for_timestamp(timestamp)
        if pose is None:
            if not self._warned_odom:
                self.get_logger().warn('No odometry within 0.30 s of cloud: skipping frame')
                self._warned_odom = True
            return
        
        #get robot transform
        p_robot, R_world_robot, pose_fresh = pose

        #Error handling
        if not pose_fresh and not self._warned_odom:
            self.get_logger().warn('No time-aligned odometry: visibility penalties disabled')
            self._warned_odom = True
        if self.camera_model is None and not self._warned_camera_info:
            self.get_logger().warn('Waiting for RGB CameraInfo: visibility penalties disabled')
            self._warned_camera_info = True

        observations = []
        raw_detections = []

        #Get pointcloud as 3D matrix
        points = point_cloud2.read_points_numpy(
            cloud_msg, 
            field_names=('x', 'y', 'z'), 
            skip_nans=False,
        ).reshape(cloud_msg.height, cloud_msg.width, 3)


        #Loop through all YOLO detections
        #To save the objects global positions 
        #and save our observations
        for detection in detections_msg.detections:

            #Not sure if this can happen
            if not detection.results:
                continue

            #Get object class, confidence and center
            obj_class = detection.results[0].hypothesis.class_id
            yolo_confidence = detection.results[0].hypothesis.score
            cx, cy = detection.bbox.center.position.x, detection.bbox.center.position.y

            #Save YOLO observation, will be used in object_tracker
            raw_detections.append({
                'class_id': obj_class,
                'bbox_center': np.array([cx, cy], dtype=float),
            })

            #Get bounding box corners
            w, h = detection.bbox.size_x, detection.bbox.size_y
            x1 = max(0, int(cx - w / 2))
            x2 = min(cloud_msg.width, int(cx + w / 2))
            y1 = max(0, int(cy - h / 2))
            y2 = min(cloud_msg.height, int(cy + h / 2))

            #Skip empty boxes
            if x2 <= x1 or y2 <= y1:
                continue

            #Object position and the number of points in the cloud sub space associated with it
            position, retained_points = self.get_position_from_point_cloud(points[y1:y2, x1:x2])

            #The only way the above function returns none is if we don't have enough valid points
            if position is None:
                self.get_logger().warn(
                    f'Not enough valid depth points for {obj_class} '
                    f'after MAD filtering ({retained_points})'
                )
                continue

            global_position = p_robot + R_world_robot @ (position + self.camera_offset)

            #Save object observation
            observations.append({
                'class_id': obj_class,
                'position': global_position,
                'covariance': self.measurement_covariance(position[0], R_world_robot),
                'bbox_center': np.array([cx, cy], dtype=float),
                'yolo_confidence': yolo_confidence,
            })

        #Create a function that goes from expected 3D object center to 2D camera image center
        #This will be sent to object tracker as an input
        project = (self.projector(p_robot, R_world_robot, cloud_msg.width, cloud_msg.height)
                   if pose_fresh else None)

        #Call the Kalman filter on each track
        #As well was the data association
        tracks = self.tracker.update(
            observations, timestamp, project=project, raw_detections=raw_detections
        )

        #For logging and ROS2 topic publishing
        confirmed_tracks = [
            track for track in tracks
            if track.status == 'confirmed'
        ]

        #Publish to Navigation
        enemies_msg = EnemyArray()
        for track in confirmed_tracks:
            enemy = Enemy()
            enemy.id = int(track.id)
            enemy.x = float(track.position[0])
            enemy.y = float(track.position[1])
            enemies_msg.enemies.append(enemy)

        self.enemies_pub.publish(enemies_msg)

        #Log
        for track in confirmed_tracks:
            tx, ty, tz = track.position
            self.get_logger().info(
                f'ID={track.id} | '
                f'{track.obj_class} | '
                f'Global: ({tx:.2f}, {ty:.2f}, {tz:.2f}) | '
                f'Observations={track.observations}'
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

main()
