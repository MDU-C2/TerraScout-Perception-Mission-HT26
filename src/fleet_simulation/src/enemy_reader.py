#!/usr/bin/env python3
"""YOLO + organized depth -> ENU static-object tracks with visibility evidence."""

from collections import deque
from fleet_interfaces.msg import Enemy, EnemyArray, TrackedObject, TrackedObjectArray
from fleet_interfaces.srv import SetKeepoutZone

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, CameraInfo, Image
from sensor_msgs_py import point_cloud2
from vision_msgs.msg import Detection2DArray
from message_filters import Subscriber, ApproximateTimeSynchronizer
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from image_geometry import PinholeCameraModel
from scipy.spatial.transform import Rotation, Slerp

from simRealDepth import simRealDepth

from object_tracker import ObjectTracker
from fleet_common.geodesy import enu_to_llh
from fleet_common.qos import latched_qos

#This toggles if we use noise or not for the oak-D distance and odometry data
#And it is specifically for noise generated in THIS file
ownOdomNoise = False
ownPointCloudNoise = True

reportCroppedImages = False

enemyClass = 'Ka-52 Enemy'

camera_offset = np.array([0.36, 0.0, 0.55])

#TODO, generate our own YOLO noise, misses and false positives?

class EnemyReader(Node):
    def __init__(self):
        super().__init__('enemy_reader')

        #Patameters:
        self.mad_multiplier = 3.5
        self.min_depth_tolerance = 0.05
        self.min_valid_points = 10
        
        self.tracker = ObjectTracker()
        self.datum = None
        self.keepout_size = 4.0
        self.keepout_update_distance = 1.0
        self._keepout_positions = {}
        self._keepout_pending = set()
        self.odom_history = deque(maxlen=150)
        self.image_history = deque(maxlen=10)
        self.camera_model = None
        self.camera_info_size = None

        self.detection_sub = Subscriber(
            self, Detection2DArray, 'oakd/detections_2d',
            qos_profile=qos_profile_sensor_data,
        )

        #Publishing enemies for Keep-out-zones for Navigation
        self.enemies_pub = self.create_publisher(
            EnemyArray,
            'enemies',
            10
        )

        #Publish tracked objects to operator
        self.objects_pub = self.create_publisher(
            TrackedObjectArray,
            'tracked_objects',
            10
        )
        self.cloud_sub = Subscriber(
            self, PointCloud2, 'oakd/points',
            qos_profile=qos_profile_sensor_data,
        )

        self.sync = ApproximateTimeSynchronizer(
            [self.detection_sub, self.cloud_sub], queue_size=10, slop=0.1, #slop 0.1 means that they can be at most 0.1 seconds away from eachother
        )

        self.odom_sub = self.create_subscription(
            # Datum-aligned ENU pose; local odom cannot be converted to LLH
            # using the GNSS datum for keepout placement.
            Odometry, 'fixposition/odometry_enu',
            self.odom_callback, 10,
        )

        self.camera_info_sub = self.create_subscription(
            CameraInfo, 'oakd/rgb/camera_info',
            self.camera_info_callback, qos_profile_sensor_data,
        )

        if reportCroppedImages:
            self.image_sub = self.create_subscription(
                Image,
                'oakd/rgb/image_raw',
                self.image_callback,
                qos_profile_sensor_data,
            )

        self.sync.registerCallback(self.sensor_callback)
        self.keepout_client = self.create_client(SetKeepoutZone, 'set_keepout_zone')
        self.datum_sub = self.create_subscription(
            NavSatFix, 'fixposition/datum', self.datum_callback, latched_qos(),
        )
        self.create_timer(0.5, self.keepout_timer_callback)
        self.get_logger().info('Enemy reader started (KF + gated association)')

    def datum_callback(self, msg):
        if np.isfinite([msg.latitude, msg.longitude, msg.altitude]).all():
            self.datum = msg

    def keepout_timer_callback(self):
        if self.datum is None or not self.keepout_client.service_is_ready():
            return
        for track in self.tracker.confirmed_tracks():
            if track.obj_class != enemyClass:
                continue
            label = f'enemy_track_{track.id}'
            position = track.position
            previous = self._keepout_positions.get(label)
            if previous is not None and np.linalg.norm(position - previous) < self.keepout_update_distance:
                continue
            if label in self._keepout_pending:
                continue
            latitude, longitude, _ = enu_to_llh(
                float(position[0]), float(position[1]), float(position[2]),
                self.datum.latitude, self.datum.longitude, self.datum.altitude,
            )
            request = SetKeepoutZone.Request()
            request.label = label
            request.clear = False
            request.latitude = latitude
            request.longitude = longitude
            request.x_length = self.keepout_size
            request.y_length = self.keepout_size
            request.yaw_deg = 0.0
            self._keepout_pending.add(label)
            future = self.keepout_client.call_async(request)
            future.add_done_callback(
                lambda completed, zone=label, pos=position.copy(),
                       lat=latitude, lon=longitude:
                    self._keepout_done(completed, zone, pos, lat, lon)
            )

    def _keepout_done(self, future, label, position, latitude, longitude):
        self._keepout_pending.discard(label)
        try:
            response = future.result()
            if response is None or not response.accepted:
                reason = 'no response' if response is None else response.reason
                self.get_logger().warning(f'keepout {label} rejected: {reason}')
                return
            self._keepout_positions[label] = position
            self.get_logger().info(
                f'updated keepout {label} from map ENU '
                f'({position[0]:.2f}, {position[1]:.2f}, {position[2]:.2f}) '
                f'at {latitude:.7f},{longitude:.7f}'
            )
        except Exception as exc:
            self.get_logger().warning(f'keepout {label} request failed: {exc}')

    @staticmethod
    def message_time(stamp):
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    #Getting robot pose and saving pose history
    def odom_callback(self, msg):

        #Position
        p = msg.pose.pose.position
        position = np.array([p.x, p.y, p.z], dtype=float)

        #Orientation
        q = msg.pose.pose.orientation
        quaternion = np.array([q.x, q.y, q.z, q.w], dtype=float)
        if not np.isfinite(quaternion).all() or np.linalg.norm(quaternion) < 0.5:
            return

        if ownOdomNoise:
            position += np.random.normal(0.0, 0.05, size=3)

        self.odom_history.append((
            self.message_time(msg.header.stamp),
            position,
            quaternion,
        ))

    def image_callback(self, msg):
        timestamp = self.message_time(msg.header.stamp)
        self.image_history.append((timestamp, msg))

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

        # Gazebo attaches this sensor to oakd_link, whose axes are base FLU:
        # +X forward, +Y left, +Z up. Do not apply the ROS optical-frame RPY
        # to these raw Gazebo points a second time.
        valid = object_points[np.isfinite(object_points).all(axis=2)]
        valid = valid[valid[:, 0] > 0.0]
 
        if len(valid) == 0:
            return None

        depths = valid[:, 0]

        #Get MAD, get tolerace, then filter
        median_depth = np.median(depths)
        mad = np.median(np.abs(depths - median_depth))
        tolerance = max(self.mad_multiplier * mad, self.min_depth_tolerance)
        filtered = valid[np.abs(depths - median_depth) <= tolerance]

        if len(filtered) < self.min_valid_points:
            return None

        position = np.median(filtered, axis=0)
        
        return position

    #Nested function, the inner project() will be sent to object_tracker
    #And we do not use this file as an include there yet
    def projector(self, robot_position, R_world_robot, width, height):

        #position (@ is matrix multiplication)
        camera_pos = robot_position + R_world_robot @ camera_offset

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
    #Called when a point cloud and YOLO pair is read, at most 0.1 s apart 
    def sensor_callback(self, detections_msg, cloud_msg):

        #Get time
        timestamp = self.message_time(cloud_msg.header.stamp)

        #Find pose in odom history matching time stamp
        pose = self.pose_for_timestamp(timestamp)
        if pose is None:
            return
        p_robot, R_world_robot, pose_fresh = pose

        #Get pointcloud as 3D matrix
        points = point_cloud2.read_points_numpy(
            cloud_msg, 
            field_names=('x', 'y', 'z'), 
            skip_nans=False,
        ).reshape(cloud_msg.height, cloud_msg.width, 3)

        if ownPointCloudNoise:
            points = simRealDepth(points)

        #get image seen by YOLO
        detection_time = self.message_time(detections_msg.header.stamp)

        image_entry = min(
            self.image_history,
            key=lambda item: abs(item[0] - detection_time),
            default=None
            )

        image_msg = None

        if image_entry is not None:
            if abs(image_entry[0] - detection_time) <= 0.001:
                image_msg = image_entry[1]

        observations = [] #What we will send to object_tracker
        image_observation = [] #The yolo observations with insufficient depth information

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

            image_observation.append({
                'class_id': obj_class,
                'bbox_center': np.array([cx, cy], dtype=float),
            })

            # Detection boxes are in RGB image pixels. The organized depth cloud
            # may use a different resolution, so scale coordinates before slicing.
            w, h = detection.bbox.size_x, detection.bbox.size_y
            if self.camera_info_size is None:
                self.get_logger().warn('Waiting for camera info before associating depth')
                continue
            image_width, image_height = self.camera_info_size
            scale_x = cloud_msg.width / image_width
            scale_y = cloud_msg.height / image_height
            x1 = max(0, int((cx - w / 2) * scale_x))
            x2 = min(cloud_msg.width, int(np.ceil((cx + w / 2) * scale_x)))
            y1 = max(0, int((cy - h / 2) * scale_y))
            y2 = min(cloud_msg.height, int(np.ceil((cy + h / 2) * scale_y)))

            #Skip empty boxes
            if x2 <= x1 or y2 <= y1:
                continue

            #Object position and the number of points in the cloud sub space associated with it
            position = self.get_position_from_point_cloud(points[y1:y2, x1:x2])

            #The only way the above function returns none is if we don't have enough valid points
            if position is None:
                self.get_logger().warn(
                    f'Not enough valid depth points for {obj_class} '
                )
                continue

            camera_position_robot = position
            global_position = p_robot + R_world_robot @ (
                camera_position_robot + camera_offset
            )

            print(
                f'PERCEPTION {obj_class}: pixel=({cx:.1f},{cy:.1f}) '
                f'bbox=({x1},{y1})-({x2},{y2}) cloud={cloud_msg.width}x{cloud_msg.height} '
                f'frame={cloud_msg.header.frame_id} cloud_xyz='
                f'({position[0]:.2f},{position[1]:.2f},{position[2]:.2f}) '
                f'robot_enu=({p_robot[0]:.2f},{p_robot[1]:.2f},{p_robot[2]:.2f}) '
                f'camera_offset=({camera_offset[0]:.2f},{camera_offset[1]:.2f},'
                f'{camera_offset[2]:.2f}) map_enu=({global_position[0]:.2f},'
                f'{global_position[1]:.2f},{global_position[2]:.2f})',
                flush=True,
            )

            #Save object observation
            observations.append({
                'class_id': obj_class,
                'position': global_position,
                'bbox_center': np.array([cx, cy], dtype=float),
                'yolo_confidence': yolo_confidence,
                'image': image_msg,
                'bbox' : (x1, y1, x2, y2)
            })

        #Create a function that goes from expected 3D object center to 2D camera image center
        #This will be sent to object tracker as an input
        project = None
        if pose_fresh and self.camera_model is not None and self.camera_info_size is not None:
            image_width, image_height = self.camera_info_size
            project = self.projector(
                p_robot, R_world_robot, image_width, image_height
            )

        #Call the Kalman filter on each track
        #As well was the data association
        tracks = self.tracker.update(
            observations, timestamp, project=project, image_observation=image_observation
        )

        #For logging and ROS2 topic publishing
        confirmed_tracks = [
            track for track in tracks
            if track.status == 'confirmed'
        ]

        #Publish to Navigation
        enemies_msg = EnemyArray()
        for track in confirmed_tracks:
            if track.obj_class == enemyClass:
                enemy = Enemy()
                enemy.id = int(track.id)
                enemy.x = float(track.position[0])
                enemy.y = float(track.position[1])
                enemies_msg.enemies.append(enemy)


        self.enemies_pub.publish(enemies_msg)

        #TODO, everything below needs to be inspected
        objects_msg = TrackedObjectArray()
        objects_msg.header.stamp = cloud_msg.header.stamp
        objects_msg.header.frame_id = 'map'

        for track in tracks:
            obj = TrackedObject()

            obj.id = int(track.id)
            obj.class_id = str(track.obj_class)
            obj.x = float(track.position[0])
            obj.y = float(track.position[1])
            obj.z = float(track.position[2])
            obj.confirmed = track.status == 'confirmed'
            
            # Image
            if track.image is not None:
                image = track.image
                obj.image.encoding = 'rgb8'
                obj.image.height = image.shape[0]
                obj.image.width = image.shape[1]
                obj.image.step = image.shape[1] * 3
                obj.image.is_bigendian = 0
                obj.image.data = image.tobytes()

            objects_msg.objects.append(obj)

        self.objects_pub.publish(objects_msg)

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
