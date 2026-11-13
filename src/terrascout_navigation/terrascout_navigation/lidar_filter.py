#!/usr/bin/env python3
import xml.etree.ElementTree as ET
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import PointCloud2

from fleet_common import signals
from fleet_common.params import declare

SENSOR_DATA_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)

MIN_RANGE_M = 0.1 # Sphere around the LiDAR origin with this radius is excluded from the point cloud.
_LIVOX_TAG_OFFSET = 16

_LIVOX_NOISE_MASK = 0x3F


# Margins around the URDF collision geometry (metres).
# Preserve the existing experimentally determined exclusion regions.
CHASSIS_X_MARGIN = 0.20
CHASSIS_Y_MARGIN = 0.105
CHASSIS_Z_LOWER_MARGIN = 0.155
CHASSIS_Z_UPPER_MARGIN = 0.295

WHEEL_RADIUS_MARGIN = 0.025
WHEEL_WIDTH_MARGIN = 0.0175


def _xyz(origin):
    if origin is None:
        return np.zeros(3)

    return np.fromstring(origin.get("xyz", "0 0 0"), sep=" ")


def _require_unrotated(origin, label):
    rpy = (np.fromstring(origin.get("rpy", "0 0 0"), sep=" ") if origin is not None else np.zeros(3))

    if not np.allclose(rpy, 0.0):
        raise ValueError(f"{label} is rotated; the axis-aligned self-filter needs updating")


def _geometry_from_urdf(urdf_xml):
    """Calculate chassis bounds and wheel cylinders in lidar_link coordinates."""

    robot = ET.fromstring(urdf_xml)
    links = {link.get("name"): link for link in robot.findall("link")}
    joints = {joint.find("child").get("link"): joint for joint in robot.findall("joint")}

    # LiDAR position relative to base_link.
    lidar_joint = joints["lidar_link"]

    if (lidar_joint.get("type") != "fixed" or lidar_joint.find("parent").get("link") != "base_link"):
        raise ValueError("Expected fixed base_link -> lidar_link joint")

    _require_unrotated(lidar_joint.find("origin"), "LiDAR")
    lidar_xyz = _xyz(lidar_joint.find("origin"))

    # Chassis collision box.
    chassis = links["base_link"].find("collision")
    _require_unrotated(chassis.find("origin"), "Chassis collision")

    chassis_center = _xyz(chassis.find("origin")) - lidar_xyz

    chassis_size = np.fromstring(chassis.find("geometry/box").get("size"),sep=" ",)

    half = chassis_size / 2.0

    chassis_bounds = (
        chassis_center[0] - half[0] - CHASSIS_X_MARGIN,
        chassis_center[0] + half[0] + CHASSIS_X_MARGIN,
        chassis_center[1] - half[1] - CHASSIS_Y_MARGIN,
        chassis_center[1] + half[1] + CHASSIS_Y_MARGIN,
        chassis_center[2] - half[2] - CHASSIS_Z_LOWER_MARGIN,
        chassis_center[2] + half[2] + CHASSIS_Z_UPPER_MARGIN,
    )

    # Four wheel collision cylinders.
    wheels = []

    for link_name, joint in joints.items():
        if not link_name.endswith("_wheel_link"):
            continue

        if joint.find("parent").get("link") != "base_link":
            raise ValueError(f"Unexpected wheel parent: {link_name}")

        _require_unrotated(joint.find("origin"), f"{link_name} joint")

        collision = links[link_name].find("collision")
        origin = collision.find("origin")

        # Current TerraScout wheel cylinders have their axes along Y.
        rpy = np.fromstring(origin.get("rpy", "0 0 0"), sep=" ")

        if not np.allclose(rpy, [0.0, np.pi / 2, np.pi / 2], atol=1e-4):
            raise ValueError(f"Unexpected wheel cylinder orientation: {link_name}")

        if not np.allclose(_xyz(origin)[[0, 2]], 0.0):
            raise ValueError(f"Wheel cylinder is not centered on its axle: {link_name}")

        wheel_xyz = (
            _xyz(joint.find("origin"))
            + _xyz(origin)
            - lidar_xyz
        )

        cylinder = collision.find("geometry/cylinder")

        wheels.append((
            wheel_xyz[0],
            wheel_xyz[1],
            wheel_xyz[2],
            float(cylinder.get("radius")) + WHEEL_RADIUS_MARGIN,
            float(cylinder.get("length")) / 2.0 + WHEEL_WIDTH_MARGIN,
        ))

    if len(wheels) != 4:
        raise ValueError(f"Expected four wheel cylinders, got {len(wheels)}")

    return chassis_bounds, wheels


def _valid_mask(
    data: bytes,
    point_step: int,
    min_range_sq: float = 0.25,
    filter_noise_tag: bool = True,
    xyz_offsets: tuple[int, int, int] = (0, 4, 8),
    tag_offset: int | None = _LIVOX_TAG_OFFSET,
    robot_geometry=None, 
) -> np.ndarray:
    n_points = len(data) // point_step if point_step > 0 else 0
    if n_points == 0:
        return np.array([], dtype=bool)

    fields: dict = {
        "names": ["x", "y", "z"],
        "formats": [np.float32, np.float32, np.float32],
        "offsets": list(xyz_offsets),
        "itemsize": point_step,
    }
    has_tag = filter_noise_tag and tag_offset is not None and point_step > tag_offset
    if has_tag:
        fields["names"].append("tag")
        fields["formats"].append(np.uint8)
        fields["offsets"].append(tag_offset)

    pts = np.frombuffer(data, dtype=np.dtype(fields))
    x = pts["x"]
    y = pts["y"]
    z = pts["z"]
 
    # Remove NaN and infinity.
    finite = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
 
    # Remove zero / invalid returns.
    nonzero = (x != 0.0) | (y != 0.0) | (z != 0.0)
 
    # Minimum LiDAR range.
    range_sq = x**2 + y**2 + z**2
    beyond_blind = range_sq > min_range_sq
 
    # Remove robot self-returns using URDF-derived geometry.
    if robot_geometry is None:
        raise ValueError("Robot self-filter geometry has not been configured")

    chassis_bounds, wheels = robot_geometry

    xmin, xmax, ymin, ymax, zmin, zmax = chassis_bounds

    inside_chassis = (
        (x >= xmin)
        & (x <= xmax)
        & (y >= ymin)
        & (y <= ymax)
        & (z >= zmin)
        & (z <= zmax)
    )

    inside_wheels = np.zeros(n_points, dtype=bool)

    for wheel_x, wheel_y, wheel_z, radius, half_width in wheels:
        wheel_section = (
            (x - wheel_x)**2 + (z - wheel_z)**2 <= radius**2
        )

        inside_wheels |= (
            wheel_section
            & (np.abs(y - wheel_y) <= half_width)
        )

    inside_robot = inside_chassis | inside_wheels

    mask = finite & nonzero & beyond_blind & ~inside_robot

    if has_tag:
        mask &= (pts["tag"] & _LIVOX_NOISE_MASK) == 0
    return mask


class LidarFilter(Node):
    def __init__(self):
        super().__init__("lidar_filter")

        self.declare_parameter("robot_description", "")
        urdf_xml = self.get_parameter("robot_description").value

        if not urdf_xml:
            raise RuntimeError("Missing robot_description parameter")

        self._robot_geometry = _geometry_from_urdf(urdf_xml)
        self.get_logger().info(
            f"URDF self-filter loaded: "
            f"chassis={self._robot_geometry[0]}, "
            f"wheels={len(self._robot_geometry[1])}"
        )


        self._filter_noise_tag = declare(
            self,
            "filter_noise_tag",
            rclpy.Parameter.Type.BOOL,
            "Whether to drop points whose per-point tag marks them as noise",
            constraints="true requires the cloud to carry a tag field",
        )
        self.get_logger().info(
            f"min_range={MIN_RANGE_M:.2f}m, filter_noise_tag={self._filter_noise_tag}"
        )
        self.pub = self.create_publisher(PointCloud2, "cloud_out", SENSOR_DATA_QOS)
        self.sub = self.create_subscription(
            PointCloud2,
            "cloud_in",
            self._callback,
            SENSOR_DATA_QOS,
        )

    def _callback(self, msg: PointCloud2):
        n_points = msg.width * msg.height
        if n_points == 0:
            self.pub.publish(msg)
            return

        offsets = {field.name: field.offset for field in msg.fields}
        missing = [name for name in ("x", "y", "z") if name not in offsets]
        if missing:
            raise RuntimeError(
                f"filter lidar cloud failed: cause: the cloud has no field {missing[0]}"
            )
        if self._filter_noise_tag and "tag" not in offsets:
            raise RuntimeError(
                "filter lidar cloud failed: cause: filter_noise_tag is set but the cloud has no tag field"
            )
        rows = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.row_step)
        raw = np.ascontiguousarray(rows[:, : msg.width * msg.point_step]).reshape(
            n_points, msg.point_step
        )
        mask = _valid_mask(
            raw.tobytes(),
            msg.point_step,
            MIN_RANGE_M**2,
            self._filter_noise_tag,
            (offsets["x"], offsets["y"], offsets["z"]),
            offsets.get("tag"),
            self._robot_geometry,
        )
        filtered = raw[mask]

        out = PointCloud2()
        out.header = msg.header
        out.height = 1
        out.width = int(mask.sum())
        out.fields = msg.fields
        out.is_bigendian = msg.is_bigendian
        out.point_step = msg.point_step
        out.row_step = out.width * msg.point_step
        out.data = filtered.tobytes()
        out.is_dense = True
        self.pub.publish(out)


def main(args=None):
    signals.init(args=args)
    node = LidarFilter()
    try:
        rclpy.spin(node)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
