import math
import xml.etree.ElementTree as ET

IDENTITY = (
    (1.0, 0.0, 0.0),
    (0.0, 1.0, 0.0),
    (0.0, 0.0, 1.0),
)

def matmul(a, b):
    return tuple(
        tuple(sum(a[i][k] * b[k][j] for k in range(3))
              for j in range(3))
        for i in range(3)
    )

def matvec(a, v):
    return tuple(
        sum(a[i][j] * v[j] for j in range(3))
        for i in range(3)
    )

def transpose(a):
    return tuple(zip(*a))

def rpy_matrix(roll, pitch, yaw):
    """URDF roll-pitch-yaw rotation: Rz(yaw) Ry(pitch) Rx(roll)."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)

    return (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    )

def urdf_extrinsics(xml, target_frame, source_frame):
    """Return source-frame pose expressed in target_frame."""
    robot = ET.fromstring(xml)
    parents = {}

    for joint in robot.findall("joint"):
        child = joint.find("child").get("link")
        parent = joint.find("parent").get("link")
        origin = joint.find("origin")

        xyz = tuple(map(float, origin.get("xyz", "0 0 0").split())) if origin is not None else (0.0, 0.0, 0.0)
        rpy = tuple(map(float, origin.get("rpy", "0 0 0").split())) if origin is not None else (0.0, 0.0, 0.0)

        parents[child] = (
            parent, joint.get("type"), xyz, rpy_matrix(*rpy)
        )

    def pose(link):
        if link not in parents:
            return link, (0.0, 0.0, 0.0), IDENTITY

        parent, joint_type, t, R = parents[link]

        if joint_type != "fixed":
            raise ValueError(f"Non-fixed joint in path to {link}")

        root, p, Q = pose(parent)

        offset = matvec(Q, t)
        position = tuple(p[i] + offset[i] for i in range(3))
        return root, position, matmul(Q, R)

    root_a, p_a, R_a = pose(target_frame)
    root_b, p_b, R_b = pose(source_frame)

    if root_a != root_b:
        raise ValueError("Frames are not in the same URDF tree")

    R_inv = transpose(R_a)
    delta = tuple(p_b[i] - p_a[i] for i in range(3))

    translation = matvec(R_inv, delta)
    rotation = matmul(R_inv, R_b)

    return (
        list(translation),
        [value for row in rotation for value in row],
    )