from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass, replace
from types import MappingProxyType

from fleet_config.model import (
    ROBOT,
    SIMULATION,
    STATION,
    WORLD,
    WORLD_UNIT_ID,
    Fleet,
    Instance,
    ResolvedRobot,
    RobotClass,
    Simulation,
    Station,
    Topology,
)
from fleet_config.paths import CONTAINER_WORKSPACE_DIR, unit_dir_of, unit_path, unit_run_dir_of

INTERFACES_PACKAGE = "fleet_interfaces"
UNIT_DESCRIPTION_TYPE = f"{INTERFACES_PACKAGE}/msg/UnitDescription"
HEALTH_TYPE = "diagnostic_msgs/msg/DiagnosticArray"
MISSION_STATE_TYPE = f"{INTERFACES_PACKAGE}/msg/MissionState"

HEALTH_MAX_HZ = 2.0
POSITION_MAX_HZ = 1.0

CLOCK_TOPIC = "/clock"
CLOCK_TYPE = "rosgraph_msgs/msg/Clock"

MISSION_STATE_TOPIC = "mission_state"
ANY_UNIT_DESCRIPTION_TOPIC = "/*/unit/description"
ANY_UNIT_HEALTH_TOPIC = "/*/unit/health"
ANY_UNIT_MISSION_STATE_TOPIC = f"/*/{MISSION_STATE_TOPIC}"


@dataclass(frozen=True)
class BoundaryEntry:
    topic: str
    type: str
    max_hz: float | None
    publisher_namespace: str | None


@dataclass(frozen=True)
class GeneratedFile:
    name: str
    text: str


@dataclass(frozen=True)
class Boundary:
    exports: tuple[BoundaryEntry, ...] = ()
    imports: tuple[BoundaryEntry, ...] = ()


@dataclass(frozen=True)
class UnitContext:
    id: str
    kind: str
    namespace: str
    index: int
    instance: Instance
    mode: str
    use_sim_time: bool
    boundary: Boundary
    robot: ResolvedRobot | None = None
    station: Station | None = None
    topology: Topology | None = None
    simulation: Simulation | None = None
    gz_resource_paths: tuple[str, ...] = ()
    site: Mapping[str, str] = MappingProxyType({})
    deployment: str = "combined"

    @property
    def on_fleet_host(self) -> bool:
        return self.kind == STATION or (
            self.mode == SIMULATION and self.deployment != "nuc"
        )

    @property
    def router_tcp_port(self) -> int:
        return self.instance.unit_router_tcp_port(self.index)

    @property
    def foxglove_port(self) -> int:
        return self.instance.foxglove_port(self.index)

    @property
    def domain_id(self) -> int:
        return self.instance.domain_id

    @property
    def path(self) -> str:
        return unit_path(self.id)

    @property
    def dir(self) -> str:
        return unit_dir_of(self.id)

    @property
    def run_dir(self) -> str:
        return unit_run_dir_of(self.id)

    @property
    def service(self) -> str:
        return WORLD if self.kind == WORLD else f"unit-{self.id}"

    def file(self, relative: str) -> str:
        return f"{self.dir}/{relative}"

    def scoped(self, name: str) -> str:
        return name if name.startswith("/") else f"/{self.namespace}/{name}"

    def resolved_robot(self) -> ResolvedRobot:
        if self.robot is None:
            raise ValueError(
                f"resolve robot of unit {self.id} failed: cause: the unit is not a robot"
            )
        return self.robot


def description_topic(namespace: str) -> str:
    return f"/{namespace}/unit/description"


def health_topic(namespace: str) -> str:
    return f"/{namespace}/unit/health"


def mission_state_topic(namespace: str) -> str:
    return f"/{namespace}/{MISSION_STATE_TOPIC}"


def position_topic(namespace: str, robot_class: RobotClass) -> str:
    return f"/{namespace}/{robot_class.position_topic}"


def robot_exports(namespace: str, robot_class: RobotClass) -> tuple[BoundaryEntry, ...]:
    return (
        BoundaryEntry(description_topic(namespace), UNIT_DESCRIPTION_TYPE, None, namespace),
        BoundaryEntry(health_topic(namespace), HEALTH_TYPE, HEALTH_MAX_HZ, namespace),
        BoundaryEntry(mission_state_topic(namespace), MISSION_STATE_TYPE, None, namespace),
        BoundaryEntry(
            position_topic(namespace, robot_class),
            robot_class.position_type,
            POSITION_MAX_HZ,
            namespace,
        ),
    )


def distributed_simulation_boundary(
    namespace: str,
    robot_class: RobotClass,
    deployment: str,
) -> Boundary:
    """ROS 2 topics exchanged between the simulation PC and NUC."""

    # Sensor data published by Gazebo on the PC.
    sensor_topics = (
        ("/clock", CLOCK_TYPE),
        (f"/{namespace}/lidar_points_raw", "sensor_msgs/msg/PointCloud2"),
        (f"/{namespace}/microstrain/imu/data", "sensor_msgs/msg/Imu"),
        (f"/{namespace}/navsat", "sensor_msgs/msg/NavSatFix"),
        (f"/{namespace}/joint_states", "sensor_msgs/msg/JointState"),
        (f"/{namespace}/odom_gt", "nav_msgs/msg/Odometry"),
    )

    sensors = tuple(
        BoundaryEntry(topic, msg_type, None, None)
        for topic, msg_type in sensor_topics
    )

    # Final velocity command generated on the NUC.
    cmd_vel = BoundaryEntry(
        f"/{namespace}/cmd_vel",
        "geometry_msgs/msg/TwistStamped",
        None,
        namespace,
    )

    if deployment == "pc":
        return Boundary(
            exports=sensors,
            imports=(cmd_vel,),
        )

    if deployment == "nuc":
        return Boundary(
            exports=(
                *robot_exports(namespace, robot_class),
                cmd_vel,
            ),
            imports=sensors,
        )

    return Boundary(
        exports=robot_exports(namespace, robot_class),
    )


def world_boundary() -> Boundary:
    return Boundary()


def station_boundary(station_id: str, positions: list[tuple[str, str]]) -> Boundary:
    imports = [
        BoundaryEntry(ANY_UNIT_DESCRIPTION_TOPIC, UNIT_DESCRIPTION_TYPE, None, None),
        BoundaryEntry(ANY_UNIT_HEALTH_TOPIC, HEALTH_TYPE, None, None),
        BoundaryEntry(ANY_UNIT_MISSION_STATE_TOPIC, MISSION_STATE_TYPE, None, None),
    ]
    for topic, message_type in positions:
        imports.append(BoundaryEntry(f"/*/{topic}", message_type, None, None))
    return Boundary(
        exports=(
            BoundaryEntry(description_topic(station_id), UNIT_DESCRIPTION_TYPE, None, station_id),
            BoundaryEntry(health_topic(station_id), HEALTH_TYPE, HEALTH_MAX_HZ, station_id),
        ),
        imports=tuple(imports),
    )


def station_position_topics(station: Station) -> list[tuple[str, str]]:
    positions: list[tuple[str, str]] = []
    for _, robot_class in station.robots:
        entry = (robot_class.position_topic, robot_class.position_type)
        if entry[0] not in {topic for topic, _ in positions}:
            positions.append(entry)
    return positions


def _world_resource_paths(robots: tuple[ResolvedRobot, ...]) -> tuple[str, ...]:
    paths: list[str] = []
    for robot in robots:
        models = robot.robot_class.gz_models_dir
        if models is not None and models not in paths:
            paths.append(models)
    for robot in robots:
        package = robot.robot_class.description_package
        share = f"{CONTAINER_WORKSPACE_DIR}/install/{package}/share"
        if package is not None and share not in paths:
            paths.append(share)
    return tuple(paths)


def contexts(
    fleet: Fleet, robots: tuple[ResolvedRobot, ...], instance: Instance
) -> list[UnitContext]:
    units: list[UnitContext] = []
    for index, robot in enumerate(robots):
        units.append(
            UnitContext(
                id=robot.id,
                kind=ROBOT,
                namespace=robot.id,
                index=index,
                instance=instance,
                mode=robot.mode,
                use_sim_time=robot.use_sim_time,
                boundary=distributed_simulation_boundary(robot.id,robot.robot_class,fleet.deployment,),
                robot=robot,
                topology=robot.topology,
                simulation=robot.simulation,
                site=fleet.site,
                deployment=fleet.deployment,
            )
        )
    if fleet.simulation is not None and fleet.deployment != "nuc":
        units.append(
            UnitContext(
                id=WORLD_UNIT_ID,
                kind=WORLD,
                namespace="",
                index=len(robots),
                instance=instance,
                mode=fleet.mode,
                use_sim_time=fleet.use_sim_time,
                boundary=world_boundary(),
                simulation=fleet.simulation,
                gz_resource_paths=_world_resource_paths(robots),
                deployment=fleet.deployment,
            )
        )
    if fleet.station is not None and fleet.deployment != "nuc":
        station = replace(
            fleet.station,
            robots=tuple((robot.id, robot.robot_class) for robot in robots) + fleet.station.robots,
        )
        units.append(
            UnitContext(
                id=station.id,
                kind=STATION,
                namespace=station.id,
                index=len(units),
                instance=instance,
                mode=fleet.mode,
                use_sim_time=fleet.use_sim_time,
                boundary=station_boundary(station.id, station_position_topics(station)),
                station=station,
                simulation=fleet.simulation,
                site=fleet.site,
                deployment=fleet.deployment,
            )
        )
    return units
