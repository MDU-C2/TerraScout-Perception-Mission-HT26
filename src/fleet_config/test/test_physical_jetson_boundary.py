"""Step 24: physical NUC/Jetson boundary and legacy-target regression tests."""

import json

from fleet_config import model
from fleet_config.classes.terrascout import TERRASCOUT
from fleet_config.model import Instance
from fleet_config.paths import DESCRIPTION_FILE, ROUTER_CONFIG_FILE
from fleet_config.render import render_target
from fleet_config.unit import (
    CLOCK_TOPIC,
    distributed_simulation_boundary,
    robot_exports,
)
from fleet_config.zenoh import data_key
from fleet_unit.manifest import load_manifest

NS = "terrascout1"
LIDAR = f"/{NS}/lidar_points_filtered"
TF = f"/{NS}/tf"
TF_STATIC = f"/{NS}/tf_static"
RGB = f"/{NS}/oakd/rgb/image_raw"
GRID = "/elevation_mapping_node/elevation_map"
DETECTIONS = f"/{NS}/oakd/detections_2d"
DETECTIONS_IMAGE = f"/{NS}/oakd/detections_image"


def test_physical_boundary_reuses_jetson_contract_without_simulated_inputs():
    sim = distributed_simulation_boundary(NS, TERRASCOUT, "elevation-pc")
    physical = distributed_simulation_boundary(NS, TERRASCOUT, "physical-jetson-nuc")

    # All Jetson->NUC topics and their ROS message types match the
    # previously working simulation deployment.
    assert physical.imports == sim.imports
    assert {entry.topic for entry in physical.imports} == {
        GRID, DETECTIONS, DETECTIONS_IMAGE,
    }
    # The physical robot uses wall time; the physical NUC currently has no
    # producer for the simulation OAK-D RGB image stream.
    assert physical.exports == tuple(
        entry for entry in sim.exports if entry.topic not in {CLOCK_TOPIC, RGB}
    )
    assert {LIDAR, TF, TF_STATIC} <= {entry.topic for entry in physical.exports}
    assert CLOCK_TOPIC not in {entry.topic for entry in physical.exports}
    assert RGB not in {entry.topic for entry in physical.exports}


def test_original_boundaries_stay_unchanged():
    default = distributed_simulation_boundary(NS, TERRASCOUT, "combined")
    assert default.exports == robot_exports(NS, TERRASCOUT)
    assert default.imports == ()

    simulated = distributed_simulation_boundary(NS, TERRASCOUT, "elevation-pc")
    assert CLOCK_TOPIC in {entry.topic for entry in simulated.exports}
    assert RGB in {entry.topic for entry in simulated.exports}
    assert GRID in {entry.topic for entry in simulated.imports}


def test_physical_boundary_is_in_rendered_description_and_zenoh_acl(tmp_path, monkeypatch):
    monkeypatch.setattr(
        model, "SITE_FILE", model.SOURCE_ROOT / "fleet_config" / "site.example.yaml"
    )
    out = tmp_path / "terrascout-physical"
    render_target("terrascout-physical", Instance(1), out)
    unit_dir = out / "units" / NS

    description = json.loads((unit_dir / DESCRIPTION_FILE).read_text())
    exported = {entry["topic"]: entry["type"] for entry in description["exports"]}
    imported = {entry["topic"]: entry["type"] for entry in description["imports"]}
    assert exported[LIDAR] == "sensor_msgs/msg/PointCloud2"
    assert exported[TF] == exported[TF_STATIC] == "tf2_msgs/msg/TFMessage"
    assert imported[GRID] == "grid_map_msgs/msg/GridMap"
    assert imported[DETECTIONS] == "vision_msgs/msg/Detection2DArray"
    assert imported[DETECTIONS_IMAGE] == "sensor_msgs/msg/Image"
    assert CLOCK_TOPIC not in exported
    assert RGB not in exported

    router = json.loads((unit_dir / ROUTER_CONFIG_FILE).read_text())
    rules = {rule["id"]: rule for rule in router["access_control"]["rules"]}
    domain = Instance(1).domain_id
    for topic in (LIDAR, TF, TF_STATIC):
        assert data_key(domain, topic) in rules["boundary_export_egress"]["key_exprs"]
    for topic in (GRID, DETECTIONS, DETECTIONS_IMAGE):
        assert data_key(domain, topic) in rules["boundary_import_ingress"]["key_exprs"]

    # Declaring the Jetson detection import makes physical perception use
    # the external inference node rather than launch a second local YOLO node.
    manifest = load_manifest(unit_dir)
    assert "yolo_detector" not in {process.name for process in manifest.processes}
