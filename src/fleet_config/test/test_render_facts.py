from collections import Counter
from collections.abc import Iterator
from pathlib import Path
from types import SimpleNamespace
from typing import Any

import pytest
import yaml

from fleet_config import model
from fleet_config.classes.terrascout import fixposition_tf_remaps
from fleet_config.model import Instance
from fleet_config.render import render_target
from fleet_config.targets import TARGETS
from fleet_unit.manifest import Manifest, Process, load_manifest

COLLISION_MONITOR_PLUGIN = "nav2_collision_monitor::CollisionMonitor"
REQUIRED_POLYGON_ACTIONS = ("slowdown",)
CLIENT_PUBLISH = "clientPublish"
FIXPOSITION_NAV2_EDGES = (("FP_ENU0", "map"), ("map", "odom"), ("odom", "vrtk_link"))


@pytest.fixture(params=sorted(TARGETS))
def rendered(request, tmp_path, monkeypatch) -> Path:
    monkeypatch.setattr(
        model, "SITE_FILE", model.SOURCE_ROOT / "fleet_config" / "site.example.yaml"
    )
    out = tmp_path / request.param
    render_target(request.param, Instance(1), out)
    return out


def unit_manifests(out: Path) -> Iterator[Manifest]:
    for unit in yaml.safe_load((out / "fleet.yaml").read_text())["units"]:
        yield load_manifest(out / "units" / unit["id"])


def parameters(manifest: Manifest, params: tuple[str, ...], node: str) -> dict[str, Any]:
    values: dict[str, Any] = {}
    for path in params:
        document = yaml.safe_load((manifest.unit_dir / path).read_text())
        values |= document.get("/**", {}).get("ros__parameters", {})
        values |= document.get(manifest.namespace, {}).get(node, {}).get("ros__parameters", {})
    return values


def process_parameters(manifest: Manifest, process: Process) -> dict[str, Any]:
    return parameters(manifest, process.params, process.node_name or process.name) | dict(
        process.param_values
    )


def tf_edges(manifest: Manifest, process: Process) -> Iterator[tuple[str, str]]:
    values = process_parameters(manifest, process)
    if process.executable == "fixposition_driver_ros2_exec" and values["nav2_mode"]:
        # Count Fixposition TFs only when they reach the live tree. Both
        # dynamic and static outputs must be isolated together.
        destinations = (
            process.remaps.get("/tf", "/tf"),
            process.remaps.get("/tf_static", "/tf_static"),
        )
        assert destinations in (
            ("/tf", "/tf_static"),
            ("tf", "tf_static"),
            ("fixposition/tf_isolated", "fixposition/tf_static_isolated"),
        ), f"{manifest.unit}/{process.name}: incomplete Fixposition TF isolation: {destinations}"
        if destinations != ("fixposition/tf_isolated", "fixposition/tf_static_isolated"):
            yield from FIXPOSITION_NAV2_EDGES
    if process.executable == "gnss_alignment_shadow.py" and values.get("publish_live_tf", False):
        yield "map", "odom"
    if process.executable == "gnss_shim.py":
        odom = values.get("frame_odom", "odom")
        yield values.get("frame_global", "map"), odom
    if process.executable == "fast_lio_odom.py":
        if process.name == "fast_lio_odom_preview":
            # This node publishes on private preview topics, not the live TF tree.
            # Fail loudly if either remap is accidentally removed or changed.
            assert process.remaps.get("/tf") == "fast_lio/odom_tf_preview", (
                f"{manifest.unit}/{process.name}: TF preview must not use live /tf"
            )
            assert process.remaps.get("odom") == "fast_lio/odom_preview", (
                f"{manifest.unit}/{process.name}: odometry preview must not use live /odom"
            )
        else:
            yield "odom", "vrtk_link"
    if process.executable == "odom_converter.py" and values.get("publish_tf", True):
        yield values.get("odom_frame", "odom"), values.get("base_frame", "base_link")
    if process.executable == "ros2_control_node":
        drive = parameters(manifest, process.params, "diff_drive_controller")
        if drive.get("enable_odom_tf", True):
            yield drive["odom_frame_id"], drive["base_frame_id"]


def test_fixposition_tf_isolation_is_opt_in_and_complete():
    from fleet_config.model import Features

    def make_unit(isolated: bool, site=None):
        robot = SimpleNamespace(features=Features(isolate_fixposition_tf=isolated))
        return SimpleNamespace(
            id="terrascout1", site=site or {}, resolved_robot=lambda: robot
        )

    original = fixposition_tf_remaps(make_unit(False))
    assert original["/tf"] == "tf"
    assert original["/tf_static"] == "tf_static"

    # Even a leftover legacy site.yaml switch cannot change the original
    # physical target. Only a Features flag on this target may isolate TF.
    legacy_site = fixposition_tf_remaps(
        make_unit(False, {"isolate_fixposition_tf": "true"})
    )
    assert legacy_site["/tf"] == "tf"
    assert legacy_site["/tf_static"] == "tf_static"

    isolated = fixposition_tf_remaps(make_unit(True))
    assert isolated["/tf"] == "fixposition/tf_isolated"
    assert isolated["/tf_static"] == "fixposition/tf_static_isolated"


def test_twist_mux_locks_have_positive_timeouts(rendered):
    for manifest in unit_manifests(rendered):
        for process in manifest.processes:
            if process.executable != "twist_mux":
                continue
            locks = process_parameters(manifest, process)["locks"]
            assert locks, f"{manifest.unit} twist_mux has no locks"
            for name, lock in locks.items():
                assert lock["timeout"] > 0, (
                    f"{manifest.unit} twist_mux lock {name} timeout {lock['timeout']}"
                )


def test_collision_monitor_polygons_are_enabled(rendered):
    for manifest in unit_manifests(rendered):
        for process in manifest.processes:
            for component in process.load:
                if component.plugin != COLLISION_MONITOR_PLUGIN:
                    continue
                monitor = parameters(manifest, component.params, component.node_name)
                enabled = {
                    monitor[name]["action_type"]
                    for name in monitor["polygons"]
                    if monitor[name]["enabled"] is True
                }
                for action in REQUIRED_POLYGON_ACTIONS:
                    assert action in enabled, (
                        f"{manifest.unit} {component.node_name} has no enabled {action} polygon"
                    )


def test_unit_services_run_an_init_process(rendered):
    services = yaml.safe_load((rendered / "compose.yaml").read_text())["services"]
    for unit in yaml.safe_load((rendered / "fleet.yaml").read_text())["units"]:
        assert services[unit["service"]].get("init") is True, (
            f"service {unit['service']} has no init: true"
        )


def test_every_tf_edge_has_one_publisher(rendered):
    for manifest in unit_manifests(rendered):
        edges = Counter(
            edge for process in manifest.processes for edge in tf_edges(manifest, process)
        )
        shared = [edge for edge, count in edges.items() if count > 1]
        assert not shared, f"{manifest.unit} publishes {shared} from more than one process"


def test_foxglove_bridges_have_topic_whitelists(rendered):
    for manifest in unit_manifests(rendered):
        for process in manifest.processes:
            if process.executable != "foxglove_bridge":
                continue
            values = process_parameters(manifest, process)
            assert values.get("topic_whitelist"), (
                f"{manifest.unit} foxglove_bridge has no topic_whitelist"
            )
            if CLIENT_PUBLISH in values["capabilities"]:
                assert values.get("client_topic_whitelist"), (
                    f"{manifest.unit} foxglove_bridge allows {CLIENT_PUBLISH} without client_topic_whitelist"
                )



def test_experimental_localization_is_off_in_all_original_targets(rendered):
    # Keep these original target names and their live localization unchanged.
    original_targets = {
        "terrascout1", "sim", "sim-record", "sim-madum",
        "sim-elevation-pc", "station",
    }
    preview_names = {
        "fast_lio_physical", "fast_lio_odom_preview",
        "fast_lio_odom_live", "gnss_alignment_shadow",
    }
    for manifest in unit_manifests(rendered):
        processes = {process.name: process for process in manifest.processes}
        if rendered.name in original_targets:
            assert not preview_names.intersection(processes), (
                f"{rendered.name}/{manifest.unit} unexpectedly launches experimental localization"
            )
        elif rendered.name == "terrascout-physical":
            handover = TARGETS[rendered.name].features.experimental_localization_handover
            adapter_name = "fast_lio_odom_live" if handover else "fast_lio_odom_preview"
            assert {"gnss_alignment_shadow", adapter_name} <= processes.keys()
            alignment = processes["gnss_alignment_shadow"]
            assert process_parameters(manifest, alignment)["publish_live_tf"] is handover
            assert alignment.remaps.get("/tf") == "tf"
            adapter = processes[adapter_name]
            assert adapter.remaps.get("/tf") == (
                "tf" if handover else "fast_lio/odom_tf_preview"
            )
            assert adapter.remaps.get("odom") == (
                "odom" if handover else "fast_lio/odom_preview"
            )
            fixposition = next(
                process for process in processes.values()
                if process.executable == "fixposition_driver_ros2_exec"
            )
            assert fixposition.remaps.get("/tf") == (
                "fixposition/tf_isolated" if handover else "tf"
            )
            assert fixposition.remaps.get("/tf_static") == (
                "fixposition/tf_static_isolated" if handover else "tf_static"
            )



def test_drive_odometry_isolation_is_opt_in():
    from types import SimpleNamespace

    from fleet_config.classes.terrascout import drive_controller_remaps
    from fleet_config.model import Features

    def unit_with_feature(enabled):
        robot = SimpleNamespace(features=Features(isolate_drive_odometry=enabled))
        return SimpleNamespace(resolved_robot=lambda: robot)

    original = drive_controller_remaps(unit_with_feature(False))
    assert original["diff_drive_controller/odom"] == "odom"
    assert original["diff_drive_controller/cmd_vel"] == "cmd_vel"
    assert original["/tf"] == "tf"
    assert original["/tf_static"] == "tf_static"

    isolated = drive_controller_remaps(unit_with_feature(True))
    assert isolated["diff_drive_controller/odom"] == "drive/odom"
    assert isolated["diff_drive_controller/cmd_vel"] == "cmd_vel"
    assert isolated["/tf"] == "tf"
    assert isolated["/tf_static"] == "tf_static"



def test_drive_odometry_isolation_is_off_in_every_current_target(rendered):
    handover = (
        rendered.name == "terrascout-physical"
        and TARGETS[rendered.name].features.experimental_localization_handover
    )
    expected = "drive/odom" if handover else "odom"
    for manifest in unit_manifests(rendered):
        for process in manifest.processes:
            if process.executable == "ros2_control_node":
                assert process.remaps.get("diff_drive_controller/odom") == expected, (
                    f"{rendered.name}/{manifest.unit}: unexpected drive odometry owner"
                )


def test_fixposition_tf_is_not_isolated_in_any_existing_target(rendered):
    # The original physical target must always retain Fixposition TF ownership.
    handover = (
        rendered.name == "terrascout-physical"
        and TARGETS[rendered.name].features.experimental_localization_handover
    )
    expected_tf = "fixposition/tf_isolated" if handover else "tf"
    expected_static = "fixposition/tf_static_isolated" if handover else "tf_static"
    for manifest in unit_manifests(rendered):
        for process in manifest.processes:
            if process.executable == "fixposition_driver_ros2_exec":
                assert process.remaps.get("/tf") == expected_tf, (
                    f"{rendered.name}/{manifest.unit}: unexpected Fixposition live TF owner"
                )
                assert process.remaps.get("/tf_static") == expected_static, (
                    f"{rendered.name}/{manifest.unit}: unexpected Fixposition static TF owner"
                )



def test_localization_handover_rejects_partial_and_simulation_modes():
    from types import SimpleNamespace

    from fleet_config.classes.terrascout import validate_localization_handover
    from fleet_config.errors import RenderError
    from fleet_config.model import Features, PHYSICAL, SIMULATION

    def unit(mode, **kwargs):
        features = Features(**kwargs)
        return SimpleNamespace(
            mode=mode,
            resolved_robot=lambda: SimpleNamespace(features=features),
        )

    assert validate_localization_handover(unit(PHYSICAL)) is False
    assert validate_localization_handover(
        unit(PHYSICAL, experimental_localization_handover=True)
    ) is True
    with pytest.raises(RenderError, match="partial TF/odometry isolation"):
        validate_localization_handover(unit(PHYSICAL, isolate_drive_odometry=True))
    with pytest.raises(RenderError, match="partial TF/odometry isolation"):
        validate_localization_handover(unit(PHYSICAL, isolate_fixposition_tf=True))
    with pytest.raises(RenderError, match="only supported by a physical"):
        validate_localization_handover(
            unit(SIMULATION, experimental_localization_handover=True)
        )


def test_localization_handover_switch_renders_coherent_live_tf(tmp_path, monkeypatch):
    from dataclasses import replace

    # Enable ONLY in a test copy of the experimental physical target.
    # The repository target remains diagnostic-only.
    target = TARGETS["terrascout-physical"]
    trial = replace(
        target,
        features=replace(target.features, experimental_localization_handover=True),
    )
    monkeypatch.setitem(TARGETS, "terrascout-physical", trial)
    monkeypatch.setattr(
        model, "SITE_FILE", model.SOURCE_ROOT / "fleet_config" / "site.example.yaml"
    )
    out = tmp_path / "handover"
    render_target("terrascout-physical", Instance(1), out)
    for manifest in unit_manifests(out):
        by_name = {process.name: process for process in manifest.processes}
        alignment = by_name["gnss_alignment_shadow"]
        adapter = by_name["fast_lio_odom_live"]
        assert "fast_lio_odom_preview" not in by_name
        assert process_parameters(manifest, alignment)["publish_live_tf"] is True
        assert alignment.remaps["/tf"] == "tf"
        assert adapter.remaps["/tf"] == "tf"
        assert adapter.remaps["odom"] == "odom"
        gnss = by_name["gnss"]
        assert gnss.remaps["/tf"] == "fixposition/tf_isolated"
        assert gnss.remaps["/tf_static"] == "fixposition/tf_static_isolated"
        controller = by_name["ros2_control_node"]
        assert controller.remaps["diff_drive_controller/odom"] == "drive/odom"
        edges = Counter(
            edge for process in manifest.processes for edge in tf_edges(manifest, process)
        )
        assert edges[("map", "odom")] == 1
        assert edges[("odom", "vrtk_link")] == 1
        assert all(n == 1 for n in edges.values()), edges
