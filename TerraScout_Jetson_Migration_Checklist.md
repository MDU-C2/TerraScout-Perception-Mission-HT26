# TerraScout + Jetson: Moving the Distributed Simulation to Another PC

**Repository:** `MDU-C2/TerraScout-Perception-Mission-HT26`  
**Branch:** `feature/fastlio2-stvl-integration`  
**Simulation target:** `sim-elevation-pc`  
**Example PC IPv4 address:** `10.132.135.87` (verify on the destination PC)  
**Example PC Ethernet interface:** `eno1` (verify on the destination PC)  
**ROS domain ID on this branch:** `133`  
**Zenoh port:** `7447` (default with `SAGE_INSTANCE=1`)

## 1. Overview

When moving the TerraScout + Jetson elevation-mapping setup to another simulation PC, the PC's network address, Zenoh router binding, and ROS domain configuration must agree with the Jetson startup configuration. In addition, the `feature/fastlio2-stvl-integration` branch was missing some deployment-model wiring from `feature/distributed-simulation`.

```text
Simulation PC (Gazebo / FAST-LIO2 / Nav2)
  ├── /terrascout1/lidar_points_raw → FAST-LIO2 (stays on PC)
  ├── /terrascout1/lidar_points_filtered ─────┐
  ├── /terrascout1/tf, /tf_static ────────────┤ Zenoh TCP
  └── /clock ────────────────────────────────┘
                                              ▼
                                   Jetson elevation_mapping_cupy
                                              │
                          /elevation_mapping_node/elevation_map
                                              │ Zenoh TCP
                                              ▼
                                   PC Nav2 costmaps / Foxglove
```

## 2. Changes made or required on the simulation PC

Files below are relative to `src/fleet_config/fleet_config/`.

| File | Change | Reason |
| --- | --- | --- |
| `targets.py` | Define `sim-elevation-pc` with `deployment="elevation-pc"`, the PC IP and interface | Select the distributed target and identify the PC network interface |
| `model.py` | Add `deployment` to `Fleet`; add `deployment` and `topology` to `SimTarget`; propagate both in `SimTarget.fleet()` | Resolve `unexpected keyword argument 'deployment'` and carry target settings into the rendered fleet |
| `unit.py` | Use `distributed_simulation_boundary(...)` in robot `UnitContext` creation | Enable mapping-specific Zenoh export/import rules |
| `zenoh.py` | Make `fleet_router_address()` use `fleet.topology.uplink_address` for distributed simulation | Avoid binding the PC's fleet router only to loopback (`127.0.0.1`) |
| `model.py` | Check `Instance.domain_id` (currently `133`) | Match the Jetson ROS domain ID |

### 2.1 Find the PC's actual IP and interface

Run **on the PC host** (not inside Docker):

```bash
ip -4 addr
ip -4 addr show eno1
```

Use the IP and interface name actually assigned to the destination PC; the example values below are machine-specific.

### 2.2 `targets.py`: configure the destination PC (uplink_adress is the IP adress of the PC/NUC)

```python
"sim-elevation-pc": replace(
    SIM,
    deployment="elevation-pc",
    topology=Topology(
        uplink_address="10.132.135.87",
        uplink_interface="eno1",
    ),
),
```

**Important:** `uplink_address` is the **PC's** address to which the Jetson connects, **not** the Jetson's IP.

### 2.3 `model.py`: distributed-deployment support

Add to the end of the `Fleet` dataclass:

```python
deployment: str = "combined"
```

Add to the `SimTarget` dataclass:

```python
deployment: str = "combined"
topology: Topology | None = None
```

In `SimTarget.fleet()`, pass through the target values:

```python
topology=self.topology,
# ...
deployment=self.deployment,
```

The earlier version had `topology=None` hardcoded and did not support a `deployment` argument, causing the target-import/build error even when building another target such as `sim-record`.

Check the domain ID in the `Instance` class:

```python
@property
def domain_id(self) -> int:
    return 133
```

Changing the domain is optional; **matching the domain on both machines is mandatory**.

### 2.4 `unit.py`: connect the boundary function

In `contexts()`, use:

```python
boundary=distributed_simulation_boundary(
    robot.id,
    robot.robot_class,
    fleet.deployment,
),
```

The `elevation-pc` boundary should export `/clock`, filtered LiDAR, dynamic/static TF, and regular robot exports; it should import the returned GridMap. Raw LiDAR stays local to the PC for FAST-LIO2.

| Direction | Topic |
| --- | --- |
| PC → Jetson | `/clock` |
| PC → Jetson | `/terrascout1/lidar_points_filtered` |
| PC → Jetson | `/terrascout1/tf` |
| PC → Jetson | `/terrascout1/tf_static` |
| Jetson → PC | `/elevation_mapping_node/elevation_map` |

### 2.5 `zenoh.py`: make the PC router externally reachable

In `fleet_router_address()`:

```python
def fleet_router_address(fleet: Fleet) -> str:
    if fleet.mode == SIMULATION:
        return (
            fleet.topology.uplink_address
            if fleet.topology is not None
            else LOOPBACK_ADDRESS
        )

    assert fleet.station is not None
    return site_value(fleet.site, fleet.station.id, UPLINK_ADDRESS_KEY)
```

Without this change, the current branch's simulation router listens on loopback only, so the Jetson cannot connect to the PC via Ethernet.

## 3. Changes required on the Jetson

The Jetson runs `elevation_mapping_cupy` in a separate Docker workspace under `~/jetson-elevation/`, outside the TerraScout PC repository.

Look for settings in these startup scripts **on the Jetson host**:

```bash
cd ~/jetson-elevation
grep -nE 'ROS_DOMAIN_ID|ZENOH_CONFIG_OVERRIDE|connect/endpoints' \
  start-terrascout-mapping.sh run-jazzy-zenoh.sh
```

Set these in the **effective environment of the running mapping node** (the relevant startup script or the shell *inside the Jetson Docker container*):

```bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ROS_DOMAIN_ID=133
export ZENOH_CONFIG_OVERRIDE='mode="client";connect/endpoints=["tcp/10.132.135.87:7447"];scouting/multicast/enabled=false'
```

**Note:** `ROS_DOMAIN_ID` and `ZENOH_CONFIG_OVERRIDE` may be set in a launcher, rather than in `terrascout.yaml`. If a script overwrites environment variables, edit that script instead of setting them only in the parent shell. The endpoint must point to the **PC**, not the Jetson.

To find the Jetson's own IP for SSH, run on the **Jetson host**:

```bash
ip -4 addr
# Or, if Ethernet is eno1:
ip -4 addr show eno1
```

The Jetson's IP is generally used for remote access (SSH); it is not the Zenoh `connect/endpoints` destination in this setup.

## 4. Build, start, and verify

### PC host

```bash
make render TARGET=sim-elevation-pc
make build TARGET=sim-elevation-pc   # if required for source/dependency changes
make up TARGET=sim-elevation-pc
```

Open a second PC terminal:

```bash
make shell TARGET=sim-elevation-pc SERVICE=unit-terrascout1
```

### Jetson host

After the PC simulation is running:

```bash
cd ~/jetson-elevation
./start-terrascout-mapping.sh
```

Make sure that launcher applies the domain ID and Zenoh endpoint above.

### Inside the PC ROS 2 container

```bash
ros2 topic list -t --no-daemon | grep elevation
ros2 topic hz /elevation_mapping_node/elevation_map
ros2 topic info /elevation_mapping_node/elevation_map --verbose
```

Verify the topic carries `grid_map_msgs/msg/GridMap`, ideally near the Jetson's configured 5 Hz publication target, and that Nav2's global/local costmap subscribers are present. An actual rate must be **measured**; 5 Hz is a configuration target, not a guarantee.

For Nav2 plugin status:

```bash
ros2 param get /terrascout1/global_costmap/global_costmap traversability_layer.enabled
ros2 param get /terrascout1/local_costmap/local_costmap traversability_layer.enabled
```

Both should report `True` if enabled in the running Nav2 configuration.

## 5. What is confirmed vs. what still needs verification?

**Confirmed in the conversation:** The initial `SimTarget` import error was fixed by changing `model.py`/`unit.py`, and the user reported a successful build.

**Recommended but not yet confirmed as successfully applied/tested on the new PC:** The `zenoh.py` listener change; matching the Jetson's `ROS_DOMAIN_ID` to `133`; updating the Jetson TCP endpoint; successful PC ↔ Jetson map transport; and any terrain-aware Smac/MPPI driving effects.

A successful build alone does not prove the distributed ROS 2 connection works. First verify the GridMap reaches Nav2; then compare planned paths and controller behavior with traversability enabled vs. disabled.

## 6. Quick migration checklist (fill boxes yourself if you do this)

- [ ] Find the **new PC IP address** and Ethernet interface with `ip -4 addr`.
- [ ] Update `targets.py` (`uplink_address`, `uplink_interface`).
- [ ] Ensure `model.py` supports and propagates `deployment` and `topology`.
- [ ] Ensure `unit.py` uses `distributed_simulation_boundary(...)`.
- [ ] Ensure `zenoh.py` binds the fleet router to the configured PC IP.
- [ ] Check PC `Instance.domain_id` and set the same `ROS_DOMAIN_ID` on the Jetson.
- [ ] Point Jetson `ZENOH_CONFIG_OVERRIDE` at `tcp/<NEW_PC_IP>:7447` (assuming instance 1).
- [ ] Render/start `sim-elevation-pc` on the PC; start mapping on the Jetson.
- [ ] Verify `/elevation_mapping_node/elevation_map` on the PC and confirm Nav2 costmap subscriptions.
- [ ] Test terrain-dependent costmaps, Smac planning, and MPPI control separately.
