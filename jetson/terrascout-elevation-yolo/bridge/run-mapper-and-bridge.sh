#!/usr/bin/env bash
set -eo pipefail

# Source ROS 2 before enabling strict undefined-variable checking.
source /opt/ros/jazzy/setup.bash
source /workspace/ros2_ws/install/setup.bash

set -u

CORE="$(ros2 pkg prefix --share elevation_mapping_cupy)/config/core/core_param.yaml"
SETUP="/workspace/ros2_ws/src/elevation_mapping_cupy/elevation_mapping_cupy/config/setups/terrascout/terrascout.yaml"
for path in "$CORE" "$SETUP"; do
  if [ ! -f "$path" ]; then echo "ERROR: Missing mapping file $path" >&2; exit 1; fi
done

# The adapter uses the same ROS 2/Zenoh environment as elevation mapping.
PYTHONPATH="/opt/terrascout-source:${PYTHONPATH:-}" \
  python3 -u /opt/terrascout-bridge/ros_adapter.py &
BRIDGE_PID=$!

ros2 run elevation_mapping_cupy elevation_mapping_node.py \
    --ros-args --params-file "$CORE" --params-file "$SETUP" \
    -p use_sim_time:=true \
    -r /tf:=/terrascout1/tf \
    -r /tf_static:=/terrascout1/tf_static &
MAP_PID=$!

cleanup() {
  trap - EXIT INT TERM
  kill -TERM "$BRIDGE_PID" "$MAP_PID" 2>/dev/null || true
  wait "$BRIDGE_PID" "$MAP_PID" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

# Exit if either process terminates; Compose stops the other service.
wait -n "$BRIDGE_PID" "$MAP_PID"
