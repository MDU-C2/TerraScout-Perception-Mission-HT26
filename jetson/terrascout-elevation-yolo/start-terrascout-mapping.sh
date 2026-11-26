#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "=========================================="
echo " TerraScout GPU Elevation Mapping"
echo "=========================================="
echo "Starting ROS 2 Jazzy + CUDA + Zenoh..."
echo ""

exec "$SCRIPT_DIR/run-jazzy-zenoh.sh" bash -lc '
    set -e

    # ROS 2 environment
    source /opt/ros/jazzy/setup.bash
    source /workspace/ros2_ws/install/setup.bash

    # ROS 2 middleware
    export RMW_IMPLEMENTATION=rmw_zenoh_cpp
    export ROS_DOMAIN_ID=133

    # Connect to the TerraScout PC Zenoh router
    export ZENOH_CONFIG_OVERRIDE="mode=\"client\";connect/endpoints=[\"tcp/130.243.66.211:7447\"];scouting/multicast/enabled=false"

    # Mapping configuration
    CORE=$(ros2 pkg prefix --share elevation_mapping_cupy)/config/core/core_param.yaml

    SETUP=/workspace/ros2_ws/src/elevation_mapping_cupy/elevation_mapping_cupy/config/setups/terrascout/terrascout.yaml

    # Verify configuration files
    if [ ! -f "$CORE" ]; then
        echo "ERROR: Core mapping configuration not found."
        exit 1
    fi

    if [ ! -f "$SETUP" ]; then
        echo "ERROR: TerraScout mapping configuration not found."
        exit 1
    fi

    echo "ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
    echo "RMW: $RMW_IMPLEMENTATION"
    echo "Mapping configuration: $SETUP"
    echo ""

    echo "Starting elevation_mapping_node..."

    exec ros2 run elevation_mapping_cupy elevation_mapping_node.py \
        --ros-args \
        --params-file "$CORE" \
        --params-file "$SETUP" \
        -p use_sim_time:=true \
        -r /tf:=/terrascout1/tf \
        -r /tf_static:=/terrascout1/tf_static
'
