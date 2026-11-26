#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")"

export HOST_UID="$(id -u)"
export HOST_GID="$(id -g)"
export TERRASCOUT_YOLO_PKG="${TERRASCOUT_YOLO_PKG:-$HOME/terrascout-yolo/src/s_perception_2d}"
export TERRASCOUT_YOLO_MODELS="${TERRASCOUT_YOLO_MODELS:-$TERRASCOUT_YOLO_PKG/models}"

# User can set deployment-specific values in a local .env.perception file.
if [ -f .env.perception ]; then
  set -a
  # shellcheck disable=SC1091
  source ./.env.perception
  set +a
fi

: "${PC_ZENOH_HOST:?Set PC_ZENOH_HOST in .env.perception}"

[ -f ./ros2_ws/install/setup.bash ] || { echo 'ERROR: Mapping workspace has not been built.' >&2; exit 1; }
[ -f "$TERRASCOUT_YOLO_MODELS/best.pt" ] || { echo "ERROR: Model not found: $TERRASCOUT_YOLO_MODELS/best.pt" >&2; exit 1; }

# Preserve existing elevation-mapping images. Build only our small derived image.
mkdir -p ./perception-logs

if ! sudo docker compose version >/dev/null 2>&1; then
  echo "ERROR: Docker Compose plugin is not installed." >&2
  exit 1
fi

if ! sudo docker image inspect jetson-jazzy-perception:dev >/dev/null 2>&1; then
  sudo docker build -f Dockerfile.perception -t jetson-jazzy-perception:dev .
fi

# foreground: Ctrl+C stops both services via Docker Compose.
exec sudo --preserve-env=HOST_UID,HOST_GID,TERRASCOUT_YOLO_PKG,TERRASCOUT_YOLO_MODELS,PC_ZENOH_HOST,PC_ZENOH_PORT,ROS_DOMAIN_ID,YOLO_IMGSZ,YOLO_MIN_NOISE_THRESHOLD,YOLO_IOU_THRESHOLD,YOLO_PORT,YOLO_INPUT_TOPIC,YOLO_OUTPUT_TOPIC,YOLO_TARGET_FPS,YOLO_CONFIDENCE_THRESHOLD,YOLO_MAX_ASPECT_RATIO_H_W,YOLO_PUBLISH_ANNOTATED,YOLO_ENABLE_FILE_LOGGING,YOLO_ANNOTATED_TOPIC \
  docker compose -f compose.perception.yaml up --no-build --abort-on-container-failure
