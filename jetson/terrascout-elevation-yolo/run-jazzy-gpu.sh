#!/usr/bin/env bash
set -e

sudo docker run --rm -it \
  --runtime=nvidia \
  --user "$(id -u):$(id -g)" \
  -e HOME=/tmp \
  --network=none \
  -e NVIDIA_VISIBLE_DEVICES=all \
  -e NVIDIA_DRIVER_CAPABILITIES=compute,utility \
  -e CUDA_PATH=/usr/local/cuda \
  -e LD_LIBRARY_PATH=/usr/local/cuda/targets/aarch64-linux/lib:/usr/local/cuda/lib64:/usr/local/nvidia/lib:/usr/local/nvidia/lib64 \
  -v /usr/local/cuda:/usr/local/cuda:ro \
  -v "$HOME/jetson-elevation/ros2_ws:/workspace/ros2_ws" \
  -w /workspace/ros2_ws \
  jetson-jazzy-elevation:dev \
  "$@"
