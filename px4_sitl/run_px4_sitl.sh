#!/bin/bash
set -e
AUTH_FILE=$(ps aux | grep -oP '(?<=-auth )\S+' | head -n 1)
[ -n "$AUTH_FILE" ] && [ -f "$AUTH_FILE" ] && sudo cp "$AUTH_FILE" /home/ubuntu/.Xauthority && sudo chown ubuntu:ubuntu /home/ubuntu/.Xauthority
export DISPLAY=:0 XAUTHORITY=/home/ubuntu/.Xauthority
xhost +local:docker >/dev/null 2>&1 || true

sg docker -c "docker run --rm -it --name px4-sitl \
  --network host \
  --gpus all \
  -e DISPLAY=$DISPLAY \
  -e XAUTHORITY=/root/.Xauthority \
  -e NVIDIA_VISIBLE_DEVICES=all \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v /home/ubuntu/.Xauthority:/root/.Xauthority:ro \
  -v /home/ubuntu/px4_docker_ws/PX4-Autopilot:/src/PX4-Autopilot:rw \
  -w /src/PX4-Autopilot \
  px4io/px4-dev-simulation-jammy \
  bash -c \"git config --global --add safe.directory '*' && make px4_sitl gz_x500\""
