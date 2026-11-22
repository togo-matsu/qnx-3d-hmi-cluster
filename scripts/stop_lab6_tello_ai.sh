#!/usr/bin/env bash
pkill -f tello_qnx_arbiter_node 2>/dev/null || true
pkill -f tello_vision_node 2>/dev/null || true
~/qnx-3d-hmi-cluster/scripts/stop_tello_ros2.sh
echo "LAB 6 Tello AI & QNX Arbiter stopped."
