#!/usr/bin/env bash
set -e
pkill -f tello_bridge.py 2>/dev/null || true
nohup python3 -u ~/qnx-3d-hmi-cluster/edge_gateway/tello_bridge.py > /tmp/tello_bridge.log 2>&1 &
sleep 3
tail -n 6 /tmp/tello_bridge.log
