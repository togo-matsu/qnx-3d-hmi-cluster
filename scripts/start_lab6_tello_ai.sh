#!/usr/bin/env bash
set -e
source /opt/ros/humble/setup.bash
source ~/qnx-3d-hmi-cluster/ros2_ws/install/setup.bash
DISPLAY=:0 xset s off -dpms 2>/dev/null || true

pgrep -f tello_ros2_node >/dev/null || ~/qnx-3d-hmi-cluster/scripts/start_tello_ros2.sh
pkill -f tello_vision_node 2>/dev/null || true
pkill -f tello_qnx_arbiter_node 2>/dev/null || true
sleep 1

nohup ros2 run sdv_control tello_vision_node > /tmp/tello_vision.log 2>&1 &
nohup ros2 run sdv_control tello_qnx_arbiter_node > /tmp/tello_arbiter.log 2>&1 &
sleep 3
tail -n 4 /tmp/tello_vision.log /tmp/tello_arbiter.log
echo ""
echo "Follow ON  : ros2 topic pub --once /tello/follow_enable std_msgs/msg/Bool '{data: true}'"
echo "Follow OFF : ros2 topic pub --once /tello/follow_enable std_msgs/msg/Bool '{data: false}'"
echo "Reset Latch: ros2 topic pub --once /tello/safety_reset std_msgs/msg/Bool '{data: true}'"
