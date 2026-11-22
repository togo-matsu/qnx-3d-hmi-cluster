#!/usr/bin/env bash
set -e
source /opt/ros/humble/setup.bash
source ~/qnx-3d-hmi-cluster/ros2_ws/install/setup.bash
pkill -f tello_ros2_node 2>/dev/null || true
nohup ros2 run sdv_control tello_ros2_node > /tmp/tello_ros2.log 2>&1 &
sleep 2
tail -n 6 /tmp/tello_ros2.log
echo ""
echo "操作ノード: ros2 run sdv_control tello_teleop_node"
echo "RViz2起動 : DISPLAY=:0 nohup rviz2 > /tmp/rviz2.log 2>&1 &"
