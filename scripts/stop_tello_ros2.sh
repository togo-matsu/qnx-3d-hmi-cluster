#!/usr/bin/env bash
pkill -f tello_teleop_node 2>/dev/null || true
pkill -f tello_ros2_node 2>/dev/null || true
pkill -f rviz2 2>/dev/null || true
echo "Tello ROS 2 nodes and RViz2 stopped."
