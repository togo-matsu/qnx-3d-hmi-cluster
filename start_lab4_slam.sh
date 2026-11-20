#!/bin/bash
set -e

AUTH_FILE=$(ps aux | grep -oP '(?<=-auth )\S+' | head -n 1)
[ -n "$AUTH_FILE" ] && [ -f "$AUTH_FILE" ] && sudo cp "$AUTH_FILE" /home/ubuntu/.Xauthority && sudo chown ubuntu:ubuntu /home/ubuntu/.Xauthority
export DISPLAY=:0 XAUTHORITY=/home/ubuntu/.Xauthority

source /opt/ros/humble/setup.bash
pkill -f "static_transform_publisher|rgbd_odometry|rtabmap_slam" 2>/dev/null || true
sleep 1

# 1. base_link (X前・Y左・Z上) -> camera_optical_frame (Z前・X右・Y下)
nohup ros2 run tf2_ros static_transform_publisher \
  --x 0 --y 0 --z 0 --yaw -1.5708 --pitch 0 --roll -1.5708 \
  --frame-id base_link --child-frame-id camera_optical_frame > /tmp/slam_tf.log 2>&1 &

# 2. RTAB-Map RGB-D Odometry (6-DoF 自己位置推定 -> /odom)
nohup ros2 run rtabmap_odom rgbd_odometry --ros-args \
  -p frame_id:=base_link \
  -p odom_frame_id:=odom \
  -p approx_sync:=true \
  -p qos:=1 \
  -p "Vis/MinInliers:='10'" \
  -p "Odom/ResetCountdown:='1'" \
  -r rgb/image:=/stereo/rgb/image_raw \
  -r depth/image:=/stereo/depth/image_raw \
  -r rgb/camera_info:=/stereo/rgb/camera_info > /tmp/rtabmap_odom.log 2>&1 &

# 3. RTAB-Map 3D SLAM (3D 点群地図 -> /rtabmap/cloud_map)
nohup ros2 run rtabmap_slam rtabmap --delete_db_on_start --ros-args \
  -p frame_id:=base_link \
  -p subscribe_depth:=true \
  -p subscribe_rgb:=true \
  -p approx_sync:=true \
  -p qos:=1 \
  -p "Vis/MinInliers:='10'" \
  -p "Grid/FromDepth:='true'" \
  -p "Mem/IncrementalMemory:='true'" \
  -r rgb/image:=/stereo/rgb/image_raw \
  -r depth/image:=/stereo/depth/image_raw \
  -r rgb/camera_info:=/stereo/rgb/camera_info \
  -r odom:=/odom > /tmp/rtabmap_slam.log 2>&1 &

sleep 4
echo "=== rgbd_odometry log ==="
tail -n 8 /tmp/rtabmap_odom.log
echo "=== rtabmap_slam log ==="
tail -n 8 /tmp/rtabmap_slam.log
