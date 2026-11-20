#!/bin/bash
set -e

# 1. 自動サスペンド無効化と既存プロセス整理
gsettings set org.gnome.settings-daemon.plugins.power sleep-inactive-ac-type 'nothing' 2>/dev/null || true
pkill -9 -f "gzserver|gzclient|component_container|nav2|rviz2|bringup_launch|amcl|map_server|bt_navigator|controller_server|planner_server|smoother_server|behavior_server|waypoint_follower|velocity_smoother|lifecycle_manager|telemetry_node|pillar_nav_node|sim_qnx_bridge_node|emqx_bridge_node" 2>/dev/null || true
sleep 2

# 2. X11 認証クッキーの同期
AUTH_FILE=$(ps aux | grep -oP '(?<=-auth )\S+' | head -n 1)
if [ -n "$AUTH_FILE" ] && [ -f "$AUTH_FILE" ]; then
    sudo cp "$AUTH_FILE" /home/ubuntu/.Xauthority
    sudo chown ubuntu:ubuntu /home/ubuntu/.Xauthority
fi

source /opt/ros/humble/setup.bash
source ~/qnx-3d-hmi-cluster/ros2_ws/install/setup.bash
export DISPLAY=:0 XAUTHORITY=/home/ubuntu/.Xauthority TURTLEBOT3_MODEL=waffle
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models

# 3. Gazebo を起動し /clock と /scan の配信完了まで自動待機
nohup ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py > /tmp/gazebo.log 2>&1 &
echo "[1/4] Waiting for Gazebo (/clock, /scan)..."
until ros2 topic list 2>/dev/null | grep -q "^/scan$"; do sleep 1; done
sleep 2

# 4. Nav2 を起動し amcl の有効化完了まで自動待機
nohup ros2 launch nav2_bringup bringup_launch.py use_sim_time:=True map:=/opt/ros/humble/share/nav2_bringup/maps/turtlebot3_world.yaml > /tmp/nav2.log 2>&1 &
echo "[2/4] Waiting for Nav2 (/amcl active)..."
until timeout 3 ros2 lifecycle get /amcl 2>/dev/null | grep -q "active \[3\]"; do sleep 1; done

# 5. 初期スポーン座標 (-2.0, -0.5) を送信し全サーバーを確実に active [3] に遷移
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped '{header: {frame_id: "map"}, pose: {pose: {position: {x: -2.0, y: -0.5, z: 0.0}, orientation: {w: 1.0}}}}' >/dev/null
sleep 4
for n in map_server amcl controller_server smoother_server planner_server behavior_server bt_navigator waypoint_follower velocity_smoother; do
    st=$(timeout 5 ros2 lifecycle get /$n 2>/dev/null || true)
    [[ "$st" == *"inactive"* ]] && timeout 5 ros2 lifecycle set /$n activate >/dev/null 2>&1 || true
done
echo "[3/4] bt_navigator status: $(timeout 5 ros2 lifecycle get /bt_navigator)"

# 6. telemetry_node (/tf 分離済) と各ブリッジノードを起動
nohup ros2 run sdv_control telemetry_node --ros-args -p use_sim_time:=True -r /tf:=/tf_telemetry_unused > /tmp/telemetry.log 2>&1 &
nohup ros2 run sdv_control pillar_nav_node --ros-args -p use_sim_time:=True > /tmp/pillar_nav.log 2>&1 &
nohup ros2 run sdv_control sim_qnx_bridge_node > /tmp/sim_qnx.log 2>&1 &
nohup ros2 run sdv_control emqx_bridge_node --ros-args -p odom_topic:=/odom > /tmp/emqx_bridge.log 2>&1 &
echo "[4/4] All HIL & Nav2 nodes started! Monitoring /tmp/pillar_nav.log..."
sleep 1
cat /tmp/pillar_nav.log
