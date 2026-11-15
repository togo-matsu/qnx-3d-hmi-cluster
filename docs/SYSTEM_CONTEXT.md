# SDV 開発環境 システムコンテキスト (`qnx-3d-hmi-cluster`)

## 1. ノード・ネットワーク・VM 構成
- **Proxmox Host**: `192.168.0.150` (`devuser` / 作業リポジトリ: `~/qnx-3d-hmi-cluster`)
- **QNX 8.0 Safety Island (VM 210: `qnx8-vm`)**: `10.0.0.169`
  - UDP `5006`: Edge（Raspi 5）および ROS 2 シミュレーション中継（`sim_qnx_bridge_node`）からのセンサー/指令入力受信
  - UDP `5008`: ROS 2 ADAS ノードからの介入指令受信
  - UDP `5005`: Ubuntu HMI へ 60Hz テレメトリ送信（16バイト `TelemetryPacket`）
- **Ubuntu GPU HMI & ROS 2 (VM 220: `ubuntu-gpu-hmi`)**: `10.0.0.155` (RTX 4070 / X11 `:0` / ROS 2 Humble)
  - UDP `5005`: QNX 60Hz テレメトリ受信ポート
  - UDP `5009` (ローカル `127.0.0.1`): ROS 2 オドメトリ（`/vehicle/odom` または `/odom`）リアルタイム同期ポート（実装・検証済）
- **Kubernetes Cluster (VM 300: `k8s-node-01`)**: `10.0.0.100`
  - EMQX MQTT Broker VIP: `10.0.0.201:1883`（トピック: `devices/sdv-ros2/telemetry`）
  - PostgreSQL NodePort: `10.0.0.100:30432`（DB: `iot_data`, User: `mqtt_user`, Pass: `mqtt_password`）
  - 注意: K8s VM 停止時は DB 履歴取得・EMQX 送信は自動スキップされ、ローカル動作に影響しない。

## 2. データ仕様・プロトコル
1. **QNX 入力パケット (`UDP 5006`, 16 bytes, Little-Endian `<fffI`)**
   - `float angle` (0.0〜360.0 deg), `float speed` (km/h), `float rpm` (800〜8000), `uint32_t mode` (0: NORMAL, 1: WARNING, 2: EMERGENCY)
   - 実機 HIL 時は Raspi 5 (`hmi_edge_bridge.py`)、シミュレーション時は `sim_qnx_bridge_node.py` が 50Hz で送信する。
2. **QNX 60Hz テレメトリ (`UDP 5005`, 16 bytes, Little-Endian)**
   - `uint32_t seq`, `float angle` (deg), `float speed` (km/h), `uint32_t mode` (0: NORMAL, 1: WARNING, 2: EMERGENCY)
   - 用途: 60FPS 描画ループ駆動、スピード/タコメーター、3D 車両モデル姿勢 (`currentAngle - 90.0f` 補正)、警告色シェーダー制御。
3. **ROS 2 Humble (`ros2_ws/src/sdv_control`) & シミュレーション連携 (実装・検証済)**
   - `emqx_bridge_node.py`: `odom_topic`（デフォルト `'/vehicle/odom'`、Gazebo/Nav2 連携時は `'/odom'`）を購読し、受信のたびに `127.0.0.1:5009` へ JSON (`{"x": ..., "y": ..., "yaw_deg": ...}`) をノンブロッキング UDP 送信する。K8s 稼働時は同座標を 5Hz で EMQX (`10.0.0.201:1883`) へ送信する。
   - `sim_qnx_bridge_node.py`: Gazebo / Nav2 の `/cmd_vel` (`geometry_msgs/msg/Twist`) と `/odom` (`nav_msgs/msg/Odometry`) を購読し、車速（`speed_scale` 倍率適用、デフォルト `150.0`）・車体ヨー角・疑似 RPM を 50Hz で `10.0.0.169:5006` へ UDP 送信する。
4. **Gazebo & Nav2 シミュレーション構成 (STEP 1 / STEP 2 検証済)**
   - `TURTLEBOT3_MODEL=waffle` を使用し、`turtlebot3_gazebo/turtlebot3_world.launch.py` と `nav2_bringup/bringup_launch.py` (`use_sim_time:=True`) を分離起動して `odom` TF と `/navigate_to_pose` による自律走行を実行する。
5. **Qt6 / OpenGL HMI (`hmi_cluster`) 軌跡復元 ＆ リアルタイム同期 (実装済)**
   - **アプローチ 1 (起動時履歴復元)**: `fetch_db_history.py` が PostgreSQL `telemetry` (`payload JSONB`) から `LAG()` ウィンドウ関数で移動履歴（最大 500 点）を `/tmp/odom_history.csv` へ抽出し、`GLWidget::loadTrajectoryHistory()` がロードする（`L` キー再読込 / `R` キーリセット）。
   - **アプローチ 2 (リアルタイム同期)**: `QUdpSocket` で `127.0.0.1:5009` を受信し、絶対座標 `(x, y)` を `odomX`, `odomY` および `trajectoryPoints` に直接反映する。UDP `5009` が 1.0 秒以上途絶えた場合のみ QNX 速度積分へ自動フォールバックする。

## 3. 設計・実装の絶対制約
- **疎結合の維持**: Qt6 (`hmi_cluster/CMakeLists.txt`) に ROS 2 (`rclcpp`) 依存を追加しないこと。ROS 2 / Nav2 / Gazebo / Unity と HMI の連携は、位置軌跡は `127.0.0.1:5009`、車速・姿勢は QNX (`10.0.0.169:5006` -> `5005`) を介して行う。
- **ホスト保護**: エージェントは `sudo`, `qm`, `kubectl`, `git push`, `pkill` 等のシステムコマンドを実行しないこと。
