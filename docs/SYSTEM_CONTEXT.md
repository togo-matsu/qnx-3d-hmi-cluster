# SDV 開発環境 システムコンテキスト (`qnx-3d-hmi-cluster`)

## 1. ノード・ネットワーク構成
- **Proxmox Host**: `192.168.0.150` (`devuser` / 作業リポジトリ: `~/qnx-3d-hmi-cluster`)
- **QNX 8.0 Safety Island (VM 210)**: `10.0.0.169`
  - UDP `5006`: Edge（Raspi 5）からのセンサー入力受信
  - UDP `5008`: ROS 2 ADAS ノードからの介入指令受信[cite: 10]
  - UDP `5005`: Ubuntu HMI へ 60Hz テレメトリ送信（16バイト `TelemetryPacket`）[cite: 10]
- **Ubuntu GPU HMI & ROS 2 (VM 220)**: `10.0.0.155` (RTX 4070 / X11 `:0` / ROS 2 Humble)[cite: 10]
  - UDP `5005`: QNX 60Hz テレメトリ受信ポート[cite: 10]
  - UDP `5009` (ローカル `127.0.0.1`): ROS 2 `/vehicle/odom` リアルタイム同期用ポート（アプローチ2）
- **Kubernetes Cluster (`k8s-node-01`)**: `10.0.0.100`
  - EMQX MQTT Broker VIP: `10.0.0.201:1883`（トピック: `devices/sdv-ros2/telemetry`）[cite: 10]
  - PostgreSQL NodePort: `10.0.0.100:30432`（DB: `iot_data`, User: `mqtt_user`, Pass: `mqtt_password`）[cite: 11]

## 2. データ仕様・プロトコル
1. **QNX 60Hz テレメトリ (`UDP 5005`, 16 bytes, Little-Endian)**[cite: 10]
   - `uint32_t seq`, `float angle` (deg), `float speed` (km/h), `uint32_t mode` (0: NORMAL, 1: WARNING, 2: EMERGENCY)[cite: 10]
   - 用途: 60FPS 描画ループ駆動、スピード/タコメーター、3D 車両モデル姿勢 (`currentAngle - 90.0f` 補正)、警告色シェーダー制御[cite: 10]。
2. **ROS 2 Humble (`ros2_ws/src/sdv_control`)**[cite: 10]
   - `/vehicle/odom` (`nav_msgs/msg/Odometry`): `pose.pose.position.x` (`odom_x`), `pose.pose.position.y` (`odom_y`), `pose.pose.orientation` (クォータニオン) を配信[cite: 10]。
   - `emqx_bridge_node.py` がこの座標を 5Hz で K8s EMQX へ送信し、PostgreSQL に永続化する[cite: 10]。
3. **PostgreSQL `telemetry` テーブル & 起動時履歴復元 (アプローチ1 実装済)**
   - カラム: `(id, device_id, topic, payload JSONB, created_at)`
   - `hmi_cluster/fetch_db_history.py` が起動時に `LAG()` ウィンドウ関数で移動履歴（最大500点）を `/tmp/odom_history.csv` (`odom_x,odom_y`) へ出力し、`GLWidget::loadTrajectoryHistory()` が初期軌跡としてロードする。

## 3. 設計・実装の絶対制約
- **疎結合の維持**: Qt6 (`hmi_cluster/CMakeLists.txt`) に ROS 2 (`rclcpp`) 依存を追加しないこと。ROS 2 と HMI の連携は同一マシン内のローカル UDP (`127.0.0.1:5009`) を使用する。
- **座標系の完全一致**: アプローチ1（DB履歴復元）とアプローチ2（リアルタイム走行・将来の Nav2/Gazebo/Unity 連携）の座標がズレないよう、ROS 2 `/vehicle/odom` の `(x, y)` を直接描画座標に採用する。
- **ホスト保護**: エージェントは `sudo`, `qm`, `kubectl`, `git push`, `pkill` 等のシステムコマンドを実行しないこと。
