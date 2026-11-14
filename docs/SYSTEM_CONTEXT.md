# SDV 開発環境 システムコンテキスト (`qnx-3d-hmi-cluster`)

## 1. ノード・ネットワーク・VM 構成
- **Proxmox Host**: `192.168.0.150` (`devuser` / 作業リポジトリ: `~/qnx-3d-hmi-cluster`)[cite: 6]
- **QNX 8.0 Safety Island (VM 210: `qnx8-vm`)**: `10.0.0.169`[cite: 6]
  - UDP `5006`: Edge（Raspi 5）からのセンサー入力受信
  - UDP `5008`: ROS 2 ADAS ノードからの介入指令受信
  - UDP `5005`: Ubuntu HMI へ 60Hz テレメトリ送信（16バイト `TelemetryPacket`）
- **Ubuntu GPU HMI & ROS 2 (VM 220: `ubuntu-gpu-hmi`)**: `10.0.0.155` (RTX 4070 / X11 `:0` / ROS 2 Humble)[cite: 6]
  - UDP `5005`: QNX 60Hz テレメトリ受信ポート
  - UDP `5009` (ローカル `127.0.0.1`): ROS 2 `/vehicle/odom` リアルタイム同期ポート（実装・検証済）
- **Kubernetes Cluster (VM 300: `k8s-node-01`)**: `10.0.0.100`[cite: 6]
  - EMQX MQTT Broker VIP: `10.0.0.201:1883`（トピック: `devices/sdv-ros2/telemetry`）[cite: 5]
  - PostgreSQL NodePort: `10.0.0.100:30432`（DB: `iot_data`, User: `mqtt_user`, Pass: `mqtt_password`）
  - 注意: K8s VM 停止時は DB 履歴取得・EMQX 送信は自動スキップされ、ローカル動作に影響しない。

## 2. データ仕様・プロトコル
1. **QNX 60Hz テレメトリ (`UDP 5005`, 16 bytes, Little-Endian)**
   - `uint32_t seq`, `float angle` (deg), `float speed` (km/h), `uint32_t mode` (0: NORMAL, 1: WARNING, 2: EMERGENCY)
   - 用途: 60FPS 描画ループ駆動、スピード/タコメーター、3D 車両モデル姿勢 (`currentAngle - 90.0f` 補正)、警告色シェーダー制御。
2. **ROS 2 Humble (`ros2_ws/src/sdv_control`) & ローカル同期 (`UDP 5009`, 実装済)**
   - `emqx_bridge_node.py` が `odom_topic`（デフォルト `'/vehicle/odom'`、`nav_msgs/msg/Odometry`）を購読し、受信のたびに `127.0.0.1:5009` へ JSON (`{"x": ..., "y": ..., "yaw_deg": ...}`) をノンブロッキング UDP 送信する。
   - K8s 稼働時は同座標を 5Hz で EMQX (`10.0.0.201:1883`) へ送信し PostgreSQL に永続化する[cite: 5]。
3. **Qt6 / OpenGL HMI (`hmi_cluster`) 軌跡復元 ＆ リアルタイム同期 (実装済)**
   - **アプローチ 1 (起動時履歴復元)**: `fetch_db_history.py` が PostgreSQL `telemetry` (`payload JSONB`) から `LAG()` ウィンドウ関数で移動履歴（最大 500 点）を `/tmp/odom_history.csv` へ抽出し、`GLWidget::loadTrajectoryHistory()` がロードする（`L` キー再読込 / `R` キーリセット）。
   - **アプローチ 2 (リアルタイム同期)**: `QUdpSocket` で `127.0.0.1:5009` を受信し、絶対座標 `(x, y)` を `odomX`, `odomY` および `trajectoryPoints` に直接反映する。UDP `5009` が 1.0 秒以上途絶えた場合のみ QNX 速度積分へ自動フォールバックする。

## 3. 設計・実装の絶対制約
- **疎結合の維持**: Qt6 (`hmi_cluster/CMakeLists.txt`) に ROS 2 (`rclcpp`) 依存を追加しないこと。ROS 2 / Nav2 / Gazebo / Unity と HMI の連携は常に `127.0.0.1:5009` を介して行う。
- **ホスト保護**: エージェントは `sudo`, `qm`, `kubectl`, `git push`, `pkill` 等のシステムコマンドを実行しないこと。
