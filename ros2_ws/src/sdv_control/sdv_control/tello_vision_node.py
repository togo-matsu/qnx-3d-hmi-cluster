import json, threading, time
import cv2, torch, rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32, String
from cv_bridge import CvBridge
from ultralytics import YOLO

CONF_THRESH = 0.45
CORRIDOR_X_LIMIT = 0.75

# 人物追従・安全距離 (画面占有率 Area 基準)
TARGET_AREA_MIN = 0.18     # これ未満(約1.8m以上)で前進 (+vX)
TARGET_AREA_MAX = 0.28     # これ超過(約1.2m以内)で後退 (-vX)
PERSON_WARN_AREA = 0.34    # 約1.0mで黄バナー (前進ブロック)
PERSON_DANGER_AREA = 0.48  # 約0.6mで赤バナー (QNX E-STOP 自動着陸)

# 人物以外の障害物しきい値
OBS_WARN_H = 0.58
OBS_WARN_AREA = 0.25
OBS_DANGER_H = 0.78
OBS_DANGER_AREA = 0.42

DEADBAND_X = 0.10
MIN_FWD_DRIVE = 0.62       # 前進・後退がホバリング制御を超える最小出力 (rc=28相当)
MAX_FWD_SPEED = 0.85       # 最大前後出力 (rc=38相当)
MIN_YAW_DRIVE = 0.38       # 旋回の最小出力 (rc=17相当)
MAX_YAW_RATE = 0.60        # 最大旋回出力 (rc=27相当)

def apply_deadband_comp(val: float, min_out: float, max_out: float) -> float:
    if abs(val) < 1e-4:
        return 0.0
    sign = 1.0 if val > 0.0 else -1.0
    mag = max(min_out, min(max_out, abs(val)))
    return sign * mag

class TelloVisionNode(Node):
    def __init__(self):
        super().__init__('tello_vision_node')
        self.bridge = CvBridge()
        self.device = 0 if torch.cuda.is_available() else 'cpu'
        self.get_logger().info(f'Loading YOLOv8n on device={self.device}...')
        self.model = YOLO('yolov8n.pt')

        self.pub_debug = self.create_publisher(Image, '/tello/vision/debug_image', 5)
        self.pub_vel = self.create_publisher(Twist, '/tello/vision/target_vel', 10)
        self.pub_risk = self.create_publisher(Int32, '/tello/vision/risk_level', 10)
        self.pub_status = self.create_publisher(String, '/tello/vision/status', 10)

        self.create_subscription(Image, '/tello/image_raw', self.on_image, 5)

        self.frame_lock = threading.Lock()
        self.latest_frame = None
        self.frame_seq = 0
        self.danger_streak = 0
        self.last_danger_log_ts = 0.0
        self.last_person = None
        self.last_person_ts = 0.0

        threading.Thread(target=self.infer_loop, daemon=True).start()
        self.get_logger().info('Tello Vision Node ready (YOLOv8n Area-Based Visual Servo)')

    def on_image(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception:
            return
        with self.frame_lock:
            self.latest_frame = frame
            self.frame_seq += 1

    def infer_loop(self):
        last_seq = -1
        while rclpy.ok():
            with self.frame_lock:
                if self.latest_frame is None or self.frame_seq == last_seq:
                    frame = None
                else:
                    frame = self.latest_frame.copy()
                    last_seq = self.frame_seq

            if frame is None:
                time.sleep(0.01)
                continue

            now = time.time()
            h, w = frame.shape[:2]
            frame_area = float(h * w)
            res = self.model.predict(frame, device=self.device, conf=CONF_THRESH, verbose=False)[0]

            best_person = None
            best_person_area = 0.0
            raw_risk = 0
            danger_label = ''
            danger_metric = 0.0

            for box in res.boxes:
                cls_id = int(box.cls[0].item())
                conf = float(box.conf[0].item())
                x1, y1, x2, y2 = [int(v) for v in box.xyxy[0].tolist()]
                bw = max(1, x2 - x1)
                bh = max(1, y2 - y1)
                cx = 0.5 * (x1 + x2)
                norm_x = abs(cx - 0.5 * w) / (0.5 * w)
                h_ratio = bh / float(h)
                area_ratio = (bw * bh) / frame_area
                label = self.model.names.get(cls_id, str(cls_id))

                if norm_x <= CORRIDOR_X_LIMIT:
                    if cls_id == 0:
                        box_risk = 2 if area_ratio >= PERSON_DANGER_AREA else (1 if area_ratio >= PERSON_WARN_AREA else 0)
                        metric = area_ratio
                    else:
                        if h_ratio >= OBS_DANGER_H or area_ratio >= OBS_DANGER_AREA:
                            box_risk = 2
                        elif h_ratio >= OBS_WARN_H or area_ratio >= OBS_WARN_AREA:
                            box_risk = 1
                        else:
                            box_risk = 0
                        metric = max(h_ratio, area_ratio)

                    if box_risk > raw_risk or (box_risk == raw_risk and box_risk > 0 and metric > danger_metric):
                        raw_risk = box_risk
                        danger_label = label
                        danger_metric = metric

                color = (0, 220, 0) if cls_id == 0 else (200, 180, 0)
                cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                cv2.putText(frame, f'{label} {conf:.2f} A:{area_ratio:.2f}', (x1, max(18, y1 - 6)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)

                if cls_id == 0 and area_ratio > best_person_area:
                    best_person_area = area_ratio
                    best_person = (x1, y1, x2, y2, area_ratio)

            if raw_risk == 2:
                self.danger_streak += 1
            else:
                self.danger_streak = 0

            if self.danger_streak >= 3:
                risk = 2
                if now - self.last_danger_log_ts >= 2.0:
                    self.last_danger_log_ts = now
                    self.get_logger().warn(f'[DANGER TRIGGER] label={danger_label} metric={danger_metric:.2f}')
            elif raw_risk >= 1:
                risk = 1
            else:
                risk = 0

            is_hold = False
            if best_person is not None:
                self.last_person = best_person
                self.last_person_ts = now
                active_person = best_person
            elif self.last_person is not None and (now - self.last_person_ts) <= 0.35:
                active_person = self.last_person
                is_hold = True
            else:
                active_person = None

            lx, az = 0.0, 0.0
            err_x = 0.0
            p_area = 0.0
            if active_person is not None and risk < 2:
                x1, y1, x2, y2, p_area = active_person
                cx = 0.5 * (x1 + x2)
                cy = 0.5 * (y1 + y2)
                err_x = (cx - 0.5 * w) / (0.5 * w)

                if abs(err_x) > DEADBAND_X:
                    az = apply_deadband_comp(-err_x * 0.90, MIN_YAW_DRIVE, MAX_YAW_RATE)
                    if is_hold:
                        az *= 0.8

                if not is_hold:
                    if p_area < TARGET_AREA_MIN and risk == 0:
                        raw_lx = (TARGET_AREA_MIN - p_area) * 4.5
                        lx = apply_deadband_comp(raw_lx, MIN_FWD_DRIVE, MAX_FWD_SPEED)
                    elif p_area > TARGET_AREA_MAX:
                        raw_lx = (TARGET_AREA_MAX - p_area) * 4.5
                        lx = apply_deadband_comp(raw_lx, MIN_FWD_DRIVE, MAX_FWD_SPEED)

                if not is_hold:
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 160, 0), 3)
                    cv2.circle(frame, (int(cx), int(cy)), 6, (255, 160, 0), -1)
                    cv2.line(frame, (w // 2, h // 2), (int(cx), int(cy)), (255, 160, 0), 2)

            margin_x = int(0.5 * w * (1.0 - CORRIDOR_X_LIMIT))
            cv2.line(frame, (margin_x, 38), (margin_x, h - 4), (180, 180, 180), 1)
            cv2.line(frame, (w - margin_x, 38), (w - margin_x, h - 4), (180, 180, 180), 1)
            cv2.drawMarker(frame, (w // 2, h // 2), (255, 255, 255), cv2.MARKER_CROSS, 20, 1)

            banner_cfg = {
                0: ((0, 150, 0), f'SAFE (A:{p_area:.2f})'),
                1: ((0, 165, 255), f'WARNING ({danger_label} {danger_metric:.2f})'),
                2: ((0, 0, 220), f'DANGER E-STOP ({danger_label} {danger_metric:.2f})'),
            }
            bg, title = banner_cfg[risk]
            cv2.rectangle(frame, (0, 0), (w, 34), bg, -1)
            fps = 1.0 / max(1e-3, time.time() - now)
            cv2.putText(frame, f'{title} | vX:{lx:+.2f} yaw:{az:+.2f} | {fps:.1f} FPS',
                        (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.56, (255, 255, 255), 2, cv2.LINE_AA)

            m_risk = Int32(); m_risk.data = risk
            self.pub_risk.publish(m_risk)

            m_vel = Twist()
            m_vel.linear.x = float(lx)
            m_vel.angular.z = float(az)
            self.pub_vel.publish(m_vel)

            m_st = String()
            m_st.data = json.dumps({
                'risk': risk, 'person': best_person is not None,
                'area': round(p_area, 3), 'err_x': round(err_x, 3),
                'lx': round(lx, 3), 'az': round(az, 3), 'fps': round(fps, 1),
            })
            self.pub_status.publish(m_st)

            img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            img_msg.header.stamp = self.get_clock().now().to_msg()
            img_msg.header.frame_id = 'tello_camera_frame'
            self.pub_debug.publish(img_msg)

def main(args=None):
    rclpy.init(args=args)
    node = TelloVisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass
