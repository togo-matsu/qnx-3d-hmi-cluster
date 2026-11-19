import time, socket, struct, threading
import numpy as np
import cv2, torch
from PIL import Image
from transformers import AutoImageProcessor, AutoModelForDepthEstimation
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image as RosImage

SYNC_MAGIC = b"\xAA\x55\xAA\x55"
BASELINE_M = 0.060
FOCAL_PX   = 265.0

def recv_exact(conn, n):
    buf = bytearray()
    while len(buf) < n:
        chunk = conn.recv(n - len(buf))
        if not chunk:
            return None
        buf.extend(chunk)
    return bytes(buf)

class StereoDepthFusionNode(Node):
    def __init__(self):
        super().__init__('stereo_depth_fusion_node')
        self.pub_zones = self.create_publisher(Float32MultiArray, '/stereo/zone_distances_m', 10)
        self.pub_debug = self.create_publisher(RosImage, '/stereo/debug_view', 10)
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.get_logger().info(f"Loading Depth-Anything-V2-Small on {self.device}...")
        model_id = "depth-anything/Depth-Anything-V2-Small-hf"
        self.processor = AutoImageProcessor.from_pretrained(model_id)
        self.model = AutoModelForDepthEstimation.from_pretrained(model_id).to(self.device).eval()
        self.stereo = cv2.StereoSGBM_create(
            minDisparity=0, numDisparities=64, blockSize=7,
            P1=8*3*7*7, P2=32*3*7*7, disp12MaxDiff=1,
            uniquenessRatio=10, speckleWindowSize=100, speckleRange=32
        )
        self.latest_pair = None
        self.pair_lock = threading.Lock()
        threading.Thread(target=self.tcp_server, daemon=True).start()
        self.create_timer(0.08, self.process_frame)
        self.get_logger().info("Stereo + AI Depth Fusion Node ready! Listening TCP:5015")

    def tcp_server(self):
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind(("0.0.0.0", 5015))
        srv.listen(1)
        while True:
            conn, addr = srv.accept()
            self.get_logger().info(f"Raspi 5 Stereo Stream Connected from {addr}")
            try:
                while True:
                    hdr = recv_exact(conn, 12)
                    if not hdr or hdr[:4] != SYNC_MAGIC:
                        break
                    len_l, len_r = struct.unpack("<II", hdr[4:12])
                    jl = recv_exact(conn, len_l)
                    jr = recv_exact(conn, len_r)
                    if jl and jr:
                        with self.pair_lock:
                            self.latest_pair = (jl, jr)
            except OSError:
                pass
            finally:
                conn.close()

    def process_frame(self):
        with self.pair_lock:
            pair = self.latest_pair
            self.latest_pair = None
        if pair is None:
            return
        t0 = time.time()
        img_l = cv2.imdecode(np.frombuffer(pair[0], np.uint8), cv2.IMREAD_COLOR)
        img_r = cv2.imdecode(np.frombuffer(pair[1], np.uint8), cv2.IMREAD_COLOR)
        if img_l is None or img_r is None:
            return

        gray_l = cv2.cvtColor(img_l, cv2.COLOR_BGR2GRAY)
        gray_r = cv2.cvtColor(img_r, cv2.COLOR_BGR2GRAY)
        disp = self.stereo.compute(gray_l, gray_r).astype(np.float32) / 16.0
        valid = disp > 2.0
        stereo_m = np.zeros_like(disp)
        stereo_m[valid] = (FOCAL_PX * BASELINE_M) / disp[valid]

        rgb_l = cv2.cvtColor(img_l, cv2.COLOR_BGR2RGB)
        inputs = self.processor(images=Image.fromarray(rgb_l), return_tensors="pt").to(self.device)
        with torch.no_grad():
            pred = self.model(**inputs).predicted_depth
        ai_inv = torch.nn.functional.interpolate(
            pred.unsqueeze(1), size=img_l.shape[:2], mode="bicubic", align_corners=False
        ).squeeze().cpu().numpy()

        cal = valid & (stereo_m > 0.15) & (stereo_m < 4.0)
        ai_safe = np.maximum(ai_inv, 1e-3)
        scale = float(np.median((1.0 / stereo_m[cal]) / ai_safe[cal])) if np.count_nonzero(cal) > 150 else 0.045
        metric_m = np.clip(1.0 / np.maximum(ai_safe * scale, 0.1), 0.15, 6.0)

        h, w = metric_m.shape
        roi = metric_m[int(h*0.25):int(h*0.75), :]
        w3 = w // 3
        d_l = float(np.percentile(roi[:, :w3], 15))
        d_c = float(np.percentile(roi[:, w3:2*w3], 15))
        d_r = float(np.percentile(roi[:, 2*w3:], 15))
        c_roi_m = stereo_m[int(h*0.35):int(h*0.65), int(w*0.35):int(w*0.65)]
        c_roi_v = valid[int(h*0.35):int(h*0.65), int(w*0.35):int(w*0.65)]
        sgbm_c = float(np.median(c_roi_m[c_roi_v])) if np.any(c_roi_v) else -1.0

        infer_ms = (time.time() - t0) * 1000.0
        self.pub_zones.publish(Float32MultiArray(data=[d_l, d_c, d_r, sgbm_c, infer_ms]))

        vis_sgbm = cv2.applyColorMap(np.clip(disp * 4, 0, 255).astype(np.uint8), cv2.COLORMAP_JET)
        vis_ai = cv2.applyColorMap(np.clip((1.0 / metric_m) * 80, 0, 255).astype(np.uint8), cv2.COLORMAP_INFERNO)
        combo = np.hstack([img_l, vis_sgbm, vis_ai])
        cv2.putText(combo, f"L:{d_l:.2f}m C:{d_c:.2f}m R:{d_r:.2f}m (SGBM:{sgbm_c:.2f}m) [{infer_ms:.1f}ms]",
                    (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (0, 255, 0), 2)
        cv2.imwrite("/tmp/stereo_fusion_latest.jpg", combo)

        msg = RosImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.height, msg.width = combo.shape[:2]
        msg.encoding, msg.step = "bgr8", msg.width * 3
        msg.data = combo.tobytes()
        self.pub_debug.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = StereoDepthFusionNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
