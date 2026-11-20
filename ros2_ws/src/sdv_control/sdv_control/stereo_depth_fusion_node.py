import time, socket, struct, threading
import numpy as np
import cv2, torch
from PIL import Image
from transformers import AutoImageProcessor, AutoModelForDepthEstimation
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image as RosImage, CameraInfo

SYNC_MAGIC = b"\xAA\x55\xAA\x55"
BASELINE_M = 0.060
FOCAL_PX   = 265.0
OPTICAL_FRAME = "camera_optical_frame"

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
        self.pub_rgb   = self.create_publisher(RosImage, '/stereo/rgb/image_raw', 10)
        self.pub_depth = self.create_publisher(RosImage, '/stereo/depth/image_raw', 10)
        self.pub_info  = self.create_publisher(CameraInfo, '/stereo/rgb/camera_info', 10)

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
        self.get_logger().info("Stereo + AI Depth Fusion Node (RGB-D SLAM ready) Listening TCP:5015")

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
                    if not hdr: break
                    if hdr[:4] != SYNC_MAGIC: continue
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
        valid_mask = disp > 2.0
        stereo_depth_m = np.zeros_like(disp)
        stereo_depth_m[valid_mask] = (FOCAL_PX * BASELINE_M) / disp[valid_mask]

        rgb_l = cv2.cvtColor(img_l, cv2.COLOR_BGR2RGB)
        inputs = self.processor(images=Image.fromarray(rgb_l), return_tensors="pt").to(self.device)
        with torch.no_grad():
            pred = self.model(**inputs).predicted_depth
        ai_inv_depth = torch.nn.functional.interpolate(
            pred.unsqueeze(1), size=img_l.shape[:2], mode="bicubic", align_corners=False
        ).squeeze().cpu().numpy()

        cal_mask = valid_mask & (stereo_depth_m > 0.15) & (stereo_depth_m < 4.0)
        ai_inv_safe = np.maximum(ai_inv_depth, 1e-3)
        if np.count_nonzero(cal_mask) > 150:
            scale = float(np.median((1.0 / stereo_depth_m[cal_mask]) / ai_inv_safe[cal_mask]))
        else:
            scale = 0.045
        metric_depth_m = np.clip(1.0 / np.maximum(ai_inv_safe * scale, 0.1), 0.15, 6.0).astype(np.float32)

        h, w = metric_depth_m.shape
        roi = metric_depth_m[int(h*0.25):int(h*0.75), :]
        w3 = w // 3
        d_left   = float(np.percentile(roi[:, :w3], 15))
        d_center = float(np.percentile(roi[:, w3:2*w3], 15))
        d_right  = float(np.percentile(roi[:, 2*w3:], 15))
        cm = valid_mask[int(h*0.35):int(h*0.65), int(w*0.35):int(w*0.65)]
        sgbm_center = float(np.median(stereo_depth_m[int(h*0.35):int(h*0.65), int(w*0.35):int(w*0.65)][cm])) if np.any(cm) else -1.0
        infer_ms = (time.time() - t0) * 1000.0
        self.pub_zones.publish(Float32MultiArray(data=[d_left, d_center, d_right, sgbm_center, infer_ms]))

        # RTAB-Map 用 RGB + Depth(32FC1, m) + CameraInfo を同一スタンプで配信
        stamp = self.get_clock().now().to_msg()
        rgb_msg = RosImage()
        rgb_msg.header.stamp, rgb_msg.header.frame_id = stamp, OPTICAL_FRAME
        rgb_msg.height, rgb_msg.width, rgb_msg.encoding, rgb_msg.step = h, w, "rgb8", w * 3
        rgb_msg.data = rgb_l.tobytes()
        self.pub_rgb.publish(rgb_msg)

        dep_msg = RosImage()
        dep_msg.header.stamp, dep_msg.header.frame_id = stamp, OPTICAL_FRAME
        dep_msg.height, dep_msg.width, dep_msg.encoding, dep_msg.step = h, w, "32FC1", w * 4
        dep_msg.data = metric_depth_m.tobytes()
        self.pub_depth.publish(dep_msg)

        cx, cy = w / 2.0, h / 2.0
        ci = CameraInfo()
        ci.header.stamp, ci.header.frame_id = stamp, OPTICAL_FRAME
        ci.height, ci.width, ci.distortion_model = h, w, "plumb_bob"
        ci.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        ci.k = [FOCAL_PX, 0.0, cx, 0.0, FOCAL_PX, cy, 0.0, 0.0, 1.0]
        ci.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        ci.p = [FOCAL_PX, 0.0, cx, 0.0, 0.0, FOCAL_PX, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        self.pub_info.publish(ci)

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
