import json, math, socket, threading, time
import av, rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float32, String
from sensor_msgs.msg import Image, Imu
from geometry_msgs.msg import Twist, TransformStamped
from tf2_ros import TransformBroadcaster
from cv_bridge import CvBridge

CTRL_PORT = 5020
VIDEO_PORT = 5021
MAX_RC_SPEED = 30  # 室内安全上限 (最大30%)

class TelloRos2Node(Node):
    def __init__(self):
        super().__init__('tello_ros2_node')
        self.pub_img = self.create_publisher(Image, '/tello/image_raw', 10)
        self.pub_imu = self.create_publisher(Imu, '/tello/imu', 20)
        self.pub_bat = self.create_publisher(Int32, '/tello/battery', 10)
        self.pub_h = self.create_publisher(Float32, '/tello/height_m', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.bridge = CvBridge()

        self.create_subscription(String, '/tello/command', self.on_command, 10)
        self.create_subscription(Twist, '/tello/cmd_vel', self.on_cmd_vel, 10)

        self.client_lock = threading.Lock()
        self.client_sock = None

        threading.Thread(target=self.tcp_server_loop, daemon=True).start()
        threading.Thread(target=self.udp_video_loop, daemon=True).start()
        self.get_logger().info(f'Tello ROS 2 Node ready (TCP:{CTRL_PORT}, UDP:{VIDEO_PORT})')

    def send_bridge_cmd(self, cmd: str):
        payload = (json.dumps({'cmd': cmd}) + '\n').encode('utf-8')
        with self.client_lock:
            sock = self.client_sock
        if sock is None:
            self.get_logger().warn(f'Cannot send "{cmd}": Raspi 5 bridge not connected')
            return
        try:
            sock.sendall(payload)
        except OSError as e:
            self.get_logger().warn(f'Send failed: {e}')

    def on_command(self, msg: String):
        cmd = msg.data.strip().lower()
        if cmd == 'hover':
            self.send_bridge_cmd('rc 0 0 0 0')
        elif cmd in ('takeoff', 'land', 'emergency'):
            self.get_logger().info(f'Command -> {cmd}')
            self.send_bridge_cmd(cmd)

    def on_cmd_vel(self, msg: Twist):
        def clamp(val):
            return max(-MAX_RC_SPEED, min(MAX_RC_SPEED, int(round(val * MAX_RC_SPEED))))
        # ROS REP-103 (+X:前, +Y:左, +Z:上, +Yaw:左旋回) -> Tello RC (a:右, b:前, c:上, d:右旋回)
        a = clamp(-msg.linear.y)
        b = clamp(msg.linear.x)
        c = clamp(msg.linear.z)
        d = clamp(-msg.angular.z)
        self.send_bridge_cmd(f'rc {a} {b} {c} {d}')

    def publish_state(self, st: dict):
        now = self.get_clock().now().to_msg()
        bat = int(st.get('bat', 0))
        h_m = float(st.get('h', 0)) / 100.0

        m_bat = Int32(); m_bat.data = bat
        self.pub_bat.publish(m_bat)

        m_h = Float32(); m_h.data = h_m
        self.pub_h.publish(m_h)

        # 姿勢角 (deg -> rad, ROS REP-103)
        r = math.radians(float(st.get('roll', 0.0)))
        p = math.radians(float(st.get('pitch', 0.0)))
        y = math.radians(-float(st.get('yaw', 0.0)))
        cr, sr = math.cos(r * 0.5), math.sin(r * 0.5)
        cp, sp = math.cos(p * 0.5), math.sin(p * 0.5)
        cy, sy = math.cos(y * 0.5), math.sin(y * 0.5)
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        qw = cr * cp * cy + sr * sp * sy

        imu = Imu()
        imu.header.stamp = now
        imu.header.frame_id = 'tello_base_link'
        imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w = qx, qy, qz, qw
        imu.linear_acceleration.x = float(st.get('agx', 0.0)) * 0.00981
        imu.linear_acceleration.y = -float(st.get('agy', 0.0)) * 0.00981
        imu.linear_acceleration.z = -float(st.get('agz', -1000.0)) * 0.00981
        self.pub_imu.publish(imu)

        tf = TransformStamped()
        tf.header.stamp = now
        tf.header.frame_id = 'odom'
        tf.child_frame_id = 'tello_base_link'
        tf.transform.translation.z = h_m
        tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w = qx, qy, qz, qw
        self.tf_broadcaster.sendTransform(tf)

    def tcp_server_loop(self):
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind(('0.0.0.0', CTRL_PORT))
        srv.listen(1)
        while rclpy.ok():
            conn, addr = srv.accept()
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            with self.client_lock:
                self.client_sock = conn
            self.get_logger().info(f'Raspi 5 Tello Bridge connected from {addr}')
            buf = b''
            try:
                while rclpy.ok():
                    chunk = conn.recv(2048)
                    if not chunk:
                        break
                    buf += chunk
                    while b'\n' in buf:
                        line, buf = buf.split(b'\n', 1)
                        try:
                            st = json.loads(line.decode('utf-8', errors='ignore'))
                            self.publish_state(st)
                        except json.JSONDecodeError:
                            pass
            except OSError:
                pass
            finally:
                with self.client_lock:
                    if self.client_sock is conn:
                        self.client_sock = None
                conn.close()
                self.get_logger().warn('Raspi 5 Tello Bridge disconnected')

    def udp_video_loop(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(('0.0.0.0', VIDEO_PORT))
        codec = av.CodecContext.create('h264', 'r')
        while rclpy.ok():
            try:
                data, _ = sock.recvfrom(4096)
                for pkt in codec.parse(data):
                    for frame in codec.decode(pkt):
                        img = frame.to_ndarray(format='bgr24')
                        msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
                        msg.header.stamp = self.get_clock().now().to_msg()
                        msg.header.frame_id = 'tello_camera_frame'
                        self.pub_img.publish(msg)
            except Exception:
                pass

def main(args=None):
    rclpy.init(args=args)
    node = TelloRos2Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.ok() and rclpy.shutdown()
