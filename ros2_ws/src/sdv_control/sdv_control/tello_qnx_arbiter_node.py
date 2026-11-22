import math, socket, struct, threading, time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float32, Bool, String
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist

QNX_IP = '10.0.0.169'
QNX_EDGE_PORT = 5006
QNX_ADAS_PORT = 5008
QNX_RX_PORT = 5005

class TelloQnxArbiterNode(Node):
    def __init__(self):
        super().__init__('tello_qnx_arbiter_node')
        self.declare_parameter('auto_land_on_estop', True)
        self.auto_land = bool(self.get_parameter('auto_land_on_estop').value)

        self.pub_cmd = self.create_publisher(String, '/tello/command', 10)
        self.pub_vel = self.create_publisher(Twist, '/tello/cmd_vel', 10)
        self.pub_qnx_mode = self.create_publisher(Int32, '/tello/qnx_mode', 10)

        self.create_subscription(Int32, '/tello/vision/risk_level', self.on_risk, 10)
        self.create_subscription(Twist, '/tello/vision/target_vel', self.on_target_vel, 10)
        self.create_subscription(Int32, '/tello/battery', self.on_battery, 10)
        self.create_subscription(Imu, '/tello/imu', self.on_imu, 10)
        self.create_subscription(Float32, '/tello/height_m', self.on_height, 10)
        self.create_subscription(Bool, '/tello/follow_enable', self.on_follow_enable, 10)
        self.create_subscription(Bool, '/tello/safety_reset', self.on_safety_reset, 10)

        self.tx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.lock = threading.Lock()
        self.vision_risk = 0
        self.last_vision_ts = 0.0
        self.target_vel = Twist()
        self.last_vel_ts = 0.0
        self.battery = -1
        self.yaw_deg = 0.0
        self.height_m = 0.0

        self.qnx_seq = 0
        self.qnx_mode = 0
        self.last_qnx_rx_ts = 0.0

        self.follow_enabled = False
        self.estop_latched = False
        self.sent_zero = True
        self.last_land_pub_ts = 0.0
        self.last_warn_ts = 0.0

        threading.Thread(target=self.qnx_rx_loop, daemon=True).start()
        self.create_timer(0.05, self.control_loop)  # 20Hz
        self.get_logger().info(
            f'QNX Safety Arbiter ready (TX->{QNX_IP}:5006/5008, RX:5005, default=MONITOR, auto_land={self.auto_land})'
        )

    def on_risk(self, msg: Int32):
        with self.lock:
            self.vision_risk = int(msg.data)
            self.last_vision_ts = time.time()

    def on_target_vel(self, msg: Twist):
        with self.lock:
            self.target_vel = msg
            self.last_vel_ts = time.time()

    def on_battery(self, msg: Int32):
        with self.lock:
            self.battery = int(msg.data)

    def on_height(self, msg: Float32):
        with self.lock:
            self.height_m = float(msg.data)

    def on_imu(self, msg: Imu):
        q = msg.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = (math.degrees(math.atan2(siny, cosy)) + 360.0) % 360.0
        with self.lock:
            self.yaw_deg = yaw

    def on_follow_enable(self, msg: Bool):
        req = bool(msg.data)
        with self.lock:
            if req and self.estop_latched:
                self.get_logger().warn('[BLOCKED] Cannot enable FOLLOW while E-STOP is latched. Publish /tello/safety_reset true first.')
                return
            self.follow_enabled = req
            if not req:
                self.sent_zero = False
        if not req:
            self.pub_vel.publish(Twist())
            self.get_logger().info('[MODE] FOLLOW -> OFF (MONITOR mode)')
        else:
            self.get_logger().info('[MODE] FOLLOW -> ON (Low-Speed Visual Servo ACTIVE)')

    def on_safety_reset(self, msg: Bool):
        if not msg.data:
            return
        with self.lock:
            fresh_risk = self.vision_risk if (time.time() - self.last_vision_ts) < 1.0 else 0
            if self.qnx_mode == 2 or fresh_risk >= 2:
                self.get_logger().warn(f'[RESET DENIED] Clear obstacle / QNX Mode 2 before resetting (qnx={self.qnx_mode}, risk={fresh_risk}).')
                return
            self.estop_latched = False
            self.last_land_pub_ts = 0.0
        self.get_logger().info('[SAFETY RESET] E-STOP latch cleared. Ready in MONITOR mode.')

    def qnx_rx_loop(self):
        rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        if hasattr(socket, 'SO_REUSEPORT'):
            rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        rx.bind(('0.0.0.0', QNX_RX_PORT))
        rx.settimeout(0.5)
        while rclpy.ok():
            try:
                data, _ = rx.recvfrom(64)
                if len(data) >= 16:
                    seq, _angle, _speed, mode = struct.unpack('<IffI', data[:16])
                    with self.lock:
                        self.qnx_seq = seq
                        self.qnx_mode = int(mode)
                        self.last_qnx_rx_ts = time.time()
            except socket.timeout:
                pass
            except OSError:
                time.sleep(0.05)

    def control_loop(self):
        now = time.time()
        with self.lock:
            yaw = self.yaw_deg
            bat = self.battery
            height = self.height_m
            risk = self.vision_risk
            vision_age = now - self.last_vision_ts if self.last_vision_ts > 0 else 999.0
            vel_age = now - self.last_vel_ts if self.last_vel_ts > 0 else 999.0
            t_vel = self.target_vel
            follow = self.follow_enabled
            latched = self.estop_latched
            q_mode = self.qnx_mode
            q_seq = self.qnx_seq
            q_alive = (now - self.last_qnx_rx_ts) < 1.0
            last_land_ts = self.last_land_pub_ts

        # 1. Tello 状態を QNX UDP 5006 へ送信
        low_bat = (0 < bat < 20)
        edge_mode = 2 if low_bat else 0
        speed_proxy = abs(t_vel.linear.x) * 100.0 if follow else 0.0
        rpm_proxy = 800.0 + speed_proxy * 30.0
        try:
            self.tx_sock.sendto(
                struct.pack('<fffI', float(yaw), float(speed_proxy), float(rpm_proxy), int(edge_mode)),
                (QNX_IP, QNX_EDGE_PORT),
            )
        except OSError:
            pass

        # 2. 視覚危険度・低電池・追従中の映像断を QNX UDP 5008 へ送信
        vision_lost_in_follow = follow and (vision_age > 2.0)
        if risk >= 2 or low_bat or vision_lost_in_follow:
            override_mode = 2
            speed_limit = 0.0
        elif risk == 1:
            override_mode = 1
            speed_limit = 30.0
        else:
            override_mode = 0
            speed_limit = 240.0

        try:
            self.tx_sock.sendto(
                struct.pack('<If', int(override_mode), float(speed_limit)),
                (QNX_IP, QNX_ADAS_PORT),
            )
        except OSError:
            pass

        # 3. QNX 確定モードを配信し、安全調停を実行
        if q_alive:
            m = Int32(); m.data = q_mode
            self.pub_qnx_mode.publish(m)

        emergency_active = (q_alive and q_mode == 2) or ((not q_alive) and risk >= 2)
        if emergency_active:
            is_airborne = (height > 0.15)
            if not latched:
                with self.lock:
                    self.estop_latched = True
                    self.follow_enabled = False
                    self.sent_zero = True
                    self.last_land_pub_ts = now
                self.pub_vel.publish(Twist())
                reason = 'QNX_LOST_DANGER' if not q_alive else (
                    'LOW_BAT' if low_bat else ('VISION_TIMEOUT' if vision_lost_in_follow else 'VISION_DANGER/EXT')
                )
                self.get_logger().error(
                    f'[QNX INTERVENTION] Mode=2 confirmed (seq={q_seq}, reason={reason}) -> FOLLOW LOCKED'
                )
                if self.auto_land:
                    c = String(); c.data = 'land'
                    self.pub_cmd.publish(c)
                    self.get_logger().error('[QNX INTERVENTION] Issued /tello/command -> land')
            elif is_airborne and self.auto_land and (now - last_land_ts >= 1.0):
                with self.lock:
                    self.last_land_pub_ts = now
                self.pub_vel.publish(Twist())
                c = String(); c.data = 'land'
                self.pub_cmd.publish(c)
                self.get_logger().error(
                    f'[QNX INTERVENTION RETRY] Airborne (h={height:.2f}m) -> Re-issued /tello/command -> land'
                )
            return

        if latched and (height > 0.15) and (now - self.last_warn_ts >= 3.0):
            with self.lock:
                self.last_warn_ts = now
            self.get_logger().warn('[LATCHED IN FLIGHT] E-STOP latch active. Publish /tello/safety_reset true when clear.')

        if follow and not latched and q_alive:
            if vel_age <= 0.5:
                out = Twist()
                out.linear.x = float(t_vel.linear.x)
                out.angular.z = float(t_vel.angular.z)
                if q_mode == 1 and out.linear.x > 0.0:
                    out.linear.x = 0.0
                self.pub_vel.publish(out)
                with self.lock:
                    self.sent_zero = False
            elif not self.sent_zero:
                self.pub_vel.publish(Twist())
                with self.lock:
                    self.sent_zero = True

def main(args=None):
    rclpy.init(args=args)
    node = TelloQnxArbiterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if rclpy.ok():
                node.pub_vel.publish(Twist())
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass
