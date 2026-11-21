import math, socket, struct, time, rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

class VioImuNode(Node):
    def __init__(self):
        super().__init__('vio_imu_node')
        self.pub = self.create_publisher(Imu, '/vio/imu', 50)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(('0.0.0.0', 5007))
        self.sock.setblocking(False)
        self.qx, self.qy, self.qz, self.qw = 0.0, 0.0, 0.0, 1.0
        self.initialized = False
        self.last_ts = None
        self.create_timer(0.005, self.poll)
        self.get_logger().info('VIO IMU ready on UDP:5007 (Axis-corrected Quaternion Filter)')

    def poll(self):
        while True:
            try:
                d, _ = self.sock.recvfrom(128)
            except BlockingIOError:
                break
            if len(d) != 40 or d[:4] != b'VIO1':
                continue

            ax, ay, az, gx, gy, gz, _, _, _ = struct.unpack('<9f', d[4:40])
            now_sec = time.monotonic()
            k = math.pi / 180.0
            # ROS REP-103 (X:前, Y:左, Z:上) に合わせてピッチ角速度 gy を反転
            wx, wy, wz = gx * k, -gy * k, gz * k

            norm_a = math.sqrt(ax*ax + ay*ay + az*az)
            if not self.initialized and norm_a > 0.1:
                r0 = math.atan2(ay, az)
                p0 = math.atan2(-ax, math.sqrt(ay*ay + az*az))
                cr, sr = math.cos(r0 * 0.5), math.sin(r0 * 0.5)
                cp, sp = math.cos(p0 * 0.5), math.sin(p0 * 0.5)
                self.qx = sr * cp
                self.qy = cr * sp
                self.qz = -sr * sp
                self.qw = cr * cp
                self.initialized = True
                self.last_ts = now_sec
                continue

            dt = max(0.001, min(0.1, now_sec - self.last_ts))
            self.last_ts = now_sec

            # 重力ベクトルを用いたロール・ピッチドリフト補正 (0.75g〜1.25g時)
            if 0.75 < norm_a < 1.25:
                ux, uy, uz = ax / norm_a, ay / norm_a, az / norm_a
                vx = 2.0 * (self.qx * self.qz - self.qw * self.qy)
                vy = 2.0 * (self.qw * self.qx + self.qy * self.qz)
                vz = self.qw*self.qw - self.qx*self.qx - self.qy*self.qy + self.qz*self.qz
                kp = 1.5
                wx += kp * (uy * vz - uz * vy)
                wy += kp * (uz * vx - ux * vz)
                wz += kp * (ux * vy - uy * vx)

            qw, qx, qy, qz = self.qw, self.qx, self.qy, self.qz
            self.qw += 0.5 * (-qx*wx - qy*wy - qz*wz) * dt
            self.qx += 0.5 * ( qw*wx + qy*wz - qz*wy) * dt
            self.qy += 0.5 * ( qw*wy - qx*wz + qz*wx) * dt
            self.qz += 0.5 * ( qw*wz + qx*wy - qy*wx) * dt
            nq = math.sqrt(self.qw**2 + self.qx**2 + self.qy**2 + self.qz**2)
            self.qw, self.qx, self.qy, self.qz = self.qw/nq, self.qx/nq, self.qy/nq, self.qz/nq

            m = Imu()
            m.header.stamp = self.get_clock().now().to_msg()
            m.header.frame_id = 'base_link'
            m.orientation.x, m.orientation.y = self.qx, self.qy
            m.orientation.z, m.orientation.w = self.qz, self.qw
            m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z = wx, wy, wz
            m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z = ax*9.81, ay*9.81, az*9.81
            for i in (0, 4, 8):
                m.orientation_covariance[i] = 0.02
                m.angular_velocity_covariance[i] = 0.01
                m.linear_acceleration_covariance[i] = 0.04
            self.pub.publish(m)

def main(args=None):
    rclpy.init(args=args); n = VioImuNode()
    try: rclpy.spin(n)
    except KeyboardInterrupt: pass
    finally: n.destroy_node(); rclpy.ok() and rclpy.shutdown()
