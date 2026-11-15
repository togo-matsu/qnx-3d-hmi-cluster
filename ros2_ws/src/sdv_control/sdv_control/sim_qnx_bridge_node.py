import math
import socket
import struct
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class SimQnxBridgeNode(Node):
    def __init__(self):
        super().__init__('sim_qnx_bridge_node')
        self.declare_parameter('qnx_ip', '10.0.0.169')
        self.declare_parameter('qnx_port', 5006)
        self.declare_parameter('speed_scale', 150.0)

        self.qnx_ip = self.get_parameter('qnx_ip').get_parameter_value().string_value
        self.qnx_port = self.get_parameter('qnx_port').get_parameter_value().integer_value
        self.speed_scale = self.get_parameter('speed_scale').get_parameter_value().double_value

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.target = (self.qnx_ip, self.qnx_port)

        self.angle_deg = 0.0
        self.speed_kmh = 0.0
        self.rpm = 800.0
        self.mode = 0

        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_cb, 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_timer(0.02, self.send_qnx_packet)  # 50Hz

        self.get_logger().info(
            f'Sim-to-QNX Bridge active -> {self.qnx_ip}:{self.qnx_port} (scale={self.speed_scale})'
        )

    def cmd_vel_cb(self, msg: Twist):
        # m/s -> km/h (視認用スケール適用)
        raw_kmh = abs(msg.linear.x) * 3.6 * self.speed_scale
        self.speed_kmh = max(0.0, min(240.0, raw_kmh))
        self.rpm = (self.speed_kmh / 240.0) * 7200.0 + 800.0

    def odom_cb(self, msg: Odometry):
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw_deg = math.degrees(math.atan2(siny_cosp, cosy_cosp))
        self.angle_deg = (yaw_deg + 360.0) % 360.0

    def send_qnx_packet(self):
        pkt = struct.pack('<fffI', self.angle_deg, self.speed_kmh, self.rpm, self.mode)
        try:
            self.sock.sendto(pkt, self.target)
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = SimQnxBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
