import socket
import struct
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float32, Bool, String

class AdasSafetyNode(Node):
    def __init__(self):
        super().__init__('adas_safety_node')
        self.qnx_target = ('10.0.0.169', 5008)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.speed_kmh = 0.0
        self.rms_g = 1.0
        self.manual_estop = False

        self.create_subscription(TwistStamped, '/vehicle/twist', self.twist_cb, 10)
        self.create_subscription(Float32, '/vehicle/Vibration_rms', self.rms_cb, 10)
        self.create_subscription(Bool, '/adas/trigger_estop', self.estop_cb, 10)
        self.status_pub = self.create_publisher(String, '/adas/status', 10)

        self.create_timer(0.05, self.control_loop) # 20Hz 監視ループ
        self.get_logger().info('ROS 2 ADAS Safety Node active -> Monitoring & Intervening to QNX (10.0.0.169:5008)')

    def twist_cb(self, msg: TwistStamped):
        self.speed_kmh = msg.twist.linear.x * 3.6

    def rms_cb(self, msg: Float32):
        self.rms_g = msg.data

    def estop_cb(self, msg: Bool):
        self.manual_estop = msg.data
        self.get_logger().warn(f'Manual ADAS E-STOP Trigger set to: {self.manual_estop}')

    def control_loop(self):
        override_mode = 0
        speed_limit = 240.0
        state_str = "NORMAL"

        # 判定1: 外部トリガーまたは激しい衝撃 (RMS > 2.2G) で緊急ブレーキ介入 (Mode 2)
        if self.manual_estop or self.rms_g > 2.2:
            override_mode = 2
            speed_limit = 0.0
            state_str = "ADAS_EMERGENCY_BRAKE"
        # 判定2: 高速域 (150 km/h 超) でアンバー警告 + リミッター作動 (Mode 1)
        elif self.speed_kmh > 150.0:
            override_mode = 1
            speed_limit = 160.0
            state_str = "ADAS_SPEED_WARNING"

        if override_mode > 0:
            pkt = struct.pack('<If', override_mode, speed_limit)
            try:
                self.sock.sendto(pkt, self.qnx_target)
            except Exception:
                pass

        msg = String()
        msg.data = state_str
        self.status_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = AdasSafetyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
