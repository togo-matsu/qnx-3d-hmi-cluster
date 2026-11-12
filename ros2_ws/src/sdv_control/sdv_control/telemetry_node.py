import math
import socket
import struct
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TwistStamped, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from tf2_ros import TransformBroadcaster

class SdvTelemetryNode(Node):
    def __init__(self):
        super().__init__('sdv_telemetry_node')
        self.imu_pub   = self.create_publisher(Imu, '/vehicle/imu', 10)
        self.twist_pub = self.create_publisher(TwistStamped, '/vehicle/twist', 10)
        self.odom_pub  = self.create_publisher(Odometry, '/vehicle/odom', 10)
        self.rpm_pub   = self.create_publisher(Float32, '/vehicle/rpm', 10)
        self.rms_pub   = self.create_publisher(Float32, '/vehicle/Vibration_rms', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('0.0.0.0', 5007))
        self.sock.setblocking(False)

        # 2D オドメトリ積分状態 (x, y [m])
        self.x = 0.0
        self.y = 0.0
        self.last_time = self.get_clock().now()

        self.create_timer(0.02, self.timer_callback) # 50Hz
        self.get_logger().info('SDV Telemetry & TF2 Odometry Node started on UDP:5007')

    def timer_callback(self):
        latest_data = None
        while True:
            try:
                data, _ = self.sock.recvfrom(64)
                if len(data) == 24:
                    latest_data = struct.unpack('<fffffI', data)
            except BlockingIOError:
                break

        if latest_data is None:
            return

        angle_deg, pitch_deg, rms_g, speed_kmh, rpm, _ = latest_data
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        # 角度 (deg -> rad) と 速度 (km/h -> m/s)
        yaw = math.radians(angle_deg)
        pitch = math.radians(pitch_deg)
        v_ms = speed_kmh / 3.6

        # オドメトリ位置積分
        if 0.0 < dt < 1.0:
            self.x += v_ms * math.cos(yaw) * dt
            self.y += v_ms * math.sin(yaw) * dt

        qz = math.sin(yaw * 0.5)
        qw = math.cos(yaw * 0.5)

        # 1. TF2 Broadcast (odom -> base_link)
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(t)

        # 2. Odometry メッセージ
        odom = Odometry()
        odom.header = t.header
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = v_ms
        self.odom_pub.publish(odom)

        # 3. TwistStamped (車速)
        twist = TwistStamped()
        twist.header = t.header
        twist.twist.linear.x = v_ms
        self.twist_pub.publish(twist)

        # 4. IMU メッセージ
        imu = Imu()
        imu.header.stamp = now.to_msg()
        imu.header.frame_id = 'base_link'
        imu.orientation.z = qz
        imu.orientation.w = qw
        imu.linear_acceleration.x = math.sin(pitch) * 9.81
        imu.linear_acceleration.z = rms_g * 9.81
        self.imu_pub.publish(imu)

        # 5. RPM & 振動 RMS
        rpm_msg = Float32()
        rpm_msg.data = float(rpm)
        self.rpm_pub.publish(rpm_msg)

        rms_msg = Float32()
        rms_msg.data = float(rms_g)
        self.rms_pub.publish(rms_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SdvTelemetryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
