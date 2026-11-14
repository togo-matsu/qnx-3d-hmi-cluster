import json
import math
import socket
import time
import paho.mqtt.client as mqtt
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, String

class Ros2EmqxBridgeNode(Node):
    def __init__(self):
        super().__init__('ros2_emqx_bridge_node')
        self.mqtt_broker = '10.0.0.201'
        self.mqtt_port   = 1883
        # K8s mqtt-worker が購読しているトピック形式 (devices/+/telemetry) に合わせる
        self.mqtt_topic  = 'devices/sdv-ros2/telemetry'

        self.client = mqtt.Client()
        try:
            self.client.connect(self.mqtt_broker, self.mqtt_port, 60)
            self.client.loop_start()
            self.get_logger().info(f'Connected to K8s EMQX ({self.mqtt_broker}:{self.mqtt_port}) -> {self.mqtt_topic}')
        except Exception as e:
            self.get_logger().warn(f'EMQX connection deferred: {e}')

        self.odom_x = 0.0
        self.odom_y = 0.0
        self.yaw_deg = 0.0
        self.speed_kmh = 0.0
        self.rpm = 800.0
        self.rms_g = 1.0
        self.adas_status = "NORMAL"

        self.declare_parameter('odom_topic', '/vehicle/odom')
        odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value

        self.udp_target = ('127.0.0.1', 5009)
        self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_sock.setblocking(False)

        self.create_subscription(Odometry, odom_topic, self.odom_cb, 10)
        self.create_subscription(Float32, '/vehicle/rpm', self.rpm_cb, 10)
        self.create_subscription(Float32, '/vehicle/Vibration_rms', self.rms_cb, 10)
        self.create_subscription(String, '/adas/status', self.status_cb, 10)

        self.create_timer(0.2, self.publish_mqtt) # 5Hz

    def odom_cb(self, msg: Odometry):
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        yaw_rad = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
        self.yaw_deg = (math.degrees(yaw_rad) + 360.0) % 360.0
        self.speed_kmh = msg.twist.twist.linear.x * 3.6

        udp_payload = json.dumps({
            "x": round(self.odom_x, 4),
            "y": round(self.odom_y, 4),
            "yaw_deg": round(self.yaw_deg, 2),
        }) + "\n"
        try:
            self.udp_sock.sendto(udp_payload.encode('utf-8'), self.udp_target)
        except (BlockingIOError, OSError):
            pass

    def rpm_cb(self, msg: Float32):
        self.rpm = msg.data

    def rms_cb(self, msg: Float32):
        self.rms_g = msg.data

    def status_cb(self, msg: String):
        self.adas_status = msg.data

    def publish_mqtt(self):
        payload = {
            "device_id": "sdv-ros2",
            "timestamp": round(time.time(), 3),
            "odom_x": round(self.odom_x, 2),
            "odom_y": round(self.odom_y, 2),
            "yaw_deg": round(self.yaw_deg, 1),
            "speed_kmh": round(self.speed_kmh, 1),
            "rpm": int(self.rpm),
            "rms_accel": round(self.rms_g, 3),
            "adas_status": self.adas_status
        }
        try:
            self.client.publish(self.mqtt_topic, json.dumps(payload))
        except Exception:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = Ros2EmqxBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
