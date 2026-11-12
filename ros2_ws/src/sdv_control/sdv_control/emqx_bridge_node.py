import json
import math
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

        self.create_subscription(Odometry, '/vehicle/odom', self.odom_cb, 10)
        self.create_subscription(Float32, '/vehicle/rpm', self.rpm_cb, 10)
        self.create_subscription(Float32, '/vehicle/Vibration_rms', self.rms_cb, 10)
        self.create_subscription(String, '/adas/status', self.status_cb, 10)

        self.create_timer(0.2, self.publish_mqtt) # 5Hz

    def odom_cb(self, msg: Odometry):
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        yaw_rad = 2.0 * math.atan2(qz, qw)
        self.yaw_deg = (math.degrees(yaw_rad) + 360.0) % 360.0
        self.speed_kmh = msg.twist.twist.linear.x * 3.6

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
