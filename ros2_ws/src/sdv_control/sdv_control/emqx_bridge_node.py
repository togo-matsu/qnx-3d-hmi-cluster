import json
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
        self.mqtt_topic  = 'sdv/ros2/telemetry'

        self.client = mqtt.Client()
        # 非同期接続にすることで、EMQX未起動時でもエラーを出さず自動再接続待機する
        self.client.connect_async(self.mqtt_broker, self.mqtt_port, 60)
        self.client.loop_start()
        self.get_logger().info(f'EMQX Bridge ready (async target: {self.mqtt_broker}:{self.mqtt_port})')

        self.odom_x = 0.0
        self.odom_y = 0.0
        self.speed_kmh = 0.0
        self.rpm = 800.0
        self.adas_status = "NORMAL"

        self.create_subscription(Odometry, '/vehicle/odom', self.odom_cb, 10)
        self.create_subscription(Float32, '/vehicle/rpm', self.rpm_cb, 10)
        self.create_subscription(String, '/adas/status', self.status_cb, 10)

        self.create_timer(0.2, self.publish_mqtt) # 5Hz

    def odom_cb(self, msg: Odometry):
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y
        self.speed_kmh = msg.twist.twist.linear.x * 3.6

    def rpm_cb(self, msg: Float32):
        self.rpm = msg.data

    def status_cb(self, msg: String):
        self.adas_status = msg.data

    def publish_mqtt(self):
        if not self.client.is_connected():
            return
        payload = {
            "timestamp": time.time(),
            "odom_x": round(self.odom_x, 2),
            "odom_y": round(self.odom_y, 2),
            "speed_kmh": round(self.speed_kmh, 1),
            "rpm": int(self.rpm),
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
