import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class DroneOffboardCommander(Node):
    def __init__(self):
        super().__init__('drone_offboard_commander')
        self.declare_parameter('roll_deg', 15.0)
        self.declare_parameter('pitch_deg', 0.0)
        self.declare_parameter('thrust', 0.55)
        self.pub = self.create_publisher(Float32MultiArray, '/drone/offboard_setpoint', 10)
        self.create_timer(0.05, self.timer_cb)
        self.get_logger().info('Streaming 20Hz Offboard Setpoint...')

    def timer_cb(self):
        r = float(self.get_parameter('roll_deg').value)
        p = float(self.get_parameter('pitch_deg').value)
        t = float(self.get_parameter('thrust').value)
        self.pub.publish(Float32MultiArray(data=[r, p, t]))

def main(args=None):
    rclpy.init(args=args)
    node = DroneOffboardCommander()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
