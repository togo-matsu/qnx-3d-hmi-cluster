import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32MultiArray, String
from geometry_msgs.msg import TwistStamped

try:
    from pymavlink import mavutil
    HAS_MAVLINK = True
except ImportError:
    HAS_MAVLINK = False


class StereoAvoidanceNode(Node):
    def __init__(self):
        super().__init__('stereo_avoidance_node')
        self.declare_parameter('estop_enter_m', 0.40)
        self.declare_parameter('estop_exit_m', 0.50)
        self.declare_parameter('avoid_dist_m', 1.00)
        self.declare_parameter('lr_deadband_m', 0.08)
        self.declare_parameter('mavlink_url', 'udpout:192.168.0.12:14550')
        self.declare_parameter('enable_mavlink', True)

        self.estop_enter = self.get_parameter('estop_enter_m').value
        self.estop_exit = self.get_parameter('estop_exit_m').value
        self.avoid_dist = self.get_parameter('avoid_dist_m').value
        self.lr_deadband = self.get_parameter('lr_deadband_m').value
        self.mavlink_url = self.get_parameter('mavlink_url').value
        self.enable_mavlink = self.get_parameter('enable_mavlink').value

        self.estop_pub = self.create_publisher(Bool, '/adas/trigger_estop', 10)
        self.cmd_pub = self.create_publisher(TwistStamped, '/mavros/setpoint_velocity/cmd_vel', 10)
        self.state_pub = self.create_publisher(String, '/stereo/avoidance_state', 10)

        self.mav = None
        if self.enable_mavlink and HAS_MAVLINK:
            try:
                self.mav = mavutil.mavlink_connection(self.mavlink_url)
                self.get_logger().info(f'MAVLink Offboard connected -> {self.mavlink_url}')
            except Exception as e:
                self.get_logger().warn(f'MAVLink direct init skipped ({e}); publishing ROS2 topics only.')

        self.estop_active = False
        self.evade_dir = 'LEFT'
        self.last_state = 'INIT'
        self.last_log_t = 0.0

        self.create_subscription(Float32MultiArray, '/stereo/zone_distances_m', self.zone_cb, 10)
        self.get_logger().info(
            f'Stereo Avoidance Ready (E-STOP <{self.estop_enter}m / release >{self.estop_exit}m, deadband={self.lr_deadband}m)'
        )

    def send_mavlink_velocity(self, vx: float, vy: float, yaw_rate: float):
        if self.mav is None:
            return
        try:
            self.mav.mav.set_position_target_local_ned_send(
                0, 1, 1,
                mavutil.mavlink.MAV_FRAME_BODY_NED,
                0x0DC7,
                0.0, 0.0, 0.0,
                vx, vy, 0.0,
                0.0, 0.0, 0.0,
                0.0, yaw_rate
            )
        except Exception:
            pass

    def zone_cb(self, msg: Float32MultiArray):
        if len(msg.data) < 3:
            return
        left_m, center_m, right_m = float(msg.data[0]), float(msg.data[1]), float(msg.data[2])
        min_front = min(left_m, center_m, right_m)

        # E-STOP ヒステリシス判定
        if self.estop_active:
            if min_front > self.estop_exit:
                self.estop_active = False
        else:
            if min_front < self.estop_enter:
                self.estop_active = True

        vx, vy, yaw_rate = 0.6, 0.0, 0.0
        if self.estop_active:
            state = 'EMERGENCY_STOP'
            vx, vy, yaw_rate = 0.0, 0.0, 0.0
        elif center_m < self.avoid_dist or left_m < (self.avoid_dist * 0.85) or right_m < (self.avoid_dist * 0.85):
            vx = 0.2
            # 左右回避ヒステリシス判定
            if (left_m - right_m) > self.lr_deadband:
                self.evade_dir = 'LEFT'
            elif (right_m - left_m) > self.lr_deadband:
                self.evade_dir = 'RIGHT'

            if self.evade_dir == 'LEFT':
                state = 'EVADE_LEFT'
                vy, yaw_rate = -0.35, -0.5
            else:
                state = 'EVADE_RIGHT'
                vy, yaw_rate = 0.35, 0.5
        else:
            state = 'CRUISE_CLEAR'

        b = Bool()
        b.data = self.estop_active
        self.estop_pub.publish(b)

        tw = TwistStamped()
        tw.header.stamp = self.get_clock().now().to_msg()
        tw.header.frame_id = 'base_link'
        tw.twist.linear.x = vx
        tw.twist.linear.y = vy
        tw.twist.angular.z = yaw_rate
        self.cmd_pub.publish(tw)
        self.send_mavlink_velocity(vx, vy, yaw_rate)

        st_msg = String()
        st_msg.data = state
        self.state_pub.publish(st_msg)

        now = time.time()
        if state != self.last_state or (now - self.last_log_t) > 1.0:
            self.get_logger().info(
                f'[{state}] L={left_m:.2f}m C={center_m:.2f}m R={right_m:.2f}m -> vx={vx:.2f}, vy={vy:.2f}, yaw={yaw_rate:.2f}'
            )
            self.last_state = state
            self.last_log_t = now


def main(args=None):
    rclpy.init(args=args)
    node = StereoAvoidanceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
