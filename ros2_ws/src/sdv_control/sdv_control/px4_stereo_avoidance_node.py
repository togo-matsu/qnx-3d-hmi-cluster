import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32MultiArray, String
from pymavlink import mavutil

class Px4StereoAvoidanceNode(Node):
    def __init__(self):
        super().__init__('px4_stereo_avoidance_node')
        self.declare_parameter('target_alt_m', 2.5)
        self.declare_parameter('estop_enter_m', 0.40)
        self.declare_parameter('estop_exit_m', 0.50)
        self.declare_parameter('avoid_dist_m', 1.00)
        self.z_ned = -abs(float(self.get_parameter('target_alt_m').value))
        self.estop_enter = float(self.get_parameter('estop_enter_m').value)
        self.estop_exit = float(self.get_parameter('estop_exit_m').value)
        self.avoid_dist = float(self.get_parameter('avoid_dist_m').value)

        self.estop_pub = self.create_publisher(Bool, '/adas/trigger_estop', 10)
        self.state_pub = self.create_publisher(String, '/stereo/avoidance_state', 10)
        self.create_subscription(Float32MultiArray, '/stereo/zone_distances_m', self.zone_cb, 10)

        self.mav = mavutil.mavlink_connection('udpin:0.0.0.0:14540')
        self.get_logger().info('Waiting for PX4 SITL HEARTBEAT on UDP 14540...')
        self.mav.wait_heartbeat()
        self.get_logger().info('PX4 SITL connected! Preparing Offboard takeoff...')
        self.vx, self.vy, self.state = 0.0, 0.0, 'TAKEOFF_HOVER'
        self.estop_active, self.evade_dir = False, 'LEFT'
        self.tick, self.armed, self.last_state, self.last_log_t = 0, False, '', 0.0
        self.create_timer(0.05, self.control_loop)  # 20Hz Offboard 送信
    def control_loop(self):
        self.mav.mav.set_position_target_local_ned_send(
            0, self.mav.target_system, self.mav.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED, 0x05E3,
            0, 0, self.z_ned, self.vx, self.vy, 0, 0, 0, 0, 0, 0
        )
        self.tick += 1
        if not self.armed and self.tick == 30:
            self.mav.mav.command_long_send(self.mav.target_system, self.mav.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0, 1, 6, 0, 0, 0, 0, 0)
            self.mav.mav.command_long_send(self.mav.target_system, self.mav.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
            self.armed = True
            self.get_logger().info(f'OFFBOARD & ARM sent! Holding altitude {-self.z_ned:.1f}m')

    def zone_cb(self, msg: Float32MultiArray):
        if len(msg.data) < 3: return
        l_m, c_m, r_m = float(msg.data[0]), float(msg.data[1]), float(msg.data[2])
        min_f = min(l_m, c_m, r_m)
        if self.estop_active and min_f > self.estop_exit: self.estop_active = False
        elif not self.estop_active and min_f < self.estop_enter: self.estop_active = True

        if self.estop_active:
            self.state, self.vx, self.vy = 'EMERGENCY_STOP', 0.0, 0.0
        elif c_m < self.avoid_dist or l_m < self.avoid_dist * 0.85 or r_m < self.avoid_dist * 0.85:
            if (l_m - r_m) > 0.08: self.evade_dir = 'LEFT'
            elif (r_m - l_m) > 0.08: self.evade_dir = 'RIGHT'
            self.vx = 0.2
            self.state, self.vy = ('EVADE_LEFT', -0.8) if self.evade_dir == 'LEFT' else ('EVADE_RIGHT', 0.8)
        else:
            self.state, self.vx, self.vy = 'CRUISE_CLEAR', 0.8, 0.0

        self.estop_pub.publish(Bool(data=self.estop_active))
        self.state_pub.publish(String(data=self.state))
        now = time.time()
        if self.state != self.last_state or (now - self.last_log_t) > 1.0:
            self.get_logger().info(f'[{self.state}] L={l_m:.2f} C={c_m:.2f} R={r_m:.2f} -> vx={self.vx:.2f} vy={self.vy:.2f}')
            self.last_state, self.last_log_t = self.state, now

def main(args=None):
    rclpy.init(args=args); node = Px4StereoAvoidanceNode(); rclpy.spin(node); node.destroy_node(); rclpy.shutdown()
