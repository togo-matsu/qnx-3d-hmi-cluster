import math
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
        self.declare_parameter('cruise_vx', 0.0)
        self.declare_parameter('hold_mode', 'spot')  # 'spot': その場ロック / 'anchor': 離陸地点へ自動復帰
        self.z_ned = -abs(float(self.get_parameter('target_alt_m').value))
        self.estop_enter = float(self.get_parameter('estop_enter_m').value)
        self.estop_exit = float(self.get_parameter('estop_exit_m').value)
        self.avoid_dist = float(self.get_parameter('avoid_dist_m').value)
        self.cruise_vx = float(self.get_parameter('cruise_vx').value)
        self.hold_mode = str(self.get_parameter('hold_mode').value)

        self.estop_pub = self.create_publisher(Bool, '/adas/trigger_estop', 10)
        self.state_pub = self.create_publisher(String, '/stereo/avoidance_state', 10)
        self.create_subscription(Float32MultiArray, '/stereo/zone_distances_m', self.zone_cb, 10)

        self.mav = mavutil.mavlink_connection('udpin:0.0.0.0:14540')
        self.get_logger().info('Waiting for PX4 SITL HEARTBEAT on UDP 14540...')
        self.mav.wait_heartbeat()
        self.get_logger().info(f'PX4 connected! hold_mode={self.hold_mode}')

        self.vx, self.vy, self.state = 0.0, 0.0, 'TAKEOFF_HOVER'
        self.use_pos_hold = True
        self.curr_x, self.curr_y, self.current_yaw = 0.0, 0.0, math.pi / 2.0
        self.anchor_x, self.anchor_y = None, None
        self.hold_x, self.hold_y = 0.0, 0.0
        self.estop_active, self.evade_dir = False, 'LEFT'
        self.tick, self.armed, self.last_state, self.last_log_t = 0, False, '', 0.0
        self.create_timer(0.05, self.control_loop)

    def control_loop(self):
        for _ in range(50):
            m = self.mav.recv_match(blocking=False)
            if m is None:
                break
            t = m.get_type()
            if t == 'ATTITUDE':
                self.current_yaw = float(m.yaw)
            elif t == 'LOCAL_POSITION_NED':
                self.curr_x, self.curr_y = float(m.x), float(m.y)
                if self.anchor_x is None:
                    self.anchor_x, self.anchor_y = self.curr_x, self.curr_y
                    self.hold_x, self.hold_y = self.curr_x, self.curr_y

        if self.use_pos_hold:
            # type_mask=0x05F8: 位置(x, y, z) + yaw_rate=0 で座標を厳密ロック
            tx = self.anchor_x if (self.hold_mode == 'anchor' and self.anchor_x is not None) else self.hold_x
            ty = self.anchor_y if (self.hold_mode == 'anchor' and self.anchor_y is not None) else self.hold_y
            self.mav.mav.set_position_target_local_ned_send(
                0, self.mav.target_system, self.mav.target_component,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED, 0x05F8,
                tx, ty, self.z_ned, 0, 0, 0, 0, 0, 0, 0, 0
            )
        else:
            # type_mask=0x05E3: 機体基準速度(vx, vy)をYaw回転変換 + 高度(z)制御
            cy, sy = math.cos(self.current_yaw), math.sin(self.current_yaw)
            vx_ned = self.vx * cy - self.vy * sy
            vy_ned = self.vx * sy + self.vy * cy
            self.mav.mav.set_position_target_local_ned_send(
                0, self.mav.target_system, self.mav.target_component,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED, 0x05E3,
                0, 0, self.z_ned, vx_ned, vy_ned, 0, 0, 0, 0, 0, 0
            )

        self.tick += 1
        if not self.armed and self.tick == 30:
            self.mav.mav.command_long_send(self.mav.target_system, self.mav.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0, 1, 6, 0, 0, 0, 0, 0)
            self.mav.mav.command_long_send(self.mav.target_system, self.mav.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
            self.armed = True
            self.get_logger().info(f'OFFBOARD & ARM! Anchor=({self.hold_x:.2f}, {self.hold_y:.2f}) alt={-self.z_ned:.1f}m')

    def zone_cb(self, msg: Float32MultiArray):
        if len(msg.data) < 3: return
        l_m, c_m, r_m = float(msg.data[0]), float(msg.data[1]), float(msg.data[2])
        min_f = min(l_m, c_m, r_m)
        if self.estop_active and min_f > self.estop_exit: self.estop_active = False
        elif not self.estop_active and min_f < self.estop_enter: self.estop_active = True

        was_holding = self.use_pos_hold
        if self.estop_active:
            self.state, self.vx, self.vy, self.use_pos_hold = 'EMERGENCY_STOP', 0.0, 0.0, True
            if not was_holding:
                self.hold_x, self.hold_y = self.curr_x, self.curr_y
        elif c_m < self.avoid_dist or l_m < self.avoid_dist * 0.85 or r_m < self.avoid_dist * 0.85:
            self.use_pos_hold = False
            diff_lr = l_m - r_m
            if abs(diff_lr) <= 0.12:
                self.state, self.vx, self.vy = 'RETREAT', -0.6, 0.0
            else:
                if diff_lr > 0.12: self.evade_dir = 'LEFT'
                elif diff_lr < -0.12: self.evade_dir = 'RIGHT'
                self.vx = 0.2 if self.cruise_vx > 0 else 0.0
                self.state, self.vy = ('EVADE_LEFT', -0.8) if self.evade_dir == 'LEFT' else ('EVADE_RIGHT', 0.8)
        else:
            if self.cruise_vx > 0.0:
                self.state, self.vx, self.vy, self.use_pos_hold = 'CRUISE_CLEAR', self.cruise_vx, 0.0, False
            else:
                self.state, self.vx, self.vy, self.use_pos_hold = 'POS_HOLD', 0.0, 0.0, True
                if not was_holding:
                    self.hold_x, self.hold_y = self.curr_x, self.curr_y

        self.estop_pub.publish(Bool(data=self.estop_active))
        self.state_pub.publish(String(data=self.state))
        now = time.time()
        if self.state != self.last_state or (now - self.last_log_t) > 1.0:
            self.get_logger().info(
                f'[{self.state}] pos=({self.curr_x:.2f},{self.curr_y:.2f}) L={l_m:.2f} C={c_m:.2f} R={r_m:.2f}'
            )
            self.last_state, self.last_log_t = self.state, now

def main(args=None):
    rclpy.init(args=args); node = Px4StereoAvoidanceNode(); rclpy.spin(node); node.destroy_node(); rclpy.shutdown()
