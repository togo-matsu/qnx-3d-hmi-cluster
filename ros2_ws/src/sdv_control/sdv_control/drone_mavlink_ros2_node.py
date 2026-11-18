import math, socket, struct, rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, Float32MultiArray
from pymavlink import mavutil

def euler_deg_to_quat(r_deg, p_deg, y_deg):
    r, p, y = math.radians(r_deg)*0.5, math.radians(p_deg)*0.5, math.radians(y_deg)*0.5
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return [cr*cp*cy + sr*sp*sy, sr*cp*cy - cr*sp*sy, cr*sp*cy + sr*cp*sy, cr*cp*sy - sr*sp*cy]

class DroneMavlinkRos2Node(Node):
    def __init__(self):
        super().__init__('drone_mavlink_ros2_node')
        self.mav_rx  = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
        self.mav_tx  = mavutil.mavlink_connection('udpout:192.168.0.12:14552', source_system=255, source_component=0)
        self.qgc_out = mavutil.mavlink_connection('udpout:127.0.0.1:14551', source_system=1, source_component=1)
        self.qnx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.qnx_addr = ('10.0.0.169', 5006)

        self.pub_armed = self.create_publisher(Bool, '/drone/armed', 10)
        self.pub_att   = self.create_publisher(Float32MultiArray, '/drone/attitude_deg', 10)
        self.pub_pwm   = self.create_publisher(Float32MultiArray, '/drone/motor_pwm', 10)

        self.create_subscription(Float32MultiArray, '/drone/offboard_setpoint', self.offboard_cb, 10)
        self.create_subscription(Bool, '/drone/cmd_arm', self.arm_cb, 10)
        self.create_subscription(Bool, '/drone/estop', self.estop_cb, 10)

        self.armed = False
        self.roll = self.pitch = self.yaw = self.alt = self.throttle = 0.0
        self.start_time = self.get_clock().now().nanoseconds * 1e-9
        self.create_timer(0.01, self.poll_mavlink)
        self.get_logger().info('Drone MAVLink-ROS2 Node ready: RX<-14550 | TX->192.168.0.12:14552')

    def _boot_ms(self):
        return int(((self.get_clock().now().nanoseconds * 1e-9) - self.start_time) * 1000) & 0xFFFFFFFF

    def offboard_cb(self, msg):
        if len(msg.data) < 3:
            return
        q = euler_deg_to_quat(float(msg.data[0]), float(msg.data[1]), 0.0)
        thrust = max(0.0, min(1.0, float(msg.data[2])))
        self.mav_tx.mav.set_attitude_target_send(self._boot_ms(), 1, 1, 0b00000111, q, 0.0, 0.0, 0.0, thrust)

    def arm_cb(self, msg):
        val = 1.0 if msg.data else 0.0
        self.mav_tx.mav.command_long_send(1, 1, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, val, 0, 0, 0, 0, 0, 0)
        self.get_logger().info(f'Sent MAVLink ARM={msg.data} -> 192.168.0.12:14552')

    def estop_cb(self, msg):
        if not msg.data:
            return
        self.mav_tx.mav.command_long_send(1, 1, mavutil.mavlink.MAV_CMD_DO_FLIGHTTERMINATION, 0, 1.0, 0, 0, 0, 0, 0, 0)
        self.get_logger().warn('Sent MAVLink E-STOP -> 192.168.0.12:14552')

    def poll_mavlink(self):
        while True:
            msg = self.mav_rx.recv_match(blocking=False)
            if msg is None:
                break
            mtype = msg.get_type()
            if mtype == 'BAD_DATA':
                continue
            if mtype == 'HEARTBEAT':
                self.armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                self.pub_armed.publish(Bool(data=self.armed))
                self.qgc_out.mav.heartbeat_send(msg.type, msg.autopilot, msg.base_mode, msg.custom_mode, msg.system_status)
            elif mtype == 'ATTITUDE':
                self.roll, self.pitch, self.yaw = math.degrees(msg.roll), math.degrees(msg.pitch), math.degrees(msg.yaw)
                self.pub_att.publish(Float32MultiArray(data=[self.roll, self.pitch, self.yaw, self.alt]))
                self.qgc_out.mav.attitude_send(msg.time_boot_ms, msg.roll, msg.pitch, msg.yaw, 0.0, 0.0, 0.0)
                mode = 0 if self.armed else 2
                pkt = struct.pack('<fffI', self.yaw % 360.0, self.throttle * 1.2 if self.armed else 0.0, 1000.0 + self.throttle * 60.0 if self.armed else 0.0, mode)
                self.qnx_sock.sendto(pkt, self.qnx_addr)
            elif mtype == 'VFR_HUD':
                self.alt, self.throttle = float(msg.alt), float(msg.throttle)
                self.qgc_out.mav.vfr_hud_send(msg.airspeed, msg.groundspeed, msg.heading, msg.throttle, msg.alt, msg.climb)
            elif mtype == 'SERVO_OUTPUT_RAW':
                self.pub_pwm.publish(Float32MultiArray(data=[float(msg.servo1_raw), float(msg.servo2_raw), float(msg.servo3_raw), float(msg.servo4_raw)]))
                self.qgc_out.mav.servo_output_raw_send(msg.time_usec, msg.port, msg.servo1_raw, msg.servo2_raw, msg.servo3_raw, msg.servo4_raw, 1000, 1000, 1000, 1000)

def main(args=None):
    rclpy.init(args=args)
    node = DroneMavlinkRos2Node()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
