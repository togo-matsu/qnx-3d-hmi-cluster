import math
import socket
import struct
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32MultiArray
from pymavlink import mavutil


def euler_deg_to_quat(roll_deg: float, pitch_deg: float, yaw_deg: float):
    r = math.radians(roll_deg) * 0.5
    p = math.radians(pitch_deg) * 0.5
    y = math.radians(yaw_deg) * 0.5
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    return [
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    ]


class DroneMavlinkRos2Node(Node):
    def __init__(self):
        super().__init__('drone_mavlink_ros2_node')
        self.mav = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
        self.qgc_out = mavutil.mavlink_connection('udpout:127.0.0.1:14551', source_system=1, source_component=1)
        self.qnx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.qnx_addr = ('10.0.0.169', 5006)

        # テレメトリ配信 (FC -> ROS 2)
        self.pub_armed = self.create_publisher(Bool, '/drone/armed', 10)
        self.pub_att   = self.create_publisher(Float32MultiArray, '/drone/attitude_deg', 10)
        self.pub_pwm   = self.create_publisher(Float32MultiArray, '/drone/motor_pwm', 10)

        # Offboard 指令購読 (ROS 2 -> FC)
        # /drone/offboard_setpoint: [roll_deg, pitch_deg, thrust_0_to_1]
        self.create_subscription(Float32MultiArray, '/drone/offboard_setpoint', self.offboard_cb, 10)
        self.create_subscription(Bool, '/drone/cmd_arm', self.arm_cb, 10)
        self.create_subscription(Bool, '/drone/estop', self.estop_cb, 10)

        self.armed = False
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.alt = 0.0
        self.throttle = 0.0
        self.start_time = self.get_clock().now().nanoseconds * 1e-9

        self.create_timer(0.01, self.poll_mavlink)
        self.get_logger().info('Bidirectional Drone MAVLink-ROS2 Node ready (UDP:14550 <-> /drone/*)')

    def _boot_ms(self) -> int:
        now = self.get_clock().now().nanoseconds * 1e-9
        return int((now - self.start_time) * 1000) & 0xFFFFFFFF

    def offboard_cb(self, msg: Float32MultiArray):
        if getattr(self.mav, 'last_addr', None) is None or len(msg.data) < 3:
            return
        q = euler_deg_to_quat(float(msg.data[0]), float(msg.data[1]), 0.0)
        thrust = max(0.0, min(1.0, float(msg.data[2])))
        self.mav.mav.set_attitude_target_send(
            self._boot_ms(), 1, 1, 0b00000111, q, 0.0, 0.0, 0.0, thrust
        )

    def arm_cb(self, msg: Bool):
        if getattr(self.mav, 'last_addr', None) is None:
            return
        val = 1.0 if msg.data else 0.0
        self.mav.mav.command_long_send(
            1, 1, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, val, 0, 0, 0, 0, 0, 0
        )
        self.get_logger().info(f'Sent MAVLink ARM={msg.data}')

    def estop_cb(self, msg: Bool):
        if not msg.data or getattr(self.mav, 'last_addr', None) is None:
            return
        self.mav.mav.command_long_send(
            1, 1, mavutil.mavlink.MAV_CMD_DO_FLIGHTTERMINATION, 0, 1.0, 0, 0, 0, 0, 0, 0
        )
        self.get_logger().warn('Sent MAVLink E-STOP (FLIGHT TERMINATION)!')

    def poll_mavlink(self):
        while True:
            msg = self.mav.recv_match(blocking=False)
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
                self.roll  = math.degrees(msg.roll)
                self.pitch = math.degrees(msg.pitch)
                self.yaw   = math.degrees(msg.yaw)
                self.pub_att.publish(Float32MultiArray(data=[self.roll, self.pitch, self.yaw, self.alt]))
                self.qgc_out.mav.attitude_send(msg.time_boot_ms, msg.roll, msg.pitch, msg.yaw, 0.0, 0.0, 0.0)

                mode = 0 if self.armed else 2
                speed_sim = self.throttle * 1.2 if self.armed else 0.0
                rpm_sim = 1000.0 + self.throttle * 60.0 if self.armed else 0.0
                pkt = struct.pack('<fffI', self.yaw % 360.0, speed_sim, rpm_sim, mode)
                self.qnx_sock.sendto(pkt, self.qnx_addr)

            elif mtype == 'VFR_HUD':
                self.alt = float(msg.alt)
                self.throttle = float(msg.throttle)
                self.qgc_out.mav.vfr_hud_send(msg.airspeed, msg.groundspeed, msg.heading, msg.throttle, msg.alt, msg.climb)

            elif mtype == 'SERVO_OUTPUT_RAW':
                pwms = [float(msg.servo1_raw), float(msg.servo2_raw), float(msg.servo3_raw), float(msg.servo4_raw)]
                self.pub_pwm.publish(Float32MultiArray(data=pwms))
                self.qgc_out.mav.servo_output_raw_send(
                    msg.time_usec, msg.port,
                    msg.servo1_raw, msg.servo2_raw, msg.servo3_raw, msg.servo4_raw,
                    1000, 1000, 1000, 1000
                )


def main(args=None):
    rclpy.init(args=args)
    node = DroneMavlinkRos2Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
