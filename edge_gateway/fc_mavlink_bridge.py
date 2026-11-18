import os
os.environ["MAVLINK20"] = "1"

import re
import math
import glob
import time
import serial
from pymavlink import mavutil

TARGET_UDP = "udpout:10.0.0.155:14550"

stm_ports  = sorted(glob.glob("/dev/serial/by-id/*STMicroelectronics*"))
nano_ports = sorted(glob.glob("/dev/serial/by-id/*Arduino*"))

if not stm_ports:
    print("[ERROR] STM32 Nucleo が見つかりません。")
    raise SystemExit(1)

stm  = serial.Serial(stm_ports[0], 115200, timeout=0.01)
nano = serial.Serial(nano_ports[0], 115200, timeout=0.01) if nano_ports else None
mav  = mavutil.mavlink_connection(TARGET_UDP, source_system=1, source_component=1)

pattern = re.compile(
    r"STATE=(ARMED|DISARMED)\s+MODE=(OFFBOARD|STABILIZ)\s+IMU=(OK|WAIT)\s+"
    r"R=([-\d.]+)\s+P=([-\d.]+)\s+Y=([-\d.]+)\s+ALT=([-\d.]+)\s+"
    r"PWM=\[M1:(\d+)\s+M2:(\d+)\s+M3:(\d+)\s+M4:(\d+)\]"
)

def quat_to_euler_deg(q):
    w, x, y, z = q[0], q[1], q[2], q[3]
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.degrees(math.atan2(sinr_cosp, cosr_cosp))

    sinp = 2.0 * (w * y - z * x)
    pitch = math.degrees(math.copysign(math.pi / 2, sinp) if abs(sinp) >= 1 else math.asin(sinp))
    return roll, pitch

start_time = time.time()
last_hb = 0.0

print(f"=== Bidirectional MAVLink v2 Bridge Started <-> {TARGET_UDP} ===")

try:
    while True:
        # 1. Nano 33 -> STM32 中継
        if nano and nano.in_waiting:
            imu_line = nano.readline()
            if b"$IMU," in imu_line:
                stm.write(imu_line)

        # 2. VM 220 (ROS 2) -> STM32 Offboard / Arm / E-STOP 指令の受信・転送
        while True:
            rx = mav.recv_match(blocking=False)
            if rx is None:
                break
            rtype = rx.get_type()
            if rtype == "SET_ATTITUDE_TARGET":
                r_sp, p_sp = quat_to_euler_deg(rx.q)
                thr_pwm = 1150.0 + max(0.0, min(1.0, float(rx.thrust))) * 700.0
                cmd = f"$CMD,-1,{r_sp:.2f},{p_sp:.2f},{thr_pwm:.1f},0\n"
                stm.write(cmd.encode("ascii"))
            elif rtype == "COMMAND_LONG":
                if rx.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
                    arm_flag = 1 if int(rx.param1) == 1 else 0
                    stm.write(f"$CMD,{arm_flag},0.0,0.0,1500.0,0\n".encode("ascii"))
                elif rx.command == mavutil.mavlink.MAV_CMD_DO_FLIGHTTERMINATION:
                    stm.write(b"$CMD,0,0.0,0.0,1000.0,1\n")

        # 3. STM32 -> VM 220 (MAVLink テレメトリ送信)
        if stm.in_waiting:
            line = stm.readline().decode("utf-8", errors="ignore").strip()
            if line.startswith(">>>"):
                print(line)
            m = pattern.search(line)
            if m:
                armed = (m.group(1) == "ARMED")
                offboard = (m.group(2) == "OFFBOARD")
                roll_deg  = float(m.group(4))
                pitch_deg = float(m.group(5))
                yaw_deg   = float(m.group(6))
                alt_m     = float(m.group(7))
                m1, m2, m3, m4 = int(m.group(8)), int(m.group(9)), int(m.group(10)), int(m.group(11))

                now = time.time()
                boot_ms = int((now - start_time) * 1000) & 0xFFFFFFFF
                boot_us = int((now - start_time) * 1e6) & 0xFFFFFFFF

                if now - last_hb >= 1.0:
                    last_hb = now
                    base_mode = (
                        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
                        | mavutil.mavlink.MAV_MODE_FLAG_STABILIZE_ENABLED
                    )
                    if offboard:
                        base_mode |= mavutil.mavlink.MAV_MODE_FLAG_GUIDED_ENABLED
                    if armed:
                        base_mode |= mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                    sys_state = mavutil.mavlink.MAV_STATE_ACTIVE if armed else mavutil.mavlink.MAV_STATE_STANDBY
                    mav.mav.heartbeat_send(
                        mavutil.mavlink.MAV_TYPE_QUADROTOR,
                        mavutil.mavlink.MAV_AUTOPILOT_PX4,
                        base_mode,
                        6 if offboard else 0,
                        sys_state,
                    )

                mav.mav.attitude_send(
                    boot_ms, math.radians(roll_deg), math.radians(pitch_deg), math.radians(yaw_deg), 0.0, 0.0, 0.0
                )
                avg_pwm = (m1 + m2 + m3 + m4) / 4.0
                throttle_pct = max(0, min(100, int((avg_pwm - 1000) / 10)))
                mav.mav.vfr_hud_send(0.0, 0.0, int(yaw_deg) % 360, throttle_pct, alt_m, 0.0)
                mav.mav.servo_output_raw_send(boot_us, 0, m1, m2, m3, m4, 1000, 1000, 1000, 1000)

except KeyboardInterrupt:
    print("\nBidirectional MAVLink Bridge stopped.")
finally:
    stm.close()
    if nano:
        nano.close()
