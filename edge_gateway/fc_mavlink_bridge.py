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

stm  = serial.Serial(stm_ports[0], 115200, timeout=0.02)
nano = serial.Serial(nano_ports[0], 115200, timeout=0.02) if nano_ports else None
mav  = mavutil.mavlink_connection(TARGET_UDP, source_system=1, source_component=1)

pattern = re.compile(
    r"STATE=(ARMED|DISARMED)\s+IMU=(OK|WAIT)\s+"
    r"R=([-\d.]+)\s+P=([-\d.]+)\s+Y=([-\d.]+)\s+ALT=([-\d.]+)\s+"
    r"PWM=\[M1:(\d+)\s+M2:(\d+)\s+M3:(\d+)\s+M4:(\d+)\]"
)

start_time = time.time()
last_hb = 0.0

print(f"=== MAVLink v2 Bridge Started -> {TARGET_UDP} ===")
print("STM32の青ボタン(B1)で ARMED/DISARMED 切替、Nano 33 を傾けて QGC の HUD を確認 (Ctrl+C で終了)")

try:
    while True:
        # Nano 33 -> STM32 中継
        if nano and nano.in_waiting:
            imu_line = nano.readline()
            if b"$IMU," in imu_line:
                stm.write(imu_line)

        # STM32 -> MAVLink パケット変換送信
        if stm.in_waiting:
            line = stm.readline().decode("utf-8", errors="ignore").strip()
            m = pattern.search(line)
            if m:
                armed = (m.group(1) == "ARMED")
                roll_deg  = float(m.group(3))
                pitch_deg = float(m.group(4))
                yaw_deg   = float(m.group(5))
                alt_m     = float(m.group(6))
                m1, m2, m3, m4 = int(m.group(7)), int(m.group(8)), int(m.group(9)), int(m.group(10))

                now = time.time()
                boot_ms = int((now - start_time) * 1000) & 0xFFFFFFFF
                boot_us = int((now - start_time) * 1e6) & 0xFFFFFFFF

                # 1. HEARTBEAT (1Hz: 機体種別と ARMED 状態を通知)
                if now - last_hb >= 1.0:
                    last_hb = now
                    base_mode = (
                        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
                        | mavutil.mavlink.MAV_MODE_FLAG_STABILIZE_ENABLED
                    )
                    if armed:
                        base_mode |= mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                    sys_state = (
                        mavutil.mavlink.MAV_STATE_ACTIVE
                        if armed
                        else mavutil.mavlink.MAV_STATE_STANDBY
                    )
                    mav.mav.heartbeat_send(
                        mavutil.mavlink.MAV_TYPE_QUADROTOR,
                        mavutil.mavlink.MAV_AUTOPILOT_PX4,
                        base_mode,
                        0,
                        sys_state,
                    )

                # 2. ATTITUDE (姿勢角: ラジアン単位)
                mav.mav.attitude_send(
                    boot_ms,
                    math.radians(roll_deg),
                    math.radians(pitch_deg),
                    math.radians(yaw_deg),
                    0.0,
                    0.0,
                    0.0,
                )

                # 3. VFR_HUD (方位・スロットル%・気圧高度)
                avg_pwm = (m1 + m2 + m3 + m4) / 4.0
                throttle_pct = max(0, min(100, int((avg_pwm - 1000) / 10)))
                heading_deg = int(yaw_deg) % 360
                mav.mav.vfr_hud_send(0.0, 0.0, heading_deg, throttle_pct, alt_m, 0.0)

                # 4. SERVO_OUTPUT_RAW (4ch モーターPWM出力)
                mav.mav.servo_output_raw_send(
                    boot_us, 0, m1, m2, m3, m4, 1000, 1000, 1000, 1000
                )

except KeyboardInterrupt:
    print("\nMAVLink ブリッジを終了しました。")
finally:
    stm.close()
    if nano:
        nano.close()
