import time
from pymavlink import mavutil

mav = mavutil.mavlink_connection('udpin:0.0.0.0:14540')
print("Waiting for PX4 HEARTBEAT on UDP 14540...")
mav.wait_heartbeat()
print(f"Connected! sys={mav.target_system} comp={mav.target_component}")

def send_vel_z(vx, vy, z_ned):
    # type_mask=0x05E3: 水平速度(vx,vy) + 高度(z) + yaw_rate を制御 (LOCAL_NED)
    mav.mav.set_position_target_local_ned_send(
        0, mav.target_system, mav.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, 0x05E3,
        0, 0, z_ned, vx, vy, 0, 0, 0, 0, 0, 0
    )

# 1. OFFBOARD 切替前に Setpoint を 1.5 秒間先行送信
for _ in range(30):
    send_vel_z(0.0, 0.0, -2.5); time.sleep(0.05)

# 2. OFFBOARD モードへ切替 & ARM
mav.mav.command_long_send(mav.target_system, mav.target_component,
    mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0, 1, 6, 0, 0, 0, 0, 0)
mav.mav.command_long_send(mav.target_system, mav.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
print("ARM & OFFBOARD sent -> Taking off to 2.5m...")

for i in range(160):  # 8秒間飛行 (前半4秒ホバリング、後半4秒右移動)
    vy = 0.0 if i < 80 else 0.8
    send_vel_z(0.0, vy, -2.5); time.sleep(0.05)

# 3. 着陸指令
mav.mav.command_long_send(mav.target_system, mav.target_component,
    mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, 0, 0)
print("LAND command sent.")
