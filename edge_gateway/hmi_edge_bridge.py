import asyncio
import struct
import socket
import serial
import glob
import threading
import time
from bleak import BleakClient, BleakScanner

DEVICE_NAME    = "Nano33Sense-Rev2"
DEVICE_MAC     = "48:91:4D:98:5A:FE"
CHAR_TELEMETRY = "19B10001-E8F2-537E-4F6C-D104768A1214"

# 配信先1: QNX 8.0 RTOS (リアルタイム 60Hz 描画ルート)
QNX_TARGET  = ("10.0.0.169", 5006)
# 配信先2: Ubuntu ROS 2 Humble (自律制御・TF2 オドメトリ・K8s 連携ルート)
ROS2_TARGET = ("10.0.0.155", 5007)

udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

state = {
    "angle": 0.0,
    "pitch": 0.0,
    "rms":   1.0,
    "speed": 0.0,
    "rpm":   800.0,
    "mode":  0,
    "stm32_connected": False
}

def stm32_can_worker():
    """ST-LINK 固有パスから STM32 の疑似 CAN フレームを受信"""
    while True:
        ports = glob.glob("/dev/serial/by-id/*STMicroelectronics*")
        if not ports:
            state["stm32_connected"] = False
            time.sleep(2.0)
            continue
        try:
            with serial.Serial(ports[0], 115200, timeout=1.0) as ser:
                state["stm32_connected"] = True
                print(f"\n[STM32 CAN] Connected to ECU on {ports[0]}")
                while True:
                    line = ser.readline().decode('utf-8', errors='ignore').strip()
                    if line.startswith("CAN 0x0D0"):
                        parts = line.split()
                        if len(parts) >= 4:
                            raw = (int(parts[2], 16) << 8) | int(parts[3], 16)
                            state["speed"] = raw / 100.0
                    elif line.startswith("CAN 0x0C0"):
                        parts = line.split()
                        if len(parts) >= 4:
                            raw = (int(parts[2], 16) << 8) | int(parts[3], 16)
                            state["rpm"] = float(raw)
        except Exception:
            state["stm32_connected"] = False
            time.sleep(2.0)

threading.Thread(target=stm32_can_worker, daemon=True).start()

def ble_notification_handler(sender, data: bytearray):
    if len(data) >= 17:
        ts_ms, angle, pitch, rms, mode = struct.unpack("<IfffB", data[:17])
        state["angle"] = angle
        state["pitch"] = pitch
        state["rms"]   = rms
        state["mode"]  = mode

async def dual_streamer_loop():
    """QNX (16 bytes) と ROS 2 (24 bytes) へ 50Hz 周期で同時送信"""
    while True:
        qnx_pkt = struct.pack("<fffI", state["angle"], state["speed"], state["rpm"], state["mode"])
        ros_pkt = struct.pack("<fffffI", state["angle"], state["pitch"], state["rms"],
                              state["speed"], state["rpm"], state["mode"])
        try:
            udp_sock.sendto(qnx_pkt, QNX_TARGET)
            udp_sock.sendto(ros_pkt, ROS2_TARGET)
        except Exception:
            pass
        await asyncio.sleep(0.02)

async def main():
    asyncio.create_task(dual_streamer_loop())
    print(f"[HIL Edge] Dual Pipeline Started -> QNX:{QNX_TARGET} | ROS2:{ROS2_TARGET}")

    while True:
        try:
            device = await BleakScanner.find_device_by_address(DEVICE_MAC, timeout=5.0)
            if not device:
                device = await BleakScanner.find_device_by_name(DEVICE_NAME, timeout=5.0)
            if not device:
                await asyncio.sleep(1.0)
                continue

            async with BleakClient(device, timeout=10.0) as client:
                print(f"\n[BLE] Connected to Nano 33! Streaming to QNX & ROS 2...")
                await client.start_notify(CHAR_TELEMETRY, ble_notification_handler)
                while client.is_connected:
                    ecu_src = "STM32" if state["stm32_connected"] else "WAIT-ECU"
                    print(f"\r[HIL] Angle:{state['angle']:5.1f}° | Speed:{state['speed']:5.1f}km/h | RPM:{int(state['rpm']):4d} | RMS:{state['rms']:4.2f}G | ECU:{ecu_src}", end="", flush=True)
                    await asyncio.sleep(0.1)
        except Exception as e:
            print(f"\n[BLE] Reconnecting: {e}")
            await asyncio.sleep(2.0)

if __name__ == "__main__":
    asyncio.run(main())
