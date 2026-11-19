import os, glob, time, socket, struct, threading, serial

VM_HOST = "10.0.0.155"
VM_PORT = 5015
SYNC_MAGIC = b"\xAA\x55\xAA\x55"

class XiaoReader(threading.Thread):
    def __init__(self, name: str, port_path: str):
        super().__init__(daemon=True)
        self.name = name
        self.port_path = port_path
        self.lock = threading.Lock()
        self.latest_jpeg = None
        self.latest_ts = 0.0
        self.fps_count = 0

    def run(self):
        ser = serial.Serial()
        ser.port = self.port_path
        ser.baudrate = 921600
        ser.timeout = 0.5
        ser.dtr = True
        ser.rts = True
        ser.open()
        time.sleep(1.5)
        ser.reset_input_buffer()
        buf = bytearray()
        while True:
            chunk = ser.read(4096)
            if not chunk:
                continue
            buf.extend(chunk)
            while True:
                idx = buf.find(SYNC_MAGIC)
                if idx < 0:
                    if len(buf) > 4:
                        del buf[:-4]
                    break
                if idx > 0:
                    del buf[:idx]
                if len(buf) < 8:
                    break
                frame_len = struct.unpack("<I", buf[4:8])[0]
                if frame_len == 0 or frame_len > 100000:
                    del buf[:4]
                    continue
                if len(buf) < 8 + frame_len:
                    break
                jpeg = bytes(buf[8 : 8 + frame_len])
                del buf[: 8 + frame_len]
                with self.lock:
                    self.latest_jpeg = jpeg
                    self.latest_ts = time.time()
                    self.fps_count += 1

def main():
    devs = sorted(glob.glob("/dev/serial/by-id/*Espressif*"))
    if len(devs) < 2:
        print(f"[ERROR] XIAO ESP32S3 が2台見つかりません (検出数: {len(devs)})")
        raise SystemExit(1)

    left_port  = os.environ.get("LEFT_CAM", devs[0])
    right_port = os.environ.get("RIGHT_CAM", devs[1])
    print(f"Left  Camera: {left_port}")
    print(f"Right Camera: {right_port}")

    cam_l = XiaoReader("LEFT", left_port)
    cam_r = XiaoReader("RIGHT", right_port)
    cam_l.start()
    cam_r.start()

    last_log = time.time()
    while True:
        try:
            sock = socket.create_connection((VM_HOST, VM_PORT), timeout=3.0)
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            print(f"[CONNECTED] Streaming Stereo Pairs -> {VM_HOST}:{VM_PORT}")
            while True:
                time.sleep(0.05)
                with cam_l.lock:
                    jl, tl = cam_l.latest_jpeg, cam_l.latest_ts
                with cam_r.lock:
                    jr, tr = cam_r.latest_jpeg, cam_r.latest_ts

                if jl and jr and abs(tl - tr) < 0.15:
                    header = SYNC_MAGIC + struct.pack("<II", len(jl), len(jr))
                    sock.sendall(header + jl + jr)

                now = time.time()
                if now - last_log >= 2.0:
                    dt = now - last_log
                    print(f"[STEREO TX] Left: {cam_l.fps_count/dt:.1f} fps | Right: {cam_r.fps_count/dt:.1f} fps | Skew: {abs(tl-tr)*1000:.1f} ms")
                    cam_l.fps_count = 0
                    cam_r.fps_count = 0
                    last_log = now
        except (OSError, ConnectionError) as e:
            print(f"[WAITING] VM 220 ({VM_HOST}:{VM_PORT}) 受信待機中... ({e})")
            time.sleep(2.0)

if __name__ == "__main__":
    main()
