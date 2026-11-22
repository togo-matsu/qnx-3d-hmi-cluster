import json, socket, threading, time

TELLO_IP = "192.168.10.1"
TELLO_CMD_PORT = 8889
TELLO_STATE_PORT = 8890
TELLO_VIDEO_PORT = 11111

VM_HOST = "10.0.0.155"
VM_CTRL_PORT = 5020
VM_VIDEO_PORT = 5021

class TelloEdgeBridge:
    def __init__(self):
        self.sock_cmd = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock_cmd.bind(("0.0.0.0", TELLO_CMD_PORT))
        self.sock_cmd.settimeout(1.0)

        self.sock_state = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock_state.bind(("0.0.0.0", TELLO_STATE_PORT))

        self.sock_video_in = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock_video_in.bind(("0.0.0.0", TELLO_VIDEO_PORT))
        self.sock_video_in.settimeout(1.0)
        self.sock_video_out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.lock = threading.Lock()
        self.cmd_lock = threading.Lock()
        self.state = {"bat": 0, "h": 0, "tof": 0, "temph": 0, "pitch": 0, "roll": 0, "yaw": 0}
        self.tcp_sock = None
        self.is_flying = False
        self.landing_active = False
        self.last_takeoff_ts = 0.0
        self.last_rc_ts = 0.0
        self.hover_sent = True
        self.video_pkts = 0
        self.state_pkts = 0
        self.last_video_ts = 0.0
        self.last_rx_resp = ""
        self.last_rx_ts = 0.0

    def send_tello_cmd(self, cmd: str):
        with self.cmd_lock:
            try:
                self.sock_cmd.sendto(cmd.encode("utf-8"), (TELLO_IP, TELLO_CMD_PORT))
                if not cmd.startswith("rc "):
                    print(f"[TELLO TX] {cmd}", flush=True)
            except OSError as e:
                print(f"[TELLO ERR] {e}", flush=True)

    def cmd_rx_loop(self):
        while True:
            try:
                data, _ = self.sock_cmd.recvfrom(1024)
                resp = data.decode("utf-8", errors="ignore").strip()
                if resp:
                    with self.lock:
                        self.last_rx_resp = resp
                        self.last_rx_ts = time.time()
                    print(f"[TELLO RX] {resp}", flush=True)
            except socket.timeout:
                pass
            except OSError:
                time.sleep(0.05)

    def _land_retry_worker(self):
        self.send_tello_cmd("rc 0 0 0 0")
        time.sleep(0.15)
        for attempt in range(1, 5):
            with self.lock:
                if not self.landing_active:
                    return
            sent_ts = time.time()
            self.send_tello_cmd("land")
            deadline = sent_ts + 5.0
            while time.time() < deadline:
                with self.lock:
                    if not self.landing_active:
                        return
                    rx_ts = self.last_rx_ts
                    rx_resp = self.last_rx_resp.lower()
                if rx_ts >= sent_ts:
                    if rx_resp == "ok":
                        print(f"[LAND CONFIRMED] ok (attempt {attempt})", flush=True)
                        with self.lock:
                            self.landing_active = False
                        return
                    if "error" in rx_resp:
                        print(f"[LAND RETRY] rejected ({rx_resp}) -> retry ({attempt}/4)", flush=True)
                        time.sleep(0.5)
                        break
                time.sleep(0.05)
        with self.lock:
            self.landing_active = False

    def video_relay_loop(self):
        last_log = time.time()
        while True:
            try:
                data, _ = self.sock_video_in.recvfrom(4096)
                self.sock_video_out.sendto(data, (VM_HOST, VM_VIDEO_PORT))
                self.video_pkts += 1
                self.last_video_ts = time.time()
            except socket.timeout:
                pass
            except OSError:
                time.sleep(0.01)

            now = time.time()
            if now - last_log >= 2.0:
                with self.lock:
                    bat = self.state.get("bat", 0)
                    temph = self.state.get("temph", 0)
                    h = self.state.get("h", 0)
                    tof = self.state.get("tof", 0)
                print(
                    f"[RELAY] video={self.video_pkts} pkts/2s | state={self.state_pkts} pkts/2s | bat={bat}% | temp={temph}C | h={h}cm | tof={tof}cm",
                    flush=True,
                )
                self.video_pkts = 0
                self.state_pkts = 0
                last_log = now

    def state_rx_loop(self):
        while True:
            try:
                data, _ = self.sock_state.recvfrom(1024)
                raw = data.decode("utf-8", errors="ignore").strip()
                parsed = {}
                for item in raw.split(";"):
                    if ":" in item:
                        k, v = item.split(":", 1)
                        try:
                            parsed[k] = float(v) if "." in v else int(v)
                        except ValueError:
                            parsed[k] = v
                if parsed:
                    with self.lock:
                        self.state.update(parsed)
                        if int(self.state.get("h", 0)) > 10 and not self.landing_active:
                            self.is_flying = True
                    self.state_pkts += 1
            except OSError:
                time.sleep(0.05)

    def failsafe_loop(self):
        last_keepalive = 0.0
        while True:
            now = time.time()
            with self.lock:
                busy = self.landing_active or (now - self.last_takeoff_ts < 5.0)
                connected = self.tcp_sock is not None
                flying = self.is_flying
                rc_age = now - self.last_rc_ts
                need_hover = flying and (not self.landing_active) and (not connected or rc_age > 0.5) and not self.hover_sent

            if not busy and (now - last_keepalive >= 10.0):
                if now - self.last_video_ts > 4.0:
                    self.send_tello_cmd("command")
                    time.sleep(0.3)
                    self.send_tello_cmd("streamon")
                else:
                    self.send_tello_cmd("battery?")
                last_keepalive = now

            if need_hover:
                self.send_tello_cmd("rc 0 0 0 0")
                with self.lock:
                    self.hover_sent = True
                print("[FAILSAFE] Auto-hover triggered", flush=True)

            time.sleep(0.05)

    def handle_vm_command(self, line: str):
        try:
            msg = json.loads(line)
        except json.JSONDecodeError:
            return
        cmd = str(msg.get("cmd", "")).strip()
        if not cmd:
            return
        if not cmd.startswith("rc "):
            print(f"[VM CMD RX] {cmd}", flush=True)
        with self.lock:
            bat = int(self.state.get("bat", 0))
            now = time.time()

        if cmd == "takeoff":
            if bat < 20:
                print(f"[SAFETY BLOCK] Takeoff denied: low battery ({bat}% < 20%)", flush=True)
                return
            with self.lock:
                if now - self.last_takeoff_ts < 4.0:
                    print("[DEBOUNCE] Duplicate takeoff ignored", flush=True)
                    return
                self.last_takeoff_ts = now
                self.landing_active = False
                self.is_flying = True
                self.hover_sent = True
            self.send_tello_cmd("takeoff")
        elif cmd == "land":
            with self.lock:
                already_landing = self.landing_active
                self.is_flying = False
                self.hover_sent = True
                self.landing_active = True
            if not already_landing:
                threading.Thread(target=self._land_retry_worker, daemon=True).start()
            else:
                print("[LAND] already landing; duplicate ignored", flush=True)
        elif cmd == "emergency":
            with self.lock:
                self.landing_active = False
                self.is_flying = False
                self.hover_sent = True
            for _ in range(3):
                self.send_tello_cmd("emergency")
                time.sleep(0.05)
        elif cmd.startswith("rc "):
            with self.lock:
                if self.landing_active:
                    return
                self.last_rc_ts = time.time()
                self.hover_sent = (cmd == "rc 0 0 0 0")
            self.send_tello_cmd(cmd)

    def tcp_rx_worker(self, sock: socket.socket):
        buf = b""
        while True:
            try:
                chunk = sock.recv(1024)
                if not chunk:
                    break
                buf += chunk
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    self.handle_vm_command(line.decode("utf-8", errors="ignore"))
            except socket.timeout:
                continue
            except OSError as e:
                print(f"[TCP RX END] {e}", flush=True)
                break

    def run(self):
        threading.Thread(target=self.cmd_rx_loop, daemon=True).start()
        threading.Thread(target=self.video_relay_loop, daemon=True).start()
        threading.Thread(target=self.state_rx_loop, daemon=True).start()
        threading.Thread(target=self.failsafe_loop, daemon=True).start()

        while True:
            try:
                sock = socket.create_connection((VM_HOST, VM_CTRL_PORT), timeout=3.0)
                sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                sock.settimeout(None)
                with self.lock:
                    self.tcp_sock = sock
                print(f"[CONNECTED] Tello Bridge -> {VM_HOST}:{VM_CTRL_PORT} (Video UDP:{VM_VIDEO_PORT})", flush=True)
                threading.Thread(target=self.tcp_rx_worker, args=(sock,), daemon=True).start()

                while True:
                    with self.lock:
                        payload = json.dumps(self.state).encode("utf-8") + b"\n"
                    sock.sendall(payload)
                    time.sleep(0.1)
            except (OSError, ConnectionError) as e:
                with self.lock:
                    self.tcp_sock = None
                print(f"[WAITING] VM 220 ({VM_HOST}:{VM_CTRL_PORT}) 待機中... ({e})", flush=True)
                time.sleep(2.0)

if __name__ == "__main__":
    TelloEdgeBridge().run()
