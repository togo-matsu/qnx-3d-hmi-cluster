import select, sys, termios, time, tty
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32, Float32
from geometry_msgs.msg import Twist

KEY_HELP = """
=== DJI Tello Safe Teleop (ROS 2) ===
  [t] 離陸 (takeoff)      [l] 着陸 (land)
  [Space] 即時ホバリング   [x] / [Ctrl+C] 緊急停止 (emergency)
  [w/s] 前進 / 後退        [a/d] 左移動 / 右移動
  [r/f] 上昇 / 下降        [q/e] 左旋回 / 右旋回
  ※移動キーを離すと 0.35 秒後に自動停止します (速度上限: 室内安全モード)
=====================================
"""

MOVE_BINDINGS = {
    'w': ( 0.6,  0.0,  0.0,  0.0),
    's': (-0.6,  0.0,  0.0,  0.0),
    'a': ( 0.0,  0.6,  0.0,  0.0),
    'd': ( 0.0, -0.6,  0.0,  0.0),
    'r': ( 0.0,  0.0,  0.6,  0.0),
    'f': ( 0.0,  0.0, -0.6,  0.0),
    'q': ( 0.0,  0.0,  0.0,  0.8),
    'e': ( 0.0,  0.0,  0.0, -0.8),
}

class TelloTeleopNode(Node):
    def __init__(self):
        super().__init__('tello_teleop_node')
        self.pub_cmd = self.create_publisher(String, '/tello/command', 10)
        self.pub_vel = self.create_publisher(Twist, '/tello/cmd_vel', 10)
        self.bat = -1
        self.height_m = 0.0
        self.create_subscription(Int32, '/tello/battery', self._on_bat, 10)
        self.create_subscription(Float32, '/tello/height_m', self._on_h, 10)

    def _on_bat(self, msg: Int32):
        self.bat = msg.data

    def _on_h(self, msg: Float32):
        self.height_m = msg.data

    def send_cmd(self, text: str):
        m = String(); m.data = text
        self.pub_cmd.publish(m)
        print(f"\r[CMD] {text:<10} | BAT: {self.bat:>3}% | H: {self.height_m:.2f}m   ", flush=True)

    def send_vel(self, lx=0.0, ly=0.0, lz=0.0, az=0.0):
        t = Twist()
        t.linear.x, t.linear.y, t.linear.z = float(lx), float(ly), float(lz)
        t.angular.z = float(az)
        self.pub_vel.publish(t)

def main(args=None):
    rclpy.init(args=args)
    node = TelloTeleopNode()
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    print(KEY_HELP, flush=True)

    last_move_ts = 0.0
    moving = False
    try:
        tty.setcbreak(fd)
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)
            rlist, _, _ = select.select([sys.stdin], [], [], 0.03)
            now = time.time()
            if rlist:
                ch = sys.stdin.read(1).lower()
                if ch == '\x03' or ch == 'x':
                    node.send_cmd('emergency')
                    if ch == '\x03':
                        break
                elif ch == 't':
                    node.send_cmd('takeoff')
                elif ch == 'l':
                    moving = False
                    node.send_cmd('land')
                elif ch == ' ':
                    node.send_vel(0, 0, 0, 0)
                    node.send_cmd('hover')
                    moving = False
                elif ch in MOVE_BINDINGS:
                    lx, ly, lz, az = MOVE_BINDINGS[ch]
                    node.send_vel(lx, ly, lz, az)
                    last_move_ts = now
                    moving = True
                    print(f"\r[VEL] key={ch} ({lx:+.1f},{ly:+.1f},{lz:+.1f},{az:+.1f}) | BAT:{node.bat:>3}% | H:{node.height_m:.2f}m ", end="", flush=True)
            elif moving and (now - last_move_ts > 0.35):
                node.send_vel(0, 0, 0, 0)
                moving = False
                print(f"\r[AUTO-HOVER] stop | BAT: {node.bat:>3}% | H: {node.height_m:.2f}m        ", end="", flush=True)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        print("\nTeleop exited safely.")
        node.destroy_node()
        rclpy.ok() and rclpy.shutdown()
