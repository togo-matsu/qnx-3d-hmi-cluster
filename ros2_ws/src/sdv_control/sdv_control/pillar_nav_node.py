import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from visualization_msgs.msg import Marker


# turtlebot3_world の 9本の柱 (P1..P9) と ヘキサ6頂点 (H1..H6) の安全停止座標 (map座標系)
TARGET_WAYPOINTS = {
    # 上段 (前傾: Pitch > +15 deg)
    'P1': (1.10,  0.55),   # 上段・左の柱の横
    'P2': (0.55,  0.00),   # 上段・中の柱の手前
    'P3': (1.10, -0.55),   # 上段・右の柱の横
    # 中段 (水平: -15 <= Pitch <= +15 deg)
    'P4': (0.00,  0.55),   # 中段・左の柱の横
    'P5': (-0.55, 0.00),   # 中段・中の柱の手前
    'P6': (0.00, -0.55),   # 中段・右の柱の横
    # 下段 (手前傾: Pitch < -15 deg)
    'P7': (-1.10,  0.55),  # 下段・左の柱の横
    'P8': (-1.65,  0.00),  # 下段・中の柱の手前
    'P9': (-1.10, -0.55),  # 下段・右の柱の横
    # ヘキサゴン 6頂点 (将来拡張用)
    'H1': (1.65,  0.00),   # 上頂点
    'H2': (0.85,  1.35),   # 左上頂点
    'H3': (-0.85, 1.35),   # 左下頂点
    'H4': (-1.75, 0.00),   # 下頂点
    'H5': (-0.85, -1.35),  # 右下頂点
    'H6': (0.85, -1.35),   # 右上頂点
}


class PillarNavNode(Node):
    def __init__(self):
        super().__init__('pillar_nav_node')
        self.selected_target = 'P5'
        self.is_moving = False
        self.active_goal_handle = None

        self._action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.marker_pub = self.create_publisher(Marker, '/target_marker', 10)

        self.create_subscription(String, '/nav/select_target', self.select_cb, 10)
        self.create_subscription(String, '/nav/execute_go', self.execute_cb, 10)
        self.create_timer(0.5, self.publish_marker)

        self.get_logger().info(
            f'PillarNavNode ready. Initial target: [{self.selected_target}] {TARGET_WAYPOINTS[self.selected_target]}'
        )

    def select_cb(self, msg: String):
        target = msg.data.strip().upper()
        if target in TARGET_WAYPOINTS:
            if self.selected_target != target:
                self.selected_target = target
                x, y = TARGET_WAYPOINTS[target]
                self.get_logger().info(f'Target selected -> [{target}] (x={x:.2f}, y={y:.2f})')
                self.publish_marker()
        else:
            self.get_logger().warn(f'Unknown target ignored: "{target}"')

    def execute_cb(self, msg: String):
        override = msg.data.strip().upper()
        if override in TARGET_WAYPOINTS:
            self.selected_target = override
            self.publish_marker()

        target = self.selected_target
        x, y = TARGET_WAYPOINTS[target]

        if not self._action_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('Nav2 /navigate_to_pose action server not available!')
            return

        if self.active_goal_handle is not None:
            self.get_logger().info('Canceling previous active goal before sending new target...')
            self.active_goal_handle.cancel_goal_async()
            self.active_goal_handle = None

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.w = 1.0

        self.is_moving = True
        self.get_logger().info(f'>>> GO COMMAND: Moving to [{target}] (x={x:.2f}, y={y:.2f})')
        send_future = self._action_client.send_goal_async(goal_msg)
        send_future.add_done_callback(lambda f, t=target: self.goal_response_cb(f, t))

    def goal_response_cb(self, future, target: str):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.is_moving = False
            self.get_logger().error(f'Goal [{target}] was REJECTED by Nav2.')
            return

        self.active_goal_handle = goal_handle
        self.get_logger().info(f'Goal [{target}] ACCEPTED. Autonomous driving in progress...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda f, t=target: self.get_result_cb(f, t))

    def get_result_cb(self, future, target: str):
        status = future.result().status
        self.is_moving = False
        self.active_goal_handle = None
        # GoalStatus.STATUS_SUCCEEDED == 4
        if status == 4:
            self.get_logger().info(f'=== ARRIVED at [{target}]! Ready for next command. ===')
        else:
            self.get_logger().warn(f'Navigation to [{target}] ended with status code: {status}')

    def publish_marker(self):
        x, y = TARGET_WAYPOINTS[self.selected_target]
        m = Marker()
        m.header.frame_id = 'map'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'pillar_target'
        m.id = 0
        m.type = Marker.CYLINDER
        m.action = Marker.ADD
        m.pose.position.x = float(x)
        m.pose.position.y = float(y)
        m.pose.position.z = 0.25
        m.pose.orientation.w = 1.0
        m.scale.x = 0.25
        m.scale.y = 0.25
        m.scale.z = 0.50
        if self.is_moving:
            m.color.r, m.color.g, m.color.b, m.color.a = 0.0, 1.0, 0.4, 0.85
        else:
            m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.8, 0.0, 0.85
        self.marker_pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = PillarNavNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
