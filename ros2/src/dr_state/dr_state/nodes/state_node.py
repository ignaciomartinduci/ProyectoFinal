import rclpy
from rclpy.node import Node
from dr_interfaces.msg import JointState, RobotState, ToolState
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy
from dr_interfaces.srv import SolveFK, SolveIK
import threading


class StateNode(Node):

    def __init__(self):
        super().__init__("state_node")

        self.declare_parameter("q0", rclpy.Parameter.Type.DOUBLE_ARRAY)

        self.q0 = list(self.get_parameter("q0").value)

        self.cb_group = ReentrantCallbackGroup()

        self.fk_client = self.create_client(
            SolveFK, "/dr/solve_fk", callback_group=self.cb_group
        )
        self.ik_client = self.create_client(
            SolveIK, "/dr/solve_ik", callback_group=self.cb_group
        )

        self.joint_state_sub = self.create_subscription(
            JointState,
            "/dr/joint_state",
            self.joint_state_callback,
            10,
            callback_group=self.cb_group,
        )
        self.tool_state_sub = self.create_subscription(
            ToolState,
            "/dr/tool_state",
            self.tool_state_callback,
            10,
            callback_group=self.cb_group,
        )

        latched_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.state_pub = self.create_publisher(
            RobotState, "/dr/robot_state", latched_qos
        )

        self._init_timer = self.create_timer(
            0.1, self.publish_initial_state, callback_group=self.cb_group
        )
        self.get_logger().info("State node iniciado.")

    def publish_initial_state(self):
        self._init_timer.cancel()
        self.get_logger().info(f"Publicando estado inicial para q0 = {self.q0}")
        self.fk_client.wait_for_service()

        request = SolveFK.Request()
        request.q = self.q0
        future = self.fk_client.call_async(request)
        event = threading.Event()
        future.add_done_callback(lambda f: event.set())
        event.wait()

        result = future.result()

        state = RobotState()
        state.q = self.q0
        state.qd = [0.0] * 6
        state.qdd = [0.0] * 6
        state.pose = list(result.pose)

        self.state_pub.publish(state)

    def joint_state_callback(self, msg):

        request = SolveFK.Request()
        request.q = list(msg.q)
        future = self.fk_client.call_async(request)
        event = threading.Event()
        future.add_done_callback(lambda f: event.set())
        event.wait()

        result = future.result()

        state = RobotState()
        state.q = list(msg.q)
        state.qd = list(msg.qd)
        state.qdd = list(msg.qdd)
        state.pose = list(result.pose)

        self.q0 = list(msg.q)
        self.state_pub.publish(state)

    def tool_state_callback(self, msg):

        request = SolveIK.Request()
        request.q0 = self.q0
        request.force = False
        request.pose = msg.pose

        future = self.ik_client.call_async(request)
        event = threading.Event()
        future.add_done_callback(lambda f: event.set())
        event.wait()

        result = future.result()

        if not result.success:
            self.get_logger().warn(f"IK falló para pose {list(msg.pose)}")
            return

        state = RobotState()
        state.q = list(result.q)
        state.qd = [0.0] * 6
        state.qdd = [0.0] * 6
        state.pose = list(msg.pose)

        self.q0 = list(result.q)
        self.state_pub.publish(state)


def main(args=None):

    rclpy.init(args=args)
    node = StateNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()
