import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from dr_interfaces.msg import RobotState
from sensor_msgs.msg import JointState

JOINT_NAMES = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
THETA_OFFSETS = [0.0, -1.5708, 0.0, 1.5708, 0.0, 0.0]


class JointStateBridgeNode(Node):

    def __init__(self):
        super().__init__("joint_state_bridge_node")
        latched_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(JointState, "/joint_states", 10)
        self.sub = self.create_subscription(
            RobotState, "/dr/robot_state", self.callback, latched_qos
        )
        self.last_q = [0.0] * 6
        self.last_qd = [0.0] * 6
        self.create_timer(0.05, self.timer_callback)
        self.get_logger().info("Joint state bridge iniciado.")

    def callback(self, msg):
        self.last_q = list(msg.q)
        self.last_qd = list(msg.qd)

    def timer_callback(self):
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = JOINT_NAMES
        js.position = [self.last_q[i] + THETA_OFFSETS[i] for i in range(6)]
        js.velocity = self.last_qd
        self.pub.publish(js)


def main(args=None):
    rclpy.init(args=args)
    node = JointStateBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
