import rclpy
from rclpy.node import Node
from dr_interfaces.msg import RobotState, TeachingDelta, ToolState


class TeachingNode(Node):

    def __init__(self):
        super().__init__("teaching_node")

        self.q0 = [0.0] * 6
        self.c0 = [0.0] * 6

        self.state_sub = self.create_subscription(RobotState, '/dr/robot_state',self.callback_state_sub, 10)
        self.teaching_sub = self.create_subscription(TeachingDelta, '/dr/teaching_delta', self.callback_teaching_sub, 10)
        self.tool_state_pub = self.create_publisher(ToolState, '/dr/tool_state', 10)
        self.get_logger().info('Teaching node iniciado.')

    def callback_state_sub(self, msg):
        self.q0 = msg.q
        self.c0 = msg.pose

    def callback_teaching_sub(self, msg):
        dx = msg.dx
        dy = msg.dy
        dz = msg.dz
        droll = msg.droll
        dpitch = msg.dpitch
        dyaw = msg.dyaw

        x = self.c0[0] + dx
        y = self.c0[1] + dy
        z = self.c0[2] + dz
        roll = self.c0[3] + droll
        pitch = self.c0[4] + dpitch
        yaw = self.c0[5] + dyaw

        tool_msg = ToolState()
        tool_msg.pose = [x, y, z, roll, pitch, yaw]
        self.tool_state_pub.publish(tool_msg)

        pass

def main(args=None):
    rclpy.init(args=args)
    node = TeachingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()