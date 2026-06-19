import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from dr_interfaces.action import Execute
from dr_interfaces.msg import JointState
import numpy as np
import time

class ExecutorNode(Node):

    def __init__(self):
        super().__init__("executor_node")

        self.declare_parameter('ctrl_freq',    rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('q_max',        rclpy.Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('q_min',        rclpy.Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('qd_max',       rclpy.Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('qdd_max',      rclpy.Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('ef_v_max',     rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('ef_a_max',     rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('ef_omega_max', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('ef_alpha_max', rclpy.Parameter.Type.DOUBLE)

        self.ctrl_freq     = self.get_parameter('ctrl_freq').value
        self.ctrl_t        = 1.0 / self.ctrl_freq
        self.q_max         = list(self.get_parameter('q_max').value)
        self.q_min         = list(self.get_parameter('q_min').value)
        self.qd_max        = list(self.get_parameter('qd_max').value)
        self.qdd_max       = list(self.get_parameter('qdd_max').value)
        self.ef_v_max      = self.get_parameter('ef_v_max').value
        self.ef_a_max      = self.get_parameter('ef_a_max').value
        self.ef_omega_max  = self.get_parameter('ef_omega_max').value
        self.ef_alpha_max  = self.get_parameter('ef_alpha_max').value

        self.cb_group = ReentrantCallbackGroup()

        self.joint_state_pub = self.create_publisher(JointState, '/dr/joint_state', 10)
        self.action_server = ActionServer(self, Execute, '/dr/execute', self.execute_callback, callback_group=self.cb_group)

        self.get_logger().info('Executor node iniciado.')

    def execute_callback(self, goal_handle):
        q_traj_flat = np.array(goal_handle.request.q_traj)
        N = goal_handle.request.points
        q_traj  = q_traj_flat.reshape((N, 6))
        qd_traj  = np.gradient(q_traj,  self.ctrl_t, axis=0)
        qdd_traj = np.gradient(qd_traj, self.ctrl_t, axis=0)

        self.get_logger().info(f'Ejecutando trayectoria de {N} puntos ({N * self.ctrl_t:.2f}s).')

        for i in range(N):
            feedback = Execute.Feedback()
            feedback.q          = [float(x) for x in q_traj[i]]
            feedback.qd         = [float(x) for x in qd_traj[i]]
            feedback.qdd        = [float(x) for x in qdd_traj[i]]
            feedback.point_index = i
            feedback.progress   = float(i) / float(N)
            goal_handle.publish_feedback(feedback)

            joint_msg     = JointState()
            joint_msg.q   = [float(x) for x in q_traj[i]]
            joint_msg.qd  = [float(x) for x in qd_traj[i]]
            joint_msg.qdd = [float(x) for x in qdd_traj[i]]
            self.joint_state_pub.publish(joint_msg)

            time.sleep(self.ctrl_t)

        goal_handle.succeed()
        result = Execute.Result()
        result.success = True
        result.message = f'Trayectoria de {N} puntos ejecutada.'
        self.get_logger().info(result.message)
        return result


def main(args=None):
    rclpy.init(args=args)
    node = ExecutorNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()
