# Copyright 2026 Ignacio Martín Duci
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to
# deal in the Software without restriction, including without limitation the
# rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
# sell copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
# FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
# DEALINGS IN THE SOFTWARE.

import os
import sys
import threading
import rclpy
from rclpy.node import Node
from dr_interfaces.msg import JointState, RobotState, ToolState
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy
from dr_interfaces.srv import SolveFK, SolveIK

class StateNode(Node):

    def __init__(self):
        super().__init__("state_node")

        self.declare_parameter('q0', rclpy.Parameter.Type.DOUBLE_ARRAY)
        self.q0 = list(self.get_parameter('q0').value)

        self.cb_group = ReentrantCallbackGroup()

        self.fk_client = self.create_client(SolveFK, '/dr/solve_fk', callback_group=self.cb_group)
        self.ik_client = self.create_client(SolveIK, '/dr/solve_ik', callback_group=self.cb_group)

        self.joint_state_sub = self.create_subscription(
            JointState, '/dr/joint_state', self.joint_state_callback, 10, callback_group=self.cb_group)
        self.tool_state_sub = self.create_subscription(
            ToolState, '/dr/tool_state', self.tool_state_callback, 10, callback_group=self.cb_group)

        latched_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.state_pub = self.create_publisher(RobotState, '/dr/robot_state', latched_qos)

        self._init_timer = self.create_timer(0.1, self.publish_initial_state, callback_group=self.cb_group)
        self.get_logger().info('State node iniciado.')
        if os.environ.get('DR_DEBUG') == '1':
            print('\033[92mNODO state_node INICIADO CORRECTAMENTE\033[0m  sub: /dr/joint_state  |  pub: /dr/robot_state', file=sys.stderr)

    def publish_initial_state(self):
        self._init_timer.cancel()
        self.get_logger().info(f'Publicando estado inicial para q0 = {self.q0}')
        self.fk_client.wait_for_service()
        request = SolveFK.Request()
        request.q = self.q0
        future = self.fk_client.call_async(request)
        future.add_done_callback(self._on_initial_fk_done)

    def _on_initial_fk_done(self, future):
        result = future.result()
        if result is None:
            return
        state = RobotState()
        state.q   = self.q0
        state.qd  = [0.0] * 6
        state.qdd = [0.0] * 6
        state.pose = list(result.pose)
        self.state_pub.publish(state)

    def joint_state_callback(self, msg):
        q   = list(msg.q)
        qd  = list(msg.qd)
        qdd = list(msg.qdd)
        self.q0 = q
        request = SolveFK.Request()
        request.q = q
        future = self.fk_client.call_async(request)
        future.add_done_callback(lambda f: self._on_fk_done(f, q, qd, qdd))

    def _on_fk_done(self, future, q, qd, qdd):
        result = future.result()
        if result is None:
            return
        state = RobotState()
        state.q   = q
        state.qd  = qd
        state.qdd = qdd
        state.pose = list(result.pose)
        self.state_pub.publish(state)
        if os.environ.get('DR_DEBUG_VERBOSE') == '1':
            G, R = '\033[92m', '\033[0m'
            print(f'{G}--> [state_node]{R} FK completado → pose={[f"{v:.3f}" for v in result.pose]}', file=sys.stderr)

    def tool_state_callback(self, msg):
        debug = os.environ.get('DR_DEBUG') == '1'
        G, RED, R = '\033[92m', '\033[91m', '\033[0m'
        PREFIX = f'{G}--> [state_node]{R}'

        pose = list(msg.pose)
        if debug:
            print(f'{PREFIX} tool_state recibido: pose={[f"{v:.3f}" for v in pose]}', file=sys.stderr)

        request = SolveIK.Request()
        request.q0    = self.q0
        request.force = False
        request.pose  = pose
        future = self.ik_client.call_async(request)
        future.add_done_callback(lambda f: self._on_ik_done(f, pose, debug, PREFIX, RED, R))

    def _on_ik_done(self, future, pose, debug, PREFIX, RED, R):
        result = future.result()
        if result is None:
            return
        if not result.success:
            self.get_logger().warn(f'IK falló para pose {pose}')
            if debug:
                print(f'{RED}--> [state_node]{R} IK falló para pose={[f"{v:.3f}" for v in pose]}', file=sys.stderr)
            return
        state = RobotState()
        state.q   = list(result.q)
        state.qd  = [0.0] * 6
        state.qdd = [0.0] * 6
        state.pose = pose
        self.q0 = list(result.q)
        self.state_pub.publish(state)
        if debug:
            print(f'{PREFIX} robot_state publicado tras IK: q={[f"{v:.3f}" for v in result.q]}', file=sys.stderr)


def main(args=None):
    rclpy.init(args=args)
    node = StateNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()
