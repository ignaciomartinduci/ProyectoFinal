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
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy
from dr_interfaces.msg import RobotState, TeachingDelta, ToolState


class TeachingNode(Node):

    def __init__(self):
        super().__init__("teaching_node")

        self.q0 = [0.0] * 6
        self.c0 = [0.0] * 6
        self._state_ready = threading.Event()

        self.cb_group = ReentrantCallbackGroup()
        latched_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.state_sub    = self.create_subscription(RobotState,     '/dr/robot_state',    self.callback_state_sub,    latched_qos, callback_group=self.cb_group)
        self.teaching_sub = self.create_subscription(TeachingDelta,  '/dr/teaching_delta', self.callback_teaching_sub, 10,          callback_group=self.cb_group)
        self.tool_state_pub = self.create_publisher(ToolState, '/dr/tool_state', 10)
        self.get_logger().info('Teaching node iniciado.')
        if os.environ.get('DR_DEBUG') == '1':
            print('\033[92mNODO teaching_node INICIADO CORRECTAMENTE\033[0m  sub: /dr/robot_state, /dr/teaching_delta  |  pub: /dr/tool_state', file=sys.stderr)

    def callback_state_sub(self, msg):
        self.q0 = list(msg.q)
        self.c0 = list(msg.pose)
        self._state_ready.set()
        if os.environ.get('DR_DEBUG_VERBOSE') == '1':
            G, R = '\033[92m', '\033[0m'
            print(f'{G}--> [teaching_node]{R} q0/c0 actualizados: q={[f"{v:.3f}" for v in self.q0]} c={[f"{v:.3f}" for v in self.c0]}', file=sys.stderr)

    def callback_teaching_sub(self, msg):
        if not self._state_ready.is_set():
            self.get_logger().warn('teaching_node: delta recibido sin estado del robot, esperando robot_state...')
            if os.environ.get('DR_DEBUG') == '1':
                G, R = '\033[92m', '\033[0m'
                print(f'{G}--> [teaching_node]{R} bloqueando hasta recibir robot_state...', file=sys.stderr)
            if not self._state_ready.wait(timeout=5.0):
                RED, R = '\033[91m', '\033[0m'
                while not self._state_ready.wait(timeout=1.0):
                    print(f'{RED}--> [teaching_node]{R} robot_state no está llegando...', file=sys.stderr)
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

        pose_final = [x, y, z, roll, pitch, yaw]
        tool_msg = ToolState()
        tool_msg.pose = pose_final
        self.tool_state_pub.publish(tool_msg)
        if os.environ.get('DR_DEBUG') == '1':
            G, R = '\033[92m', '\033[0m'
            c0_str = [f'{v:.3f}' for v in self.c0]
            c1_str = [f'{v:.3f}' for v in pose_final]
            print(f'{G}--> [teaching_node]{R} {c0_str} → {c1_str}', file=sys.stderr)

def main(args=None):
    rclpy.init(args=args)
    node = TeachingNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()