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
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from dr_interfaces.msg import RobotState
from sensor_msgs.msg import JointState

JOINT_NAMES    = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
THETA_OFFSETS  = [0.0, -1.5708, 0.0, 1.5708, 0.0, 0.0]

class JointStateBridgeNode(Node):

    def __init__(self):
        super().__init__('joint_state_bridge_node')
        latched_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(JointState, '/joint_states', 10)
        self.sub = self.create_subscription(RobotState, '/dr/robot_state', self.callback, latched_qos)
        self.last_q  = [0.0] * 6
        self.last_qd = [0.0] * 6
        self._first_bridged = False
        self.create_timer(0.05, self.timer_callback)
        self.get_logger().info('Joint state bridge iniciado.')
        if os.environ.get('DR_DEBUG') == '1':
            print('\033[92mNODO joint_state_bridge_node INICIADO CORRECTAMENTE\033[0m  sub: /dr/robot_state  |  pub: /joint_states', file=sys.stderr)

    def callback(self, msg):
        self.last_q  = list(msg.q)
        self.last_qd = list(msg.qd)
        if not self._first_bridged and os.environ.get('DR_DEBUG') == '1':
            self._first_bridged = True
            G, R = '\033[92m', '\033[0m'
            print(f'{G}--> [joint_state_bridge_node]{R} primer robot_state recibido: q={[f"{v:.3f}" for v in self.last_q]}', file=sys.stderr)

    def timer_callback(self):
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = JOINT_NAMES
        js.position = [self.last_q[i] + THETA_OFFSETS[i] for i in range(6)]
        js.velocity  = self.last_qd
        self.pub.publish(js)

def main(args=None):
    rclpy.init(args=args)
    node = JointStateBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()