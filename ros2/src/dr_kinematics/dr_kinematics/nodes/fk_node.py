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
from dr_interfaces.srv import SolveFK
from dr_kinematics.forw_kinematics import ForwKinematics

class FKNode(Node):

    def __init__(self):
        super().__init__("fk_node")

                # Declaración de parámetros

        # Arrays DH - sin default, tipo explícito
        self.declare_parameter('dh_theta_offset', rclpy.Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('dh_d',            rclpy.Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('dh_a',            rclpy.Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('dh_alpha',        rclpy.Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('q_min',           rclpy.Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('q_max',           rclpy.Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('qd_max',          rclpy.Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('qdd_max',         rclpy.Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('T_base.xyz',      rclpy.Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('T_base.rpy',      rclpy.Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('T_tool.xyz',      rclpy.Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('T_tool.rpy',      rclpy.Parameter.Type.DOUBLE_ARRAY)

        # Escalares - con default está bien
        self.declare_parameter('ef_v_max',     rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('ef_a_max',     rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('ef_omega_max', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('ef_alpha_max', rclpy.Parameter.Type.DOUBLE)

        # Construir diccionario de parámetros
        params = {
            'q_min':            self.get_parameter('q_min').value,
            'q_max':            self.get_parameter('q_max').value,
            'qd_max':           self.get_parameter('qd_max').value,
            'qdd_max':          self.get_parameter('qdd_max').value,
            'dh_theta_offset':  self.get_parameter('dh_theta_offset').value,
            'dh_d':             self.get_parameter('dh_d').value,
            'dh_a':             self.get_parameter('dh_a').value,
            'dh_alpha':         self.get_parameter('dh_alpha').value,
            'ef_v_max':         self.get_parameter('ef_v_max').value,
            'ef_a_max':         self.get_parameter('ef_a_max').value,
            'ef_omega_max':     self.get_parameter('ef_omega_max').value,
            'ef_alpha_max':     self.get_parameter('ef_alpha_max').value,
            'T_base_xyz':   self.get_parameter('T_base.xyz').value,
            'T_base_rpy':   self.get_parameter('T_base.rpy').value,
            'T_tool_xyz':   self.get_parameter('T_tool.xyz').value,
            'T_tool_rpy':   self.get_parameter('T_tool.rpy').value,

        }

        self.solver = ForwKinematics(params)
        self.srv = self.create_service(SolveFK, '/dr/solve_fk', self.solve_fk_callback)
        self.get_logger().info('FK node iniciado.')
        if os.environ.get('DR_DEBUG') == '1':
            print('\033[92mNODO fk_node INICIADO CORRECTAMENTE\033[0m  srv: /dr/solve_fk', file=sys.stderr)

    def solve_fk_callback(self, request, response):
        debug   = os.environ.get('DR_DEBUG')         == '1'
        verbose = os.environ.get('DR_DEBUG_VERBOSE') == '1'
        G, RED, R = '\033[92m', '\033[91m', '\033[0m'
        PREFIX = f'{G}--> [fk_node]{R}'

        q = list(request.q)
        if verbose:
            print(f'{PREFIX} request: q={[f"{v:.3f}" for v in q]}', file=sys.stderr)

        [success, pose] = self.solver.solve_fk(q[0], q[1], q[2], q[3], q[4], q[5])

        if success:
            response.success = True
            response.pose = pose
            response.message = 'Solución para FK encontrada'
            if verbose:
                print(f'{PREFIX} → pose={[f"{v:.3f}" for v in pose]}', file=sys.stderr)
        else:
            response.success = False
            response.pose = [0.0]*6
            response.message = 'Fallo al calcular FK'
            self.get_logger().warn(f'FK falló para q = {q}')
            if debug:
                print(f'{RED}--> [fk_node]{R} FK falló para q={[f"{v:.3f}" for v in q]}', file=sys.stderr)

        return response


def main(args=None):
    rclpy.init(args=args)
    node = FKNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()