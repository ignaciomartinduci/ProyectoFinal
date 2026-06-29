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

import numpy as np
from dr_kinematics.transforms import *

class ForwKinematics:

    def __init__(self, params):

        # 1. Extraer parámetros DH

        self.qlim_mins = params['q_min'] 
        self.qlim_maxs = params['q_max']

        self.dh_theta_offset = params['dh_theta_offset']
        self.d = params['dh_d']
        self.a = params['dh_a']
        self.alpha = params['dh_alpha']

        self.base_xyz = params['T_base_xyz']
        self.base_rpy = params['T_base_rpy']

        self.tool_xyz = params['T_tool_xyz']
        self.tool_rpy = params['T_tool_rpy']

        self.T_base = transl(self.base_xyz[0], self.base_xyz[1], self.base_xyz[2]) @ trotx(self.base_rpy[0]) @ troty(self.base_rpy[1]) @ trotz(self.base_rpy[2])
        self.T_tool = transl(self.tool_xyz[0], self.tool_xyz[1], self.tool_xyz[2]) @ trotx(self.tool_rpy[0]) @ troty(self.tool_rpy[1]) @ trotz(self.tool_rpy[2])

    def solve_fk(self, q1, q2, q3, q4, q5, q6):

        q = [q1, q2, q3, q4, q5, q6]

        success = self.check_limits(q)

        T_06 = self.T_base @ np.eye(4)

        for i in range(len(q)):
            T_i = trotz(q[i]+self.dh_theta_offset[i]) @ transl(0, 0, self.d[i]) @ transl(self.a[i], 0, 0) @ trotx(self.alpha[i])
            T_06 = T_06 @ T_i

        T_06 = T_06 @ self.T_tool

        [roll, pitch, yaw] = rot_to_rpy(T_06[0:3, 0:3])

        x = T_06[0,3]
        y = T_06[1,3]
        z = T_06[2,3]

        pose = [x, y, z, roll, pitch, yaw]
        pose = [float(x) for x in pose]

        return [success, pose]
    
    def check_limits(self, q):
        
        for i in range(len(q)):

            if abs(q[i]) > self.qlim_maxs[i]:
                return False
            
        return True

    
# TEST
if __name__ == '__main__':

    # Parámetros del robot (extraídos de robot_params.yml)
    params = {
        'q_min':           [-6.28318, -6.28318, -6.28318, -6.28318, -6.28318, -1e9],
        'q_max':           [ 6.28318,  6.28318,  6.28318,  6.28318,  6.28318,  1e9],
        'dh_theta_offset': [0.0, -1.5708, 0.0, 1.5708, 0.0, 0.0],
        'dh_d':            [0.25, 0.0, 0.295, 0.0, 0.1, 0.0],
        'dh_a':            [0.0, 0.25, 0.25, 0.0, 0.0, 0.0],
        'dh_alpha':        [-1.5708, 0.0, 0.0, 1.5708, -1.5708, 0.0],
        'T_base_xyz':      [0.0, 0.0, 0.0],
        'T_base_rpy':      [0.0, 0.0, 0.0],
        'T_tool_xyz':      [0.0, 0.0, 0.190],
        'T_tool_rpy':      [0.0, 0.0, 0.0],
    }

    q1 = 0.7
    q2 = -0.7
    q3 = 0.7
    q4 = -0.7
    q5 = 0.7
    q6 = -0.7

    fk_solver = ForwKinematics(params)
    [success,pose] = fk_solver.solve_fk(q1, q2, q3, q4, q5, q6)
    print(success)
    print(pose)




    