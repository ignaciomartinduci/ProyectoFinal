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


class InvKinematics:

    def __init__(self, params):

        # 1. Extraer parámetros DH

        self.qlim1_inf = params['q_min'][0]
        self.qlim1_sup = params['q_max'][0]
        self.qlim2_inf = params['q_min'][1]
        self.qlim2_sup = params['q_max'][1]
        self.qlim3_inf = params['q_min'][2]
        self.qlim3_sup = params['q_max'][2]
        self.qlim4_inf = params['q_min'][3]
        self.qlim4_sup = params['q_max'][3]
        self.qlim5_inf = params['q_min'][4]
        self.qlim5_sup = params['q_max'][4]
        self.qlim6_inf = params['q_min'][5]
        self.qlim6_sup = params['q_max'][5]

        self.d_1 = params['dh_d'][0]
        self.d_3 = params['dh_d'][2]
        self.d_5 = params['dh_d'][4]

        self.a_2 = params['dh_a'][1]
        self.a_3 = params['dh_a'][2]

        self.T_base = np.eye(4)
        self.T_tool = np.eye(4)

        self._init_transforms(params)
        
    def _init_transforms(self, params):

        # 2. Obtener T_base y T_tool a partir de los parámetros de entrada

        base_xyz = params['T_base_xyz']
        base_rpy = params['T_base_rpy']

        tool_xyz = params['T_tool_xyz']
        tool_rpy = params['T_tool_rpy']

        self.T_base = transl(base_xyz[0], base_xyz[1], base_xyz[2]) @ trotz(base_rpy[2]) @ troty(base_rpy[1]) @ trotz(base_rpy[2])
        self.T_tool = transl(tool_xyz[0], tool_xyz[1], tool_xyz[2]) @ trotz(tool_rpy[2]) @ troty(tool_rpy[1]) @ trotz(tool_rpy[2])

    def solve_ik(self, x, y, z, roll, pitch, yaw, force, q0):

        # 3. Inicialización de variables de cálculo

        geometric_solutions = []
        
        q_11 = 0.0
        q_12 = 0.0
        q_611 = 0.0
        q_612 = 0.0
        q_621 = 0.0
        q_622 = 0.0
        q_511 = 0.0
        q_512 = 0.0
        q_521 = 0.0
        q_522 = 0.0
        q_21 = 0.0
        q_22 = 0.0
        q_23 = 0.0
        q_24 = 0.0
        q_25 = 0.0
        q_26 = 0.0
        q_27 = 0.0
        q_28 = 0.0
        q_31 = 0.0
        q_32 = 0.0
        q_33 = 0.0
        q_34 = 0.0
        q_35 = 0.0
        q_36 = 0.0
        q_37 = 0.0
        q_38 = 0.0
        q_41 = 0.0
        q_42 = 0.0
        q_43 = 0.0
        q_44 = 0.0
        q_45 = 0.0
        q_46 = 0.0
        q_47 = 0.0
        q_48 = 0.0

        A = 0.0
        B = 0.0
        C = 0.0
        D = 0.0
        E = 0.0
        a = 0.0
        b = 0.0

        num = 0.0
        den = 0.0

        s1_valid = True
        s2_valid = True
        s3_valid = True
        s4_valid = True
        s5_valid = True
        s6_valid = True
        s7_valid = True
        s8_valid = True

        q_mejor = q0

        # 4. Formar MTH desde la base hasta la herramienta (T_06) y desacoplar base y herramienta

        T_06_full = transl(x, y, z) @ trotz(yaw) @ troty(pitch) @ trotx(roll)
        T_06 = inv_T(self.T_base) @ T_06_full @ inv_T(self.T_tool)

        # 5. Descomponer matriz en sus elementos

        r_11 = T_06[0, 0]
        r_12 = T_06[0, 1]
        r_13 = T_06[0, 2]
        r_21 = T_06[1, 0]
        r_22 = T_06[1, 1]
        r_23 = T_06[1, 2]
        r_31 = T_06[2, 0]
        r_32 = T_06[2, 1]
        r_33 = T_06[2, 2]
        
        x = T_06[0, 3]
        y = T_06[1, 3]
        z = T_06[2, 3]

        # 6. Cálculo de q1

        A = y
        B = -x
        C = -self.d_3

        discriminante = 4*B**2-4*(C-A)*(C+A)
        den = C-A

        if discriminante < 0 and den != 0:
            s1_valid = False
            s2_valid = False
            s3_valid = False
            s4_valid = False
        else:
            t_11 = (-2*B+np.sqrt(discriminante))/(2*(C-A))
            t_12 = (-2*B-np.sqrt(discriminante))/(2*(C-A))
        
            q_11 = 2*np.arctan(t_11)
            q_12 = 2*np.arctan(t_12)

            q_11 = np.mod(q_11-self.qlim1_inf, self.qlim1_sup-self.qlim1_inf) + self.qlim1_inf
            q_12 = np.mod(q_12-self.qlim1_inf, self.qlim1_sup-self.qlim1_inf) + self.qlim1_inf


        # 7. Cálculo de q6

        if s1_valid and s2_valid and s3_valid and s4_valid:

            A = -(r_22*np.cos(q_11)-r_12*np.sin(q_11))
            B = r_21*np.cos(q_11)-r_11*np.sin(q_11)

            q_611 = np.arctan2(A, B)
            q_611 = np.mod(q_611-self.qlim6_inf, self.qlim6_sup-self.qlim6_inf) + self.qlim6_inf
            q_612 = np.mod(q_611-self.qlim6_inf+np.pi, self.qlim6_sup-self.qlim6_inf) + self.qlim6_inf
            
        if s5_valid and s6_valid and s7_valid and s8_valid:

            A = -(r_22*np.cos(q_12)-r_12*np.sin(q_12))
            B = r_21*np.cos(q_12)-r_11*np.sin(q_12)

            q_621 = np.arctan2(A, B)
            q_621 = np.mod(q_621-self.qlim6_inf, self.qlim6_sup-self.qlim6_inf) + self.qlim6_inf
            q_622 = np.mod(q_621-self.qlim6_inf+np.pi, self.qlim6_sup-self.qlim6_inf) + self.qlim6_inf                 
        
        # 8. Cálculo de q5
        
        if s1_valid and s2_valid and s3_valid and s4_valid:

            num = np.cos(q_611)*(r_21*np.cos(q_11)-r_11*np.sin(q_11))-np.sin(q_611)*(r_22*np.cos(q_11)-r_12*np.sin(q_11))
            den = r_23*np.cos(q_11)-r_13*np.sin(q_11)

            q_511 = np.arctan2(num, den)
            q_511 = np.mod(q_511-self.qlim5_inf, self.qlim5_sup-self.qlim5_inf) + self.qlim5_inf


            num = np.cos(q_612)*(r_21*np.cos(q_11)-r_11*np.sin(q_11))-np.sin(q_612)*(r_22*np.cos(q_11)-r_12*np.sin(q_11))
            den = r_23*np.cos(q_11)-r_13*np.sin(q_11)

            q_512 = np.arctan2(num, den)
            q_512 = np.mod(q_512-self.qlim5_inf, self.qlim5_sup-self.qlim5_inf) + self.qlim5_inf

        if s5_valid and s6_valid and s7_valid and s8_valid:

            num = np.cos(q_621)*(r_21*np.cos(q_12)-r_11*np.sin(q_12))-np.sin(q_621)*(r_22*np.cos(q_12)-r_12*np.sin(q_12))
            den = r_23*np.cos(q_12)-r_13*np.sin(q_12)

            q_521 = np.arctan2(num, den)
            q_521 = np.mod(q_521-self.qlim5_inf, self.qlim5_sup-self.qlim5_inf) + self.qlim5_inf


            num = np.cos(q_622)*(r_21*np.cos(q_12)-r_11*np.sin(q_12))-np.sin(q_622)*(r_22*np.cos(q_12)-r_12*np.sin(q_12))
            den = r_23*np.cos(q_12)-r_13*np.sin(q_12)

            q_522 = np.arctan2(num, den)
            q_522 = np.mod(q_522-self.qlim5_inf, self.qlim5_sup-self.qlim5_inf) + self.qlim5_inf

        # 9. Cálculo de q2

        # sol 1 - 2

        if s1_valid and s2_valid:

            if abs(np.sin(q_511)) < 1e-10:
                s1_valid = False
                s2_valid = False

            else: 

                A = (r_13*np.cos(q_11)+r_23*np.sin(q_11))/(-np.sin(q_511))
                B = -r_33/(-np.sin(q_511))

                a = -self.d_5*A+z-self.d_1
                b = self.d_5*B-x*np.cos(q_11)-y*np.sin(q_11)

                C = self.a_2**2+a**2+b**2-self.a_3**2
                D = 2*self.a_2*b
                E = -2*self.a_2*a

                discriminante = 4*D**2-4*(C-E)*(C+E)

                if discriminante < 0:
                    s1_valid = False
                    s2_valid = False

                else:

                    t_21 = (-2*D+np.sqrt(discriminante))/(2*(C-E))
                    t_22 = (-2*D-np.sqrt(discriminante))/(2*(C-E))

                    q_21 = 2*np.arctan(t_21)
                    q_22 = 2*np.arctan(t_22)

                    q_21 = np.mod(q_21-self.qlim2_inf, self.qlim2_sup-self.qlim2_inf) + self.qlim2_inf
                    q_22 = np.mod(q_22-self.qlim2_inf, self.qlim2_sup-self.qlim2_inf) + self.qlim2_inf

        # sol 3 - 4

        if s3_valid and s4_valid:

            if abs(np.sin(q_512)) < 1e-10:
                s3_valid = False
                s4_valid = False
            else:
                A = (r_13*np.cos(q_11)+r_23*np.sin(q_11))/(-np.sin(q_512))
                B = -r_33/(-np.sin(q_512))


                a = -self.d_5*A+z-self.d_1
                b = self.d_5*B-x*np.cos(q_11)-y*np.sin(q_11)

                C = self.a_2**2+a**2+b**2-self.a_3**2
                D = 2*self.a_2*b
                E = -2*self.a_2*a

                discriminante = 4*D**2-4*(C-E)*(C+E)

                if discriminante < 0:
                    s3_valid = False
                    s4_valid = False

                else:

                    t_23 = (-2*D+np.sqrt(discriminante))/(2*(C-E))
                    t_24 = (-2*D-np.sqrt(discriminante))/(2*(C-E))

                    q_23 = 2*np.arctan(t_23)
                    q_24 = 2*np.arctan(t_24)

                    q_23 = np.mod(q_23-self.qlim2_inf, self.qlim2_sup-self.qlim2_inf) + self.qlim2_inf
                    q_24 = np.mod(q_24-self.qlim2_inf, self.qlim2_sup-self.qlim2_inf) + self.qlim2_inf

        # sol 5 - 6

        if s5_valid and s6_valid:

            if abs(np.sin(q_521)) < 1e-10:
                s5_valid = False
                s6_valid = False

            else:

                A = (r_13*np.cos(q_12)+r_23*np.sin(q_12))/(-np.sin(q_521))
                B = -r_33/(-np.sin(q_521))

                a = -self.d_5*A+z-self.d_1
                b = self.d_5*B-x*np.cos(q_12)-y*np.sin(q_12)

                C = self.a_2**2+a**2+b**2-self.a_3**2
                D = 2*self.a_2*b
                E = -2*self.a_2*a

                discriminante = 4*D**2-4*(C-E)*(C+E)

                if discriminante < 0:
                    s5_valid = False
                    s6_valid = False
                
                else:

                    t_25 = (-2*D+np.sqrt(discriminante))/(2*(C-E))
                    t_26 = (-2*D-np.sqrt(discriminante))/(2*(C-E))

                    q_25 = 2*np.arctan(t_25)
                    q_26 = 2*np.arctan(t_26)

                    q_25 = np.mod(q_25-self.qlim2_inf, self.qlim2_sup-self.qlim2_inf) + self.qlim2_inf
                    q_26 = np.mod(q_26-self.qlim2_inf, self.qlim2_sup-self.qlim2_inf) + self.qlim2_inf

        # sol 7 - 8

        if s7_valid and s8_valid:

            if abs(np.sin(q_522)) < 1e-10:
                s7_valid = False
                s8_valid = False

            else:

                A = (r_13*np.cos(q_12)+r_23*np.sin(q_12))/(-np.sin(q_522))
                B = -r_33/(-np.sin(q_522))

                a = -self.d_5*A+z-self.d_1
                b = self.d_5*B-x*np.cos(q_12)-y*np.sin(q_12)

                C = self.a_2**2+a**2+b**2-self.a_3**2
                D = 2*self.a_2*b
                E = -2*self.a_2*a

                discriminante = 4*D**2-4*(C-E)*(C+E)

                if discriminante < 0:
                    s7_valid = False
                    s8_valid = False

                else:

                    t_27 = (-2*D+np.sqrt(discriminante))/(2*(C-E))
                    t_28 = (-2*D-np.sqrt(discriminante))/(2*(C-E))

                    q_27 = 2*np.arctan(t_27)
                    q_28 = 2*np.arctan(t_28)

                    q_27 = np.mod(q_27-self.qlim2_inf, self.qlim2_sup-self.qlim2_inf) + self.qlim2_inf
                    q_28 = np.mod(q_28-self.qlim2_inf, self.qlim2_sup-self.qlim2_inf) + self.qlim2_inf

        # 10. Cálculo de q3

        # sol 1 - 2

        if s1_valid:

            if abs(np.sin(q_511)) < 1e-10:
                s1_valid = False
                s2_valid = False
            
            else:

                A = (r_13*np.cos(q_11)+r_23*np.sin(q_11))/(-np.sin(q_511))
                B = -r_33/(-np.sin(q_511))

                a = -self.d_5*A+z-self.d_1
                b = self.d_5*B-x*np.cos(q_11)-y*np.sin(q_11)

                num = -self.a_2*np.sin(q_21)-b
                den = -self.a_2*np.cos(q_21)+a

                q23 = np.arctan2(num, den)
                q_31 = q23 - q_21
                q_31 = np.mod(q_31-self.qlim3_inf, self.qlim3_sup-self.qlim3_inf) + self.qlim3_inf

        if s2_valid:

            if abs(np.sin(q_511)) < 1e-10:
                s1_valid = False
                s2_valid = False

            else:

                A = (r_13*np.cos(q_11)+r_23*np.sin(q_11))/(-np.sin(q_511))
                B = -r_33/(-np.sin(q_511))

                a = -self.d_5*A+z-self.d_1
                b = self.d_5*B-x*np.cos(q_11)-y*np.sin(q_11)

                num = -self.a_2*np.sin(q_22)-b
                den = -self.a_2*np.cos(q_22)+a

                q23 = np.arctan2(num, den)
                q_32 = q23 - q_22
                q_32 = np.mod(q_32-self.qlim3_inf, self.qlim3_sup-self.qlim3_inf) + self.qlim3_inf

        # sol 3 - 4

        if s3_valid:

            if abs(np.sin(q_512)) < 1e-10:
                s3_valid = False
                s4_valid = False

            else:

                A = (r_13*np.cos(q_11)+r_23*np.sin(q_11))/(-np.sin(q_512))
                B = -r_33/(-np.sin(q_512))

                a = -self.d_5*A+z-self.d_1
                b = self.d_5*B-x*np.cos(q_11)-y*np.sin(q_11)

                num = -self.a_2*np.sin(q_23)-b
                den = -self.a_2*np.cos(q_23)+a

                q23 = np.arctan2(num, den)
                q_33 = q23 - q_23
                q_33 = np.mod(q_33-self.qlim3_inf, self.qlim3_sup-self.qlim3_inf) + self.qlim3_inf

        if s4_valid:

            if abs(np.sin(q_512)) < 1e-10:
                s3_valid = False
                s4_valid = False

            else:

                A = (r_13*np.cos(q_11)+r_23*np.sin(q_11))/(-np.sin(q_512))
                B = -r_33/(-np.sin(q_512))

                a = -self.d_5*A+z-self.d_1
                b = self.d_5*B-x*np.cos(q_11)-y*np.sin(q_11)

                num = -self.a_2*np.sin(q_24)-b
                den = -self.a_2*np.cos(q_24)+a

                q23 = np.arctan2(num, den)
                q_34 = q23 - q_24
                q_34 = np.mod(q_34-self.qlim3_inf, self.qlim3_sup-self.qlim3_inf) + self.qlim3_inf

        # sol 5 - 6

        if s5_valid:

            if abs(np.sin(q_521)) < 1e-10:
                s5_valid = False
                s6_valid = False

            else:

                A = (r_13*np.cos(q_12)+r_23*np.sin(q_12))/(-np.sin(q_521))
                B = -r_33/(-np.sin(q_521))

                a = -self.d_5*A+z-self.d_1
                b = self.d_5*B-x*np.cos(q_12)-y*np.sin(q_12)

                num = -self.a_2*np.sin(q_25)-b
                den = -self.a_2*np.cos(q_25)+a

                q23 = np.arctan2(num, den)
                q_35 = q23 - q_25
                q_35 = np.mod(q_35-self.qlim3_inf, self.qlim3_sup-self.qlim3_inf) + self.qlim3_inf

        if s6_valid:

            if abs(np.sin(q_521)) < 1e-10:
                s5_valid = False
                s6_valid = False

            else:

                A = (r_13*np.cos(q_12)+r_23*np.sin(q_12))/(-np.sin(q_521))
                B = -r_33/(-np.sin(q_521))

                a = -self.d_5*A+z-self.d_1
                b = self.d_5*B-x*np.cos(q_12)-y*np.sin(q_12)

                num = -self.a_2*np.sin(q_26)-b
                den = -self.a_2*np.cos(q_26)+a

                q23 = np.arctan2(num, den)
                q_36 = q23 - q_26
                q_36 = np.mod(q_36-self.qlim3_inf, self.qlim3_sup-self.qlim3_inf) + self.qlim3_inf

        # sol 7 - 8

        if s7_valid:

            if abs(np.sin(q_522)) < 1e-10:
                s7_valid = False
                s8_valid = False

            else:

                A = (r_13*np.cos(q_12)+r_23*np.sin(q_12))/(-np.sin(q_522))
                B = -r_33/(-np.sin(q_522))

                a = -self.d_5*A+z-self.d_1
                b = self.d_5*B-x*np.cos(q_12)-y*np.sin(q_12)

                num = -self.a_2*np.sin(q_27)-b
                den = -self.a_2*np.cos(q_27)+a

                q23 = np.arctan2(num, den)
                q_37 = q23 - q_27
                q_37 = np.mod(q_37-self.qlim3_inf, self.qlim3_sup-self.qlim3_inf) + self.qlim3_inf

        if s8_valid:

            if abs(np.sin(q_522)) < 1e-10:
                s7_valid = False
                s8_valid = False

            else:

                A = (r_13*np.cos(q_12)+r_23*np.sin(q_12))/(-np.sin(q_522))
                B = -r_33/(-np.sin(q_522))

                a = -self.d_5*A+z-self.d_1
                b = self.d_5*B-x*np.cos(q_12)-y*np.sin(q_12)

                num = -self.a_2*np.sin(q_28)-b
                den = -self.a_2*np.cos(q_28)+a

                q23 = np.arctan2(num, den)
                q_38 = q23 - q_28
                q_38 = np.mod(q_38-self.qlim3_inf, self.qlim3_sup-self.qlim3_inf) + self.qlim3_inf

        # 11. Cálculo de q4

        # Sol 1
        if s1_valid:

            num = r_33
            den = -(r_13*np.cos(q_11)+r_23*np.sin(q_11))
            q234 = np.arctan2(num, den)
            q_41 = q234 - q_21 - q_31
            q_41 = np.mod(q_41-self.qlim4_inf, self.qlim4_sup-self.qlim4_inf) + self.qlim4_inf

        # Sol 2
        if s2_valid:
            num = r_33
            den = -(r_13*np.cos(q_11)+r_23*np.sin(q_11))
            q234 = np.arctan2(num, den)
            q_42 = q234 - q_22 - q_32
            q_42 = np.mod(q_42-self.qlim4_inf, self.qlim4_sup-self.qlim4_inf) + self.qlim4_inf

        # Sol 3
        if s3_valid:
            num = r_33
            den = -(r_13*np.cos(q_11)+r_23*np.sin(q_11))
            q234 = np.arctan2(num, den)
            q_43 = (q234 - q_23 - q_33) + np.pi
            q_43 = np.mod(q_43-self.qlim4_inf, self.qlim4_sup-self.qlim4_inf) + self.qlim4_inf


        # Sol 4
        if s4_valid:
            num = r_33
            den = -(r_13*np.cos(q_11)+r_23*np.sin(q_11))
            q234 = np.arctan2(num, den)
            q_44 = q234 - q_24 - q_34 + np.pi
            q_44 = np.mod(q_44-self.qlim4_inf, self.qlim4_sup-self.qlim4_inf) + self.qlim4_inf


        # Sol 5
        if s5_valid:
            num = r_33
            den = -(r_13*np.cos(q_12)+r_23*np.sin(q_12))
            q234 = np.arctan2(num, den)
            q_45 = q234 - q_25 - q_35
            q_45 = np.mod(q_45-self.qlim4_inf, self.qlim4_sup-self.qlim4_inf) + self.qlim4_inf

        # Sol 6
        if s6_valid:
            num = r_33
            den = -(r_13*np.cos(q_12)+r_23*np.sin(q_12))
            q234 = np.arctan2(num, den)
            q_46 = q234 - q_26 - q_36
            q_46 = np.mod(q_46-self.qlim4_inf, self.qlim4_sup-self.qlim4_inf) + self.qlim4_inf


        # Sol 7
        if s7_valid:
            num = r_33
            den = -(r_13*np.cos(q_12)+r_23*np.sin(q_12))
            q234 = np.arctan2(num, den)
            q_47 = q234 - q_27 - q_37 + np.pi
            q_47 = np.mod(q_47-self.qlim4_inf, self.qlim4_sup-self.qlim4_inf) + self.qlim4_inf

        # Sol 8
        if s8_valid:
            num = r_33
            den = -(r_13*np.cos(q_12)+r_23*np.sin(q_12))
            q234 = np.arctan2(num, den)
            q_48 = q234 - q_28 - q_38 + np.pi
            q_48 = np.mod(q_48-self.qlim4_inf, self.qlim4_sup-self.qlim4_inf) + self.qlim4_inf

        # 12. Ensamblar soluciones válidas

        if s1_valid:
            geometric_solutions.append([q_11, q_21, q_31, q_41, q_511, q_611])

        if s2_valid:
            geometric_solutions.append([q_11, q_22, q_32, q_42, q_511, q_611])

        if s3_valid:
            geometric_solutions.append([q_11, q_23, q_33, q_43, q_512, q_612])

        if s4_valid:
            geometric_solutions.append([q_11, q_24, q_34, q_44, q_512, q_612])

        if s5_valid:
            geometric_solutions.append([q_12, q_25, q_35, q_45, q_521, q_621])

        if s6_valid:
            geometric_solutions.append([q_12, q_26, q_36, q_46, q_521, q_621])

        if s7_valid:
            geometric_solutions.append([q_12, q_27, q_37, q_47, q_522, q_622])

        if s8_valid:
            geometric_solutions.append([q_12, q_28, q_38, q_48, q_522, q_622])

        # 13. Ramificación para selección de mejor solución

        if len(geometric_solutions) == 0:
            return [False, q0]

        if force:
            q_mejor = self.best_solution(geometric_solutions, q0)
            return [True, q_mejor]

        else:
            q_mejor = self.periodic_solutions(geometric_solutions, q0)
            return [True, q_mejor]
        
        
    
    def periodic_solutions(self, geometric_solutions, q0):

        # 14. Generación de soluciones periódicas para cada solución geométrica

        all_solutions = np.zeros((len(geometric_solutions)*64, 6))
        comb = np.array([[int(b) for b in format(i, '06b')] for i in range(64)])

        for i, sol in enumerate(geometric_solutions):

            q_aux = np.zeros(6)

            for j in range(6):

                if sol[j] > 0:
                    q_aux[j] = sol[j] - 2*np.pi

                else:
                    q_aux[j] = sol[j] + 2*np.pi
    
            for k in range(64):
                for l in range(6):
                    if comb[k, l] == 0:
                        all_solutions[i*64+k, l] = sol[l]
                    else:
                        all_solutions[i*64+k, l] = q_aux[l]

        q_mejor = self.best_solution(all_solutions, q0)

        return q_mejor

    def best_solution(self, solutions, q0):

        # 15. Selección de la mejor solución según criterio de cercanía a q0

        q0 = np.array(q0)
        q_mejor = q0
        min_distancia = 9999.999

        for i in range(len(solutions)):

            sol_actual = np.array(solutions[i])
            
            dq = sol_actual - q0
            costo_actual = np.sum(np.abs(dq))

            if costo_actual < min_distancia:
                min_distancia = costo_actual
                q_mejor = sol_actual

        return q_mejor.tolist()
    
    def wrap_to_pi(self, angles):
        angles = np.asarray(angles, dtype=np.float64).copy()
        q = (angles < -np.pi) | (np.pi < angles)
        angles[q] = np.mod(angles[q] + np.pi, 2 * np.pi) - np.pi
        return angles


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

    x = 0
    y = 0.2893
    z = 0.056
    roll = 0.1745
    pitch = 0.698
    yaw = 0.2622
    force=True
    q0 = [0,0,0,0,0,0]

    ik_solver = InvKinematics(params)
    [success,q] = ik_solver.solve_ik(x,y,z,roll,pitch,yaw,force,q0)

    print(success)
    print(q)

