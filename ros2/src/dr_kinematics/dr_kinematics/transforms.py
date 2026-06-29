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

def transl(X, Y, Z):
    T = np.eye(4)
    T[0:3, 3] = [X, Y, Z]
    return T

def trotx(THETA):
    T = np.eye(4)
    c = np.cos(THETA)
    s = np.sin(THETA)
    T[0:3, 0:3] = [[1, 0,  0],
                    [0, c, -s],
                    [0, s,  c]]
    return T

def troty(THETA):
    T = np.eye(4)
    c = np.cos(THETA)
    s = np.sin(THETA)
    T[0:3, 0:3] = [[ c, 0, s],
                    [ 0, 1, 0],
                    [-s, 0, c]]
    return T

def trotz(THETA):
    T = np.eye(4)
    c = np.cos(THETA)
    s = np.sin(THETA)
    T[0:3, 0:3] = [[c, -s, 0],
                    [s,  c, 0],
                    [0,  0, 1]]
    return T

def inv_T(T):
    R = T[:3, :3]
    p = T[:3, 3]
    
    T_inv = np.eye(4)
    T_inv[:3, :3] = R.T          # transpuesta de la rotacion
    T_inv[:3, 3]  = -R.T @ p     # -R^T * p
    return T_inv

def rot_to_rpy(R):
    pitch = np.arctan2(-R[2,0], np.sqrt(R[2,1]**2 + R[2,2]**2))
    roll  = np.arctan2(R[2,1], R[2,2])
    yaw   = np.arctan2(R[1,0], R[0,0])
    return np.array([roll, pitch, yaw])