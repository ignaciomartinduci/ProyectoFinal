import numpy as np


def transl(X, Y, Z):
    T = np.eye(4)
    T[0:3, 3] = [X, Y, Z]
    return T


def trotx(THETA):
    T = np.eye(4)
    c = np.cos(THETA)
    s = np.sin(THETA)
    T[0:3, 0:3] = [[1, 0, 0], [0, c, -s], [0, s, c]]
    return T


def troty(THETA):
    T = np.eye(4)
    c = np.cos(THETA)
    s = np.sin(THETA)
    T[0:3, 0:3] = [[c, 0, s], [0, 1, 0], [-s, 0, c]]
    return T


def trotz(THETA):
    T = np.eye(4)
    c = np.cos(THETA)
    s = np.sin(THETA)
    T[0:3, 0:3] = [[c, -s, 0], [s, c, 0], [0, 0, 1]]
    return T


def inv_T(T):
    R = T[:3, :3]
    p = T[:3, 3]

    T_inv = np.eye(4)
    T_inv[:3, :3] = R.T  # transpuesta de la rotacion
    T_inv[:3, 3] = -R.T @ p  # -R^T * p
    return T_inv


def rot_to_rpy(R):
    pitch = np.arctan2(-R[2, 0], np.sqrt(R[2, 1] ** 2 + R[2, 2] ** 2))
    roll = np.arctan2(R[2, 1], R[2, 2])
    yaw = np.arctan2(R[1, 0], R[0, 0])
    return np.array([roll, pitch, yaw])
