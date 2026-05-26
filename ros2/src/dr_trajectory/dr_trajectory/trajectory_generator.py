import numpy as np


class TrajectoryGenerator:

    def __init__(self, params, ik_call, fk_call, logger=None):

        self.logger = logger

        self.q_max = np.array(params["q_max"])
        self.q_min = np.array(params["q_min"])
        self.qd_max = np.array(params["qd_max"])
        self.qdd_max = np.array(params["qdd_max"])
        self.ef_v_max = params["ef_v_max"]
        self.ef_a_max = params["ef_a_max"]
        self.ef_omega_max = params["ef_omega_max"]
        self.ef_alpha_max = params["ef_alpha_max"]
        self.ctrl_freq = params["ctrl_freq"]
        self.ctrl_t = 1 / self.ctrl_freq

        self.t_max = self.qd_max / self.qdd_max

        self.ik_call = ik_call
        self.fk_call = fk_call

    def generate(self, pose_obj, mode, speed, q0, c0):

        self.pose_obj = np.array(pose_obj)
        self.mode = mode
        self.speed = speed
        self.q0 = np.array(q0)
        self.c0 = np.array(c0)

        if speed == 1:
            self.speed_reducer = 4
        elif speed == 2:
            self.speed_reducer = 2
        elif speed == 3:
            self.speed_reducer = 1

        if self.mode == 0:
            [success, q_traj, points] = self.articular_interpolation()

        elif self.mode == 1:
            [success, q_traj, points] = self.cartesian_interpolation()

        else:
            return [False, [], 0]

        return [success, q_traj, points]

    def articular_interpolation(self, T_override=None, q_obj=None):

        if q_obj is None:
            success, self.q_obj = self.ik_call(self.pose_obj)

            if not success:
                return [False, [], 0]

            self.q_obj = np.array(self.q_obj)

        else:
            self.q_obj = q_obj

        s, sd, sdd, T, N = self.compute_s(0, T_override=T_override)

        delta_q = self.q_obj - self.q0

        q_traj = self.q0 + s[:, np.newaxis] * delta_q
        qd_traj = sd[:, np.newaxis] * delta_q
        qdd_traj = sdd[:, np.newaxis] * delta_q

        c_traj = np.zeros((N, 6))

        for i, c in enumerate(q_traj):
            success, c_val = self.fk_call(q_traj[i, :])

            if not success:
                return [False, [], 0]
            else:
                c_traj[i, :] = c_val

        cd_traj = np.gradient(c_traj, self.ctrl_t, axis=0)
        cdd_traj = np.gradient(cd_traj, self.ctrl_t, axis=0)

        # Verificación solo lineal de cinemática del EF

        cv_scale = np.max(np.abs(cd_traj[:, 0:3]) / self.ef_v_max, axis=0)
        ca_scale = np.max(np.abs(cdd_traj[:, 0:3]) / self.ef_a_max, axis=0)
        # comega_scale = np.max(np.abs(cd_traj[:,3:6]) / self.ef_omega_max, axis=0)
        # calpha_scale = np.max(np.abs(cdd_traj[:,3:6]) / self.ef_alpha_max, axis=0)

        scale = max(
            np.max(cv_scale), np.sqrt(np.max(ca_scale))
        )  # , np.max(comega_scale), np.sqrt(np.max(calpha_scale)))

        if scale > 1.01:
            T = T * scale
            if self.logger:
                self.logger.warn(
                    f"Velocidad articular excedida (scale={scale:.2f}), reescalando T={T:.3f}s."
                )
            return self.articular_interpolation(T_override=T, q_obj=self.q_obj)

        q_traj_flat = [float(x) for x in q_traj.flatten()]
        return [True, q_traj_flat, N]

    def cartesian_interpolation(self, T_override=None, q0_original=None):

        if q0_original is None:
            q0_original = self.q0.copy()
        else:
            self.q0 = q0_original.copy()

        s, sd, sdd, T, N = self.compute_s(1, T_override=T_override)

        delta_pose = self.pose_obj - self.c0

        c_traj = self.c0 + s[:, np.newaxis] * delta_pose
        cv_traj = sd[:, np.newaxis] * delta_pose
        ca_traj = sdd[:, np.newaxis] * delta_pose

        q_traj = np.zeros((N, 6))

        for i, c in enumerate(c_traj):
            success, q = self.ik_call(c)

            if not success:
                return [False, self.q0.tolist(), 0]
            else:
                q_traj[i, :] = q
                self.q0 = np.array(q)

        qd_traj = np.gradient(q_traj, self.ctrl_t, axis=0)
        qdd_traj = np.gradient(qd_traj, self.ctrl_t, axis=0)

        qd_scale = np.max(np.abs(qd_traj) / self.qd_max, axis=0)
        qdd_scale = np.max(np.abs(qdd_traj) / self.qdd_max, axis=0)

        scale = max(np.max(qd_scale), np.sqrt(np.max(qdd_scale)))

        if scale > 1.01:
            T = T * scale
            if self.logger:
                self.logger.warn(
                    f"Velocidad cartesiana excedida (scale={scale:.2f}), reescalando T={T:.3f}s."
                )
            return self.cartesian_interpolation(T_override=T, q0_original=q0_original)

        q_traj_flat = [float(x) for x in q_traj.flatten()]
        return [True, q_traj_flat, N]

    # HELPERS

    def compute_s(self, mode, T_override=None):

        if T_override is None and mode == 0:

            T_qd = 15 / 8 * np.abs(self.q_obj - self.q0) / self.qd_max
            T_qdd = np.sqrt(
                10 / np.sqrt(3) * np.abs(self.q_obj - self.q0) / self.qdd_max
            )

            T = np.max(np.maximum(T_qd, T_qdd)) * self.speed_reducer

        elif T_override is None and mode == 1:
            linear_distance = np.linalg.norm(self.pose_obj[0:3] - self.c0[0:3])
            angular_distance = np.linalg.norm(self.pose_obj[3:6] - self.c0[3:6])

            T_v = 15 / 8 * linear_distance / self.ef_v_max
            T_a = np.sqrt(10 / np.sqrt(3) * linear_distance / self.ef_a_max)
            T_omega = 15 / 8 * angular_distance / self.ef_omega_max
            T_alpha = np.sqrt(10 / np.sqrt(3) * angular_distance / self.ef_alpha_max)
            T = max(T_v, T_a, T_omega, T_alpha) * self.speed_reducer

        else:
            T = T_override

        N = max(int(np.ceil(T * self.ctrl_freq)), 2)
        T = N * self.ctrl_t
        t = np.linspace(0, T, N)  # vector de tiempo
        tau = t / T  # vector de tiempo normalizado

        s = 10 * tau**3 - 15 * tau**4 + 6 * tau**5
        sd = (1 / T) * (30 * tau**2 - 60 * tau**3 + 30 * tau**4)
        sdd = (1 / T**2) * (60 * tau - 180 * tau**2 + 120 * tau**3)

        return s, sd, sdd, T, N
