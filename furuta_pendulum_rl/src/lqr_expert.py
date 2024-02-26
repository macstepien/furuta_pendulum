import math
import numpy as np
from utils import limit_minus_pi_pi


class LQRExpert:
    def __init__(self, n_envs=1) -> None:
        self._dtheta2_filtered = [0.0] * n_envs
        self._dtheta2_alpha = 0.01

        self.g_ = 9.80665

        self.m1_ = 0.040466
        self.m2_ = 0.009801

        self.l1_ = 0.05415
        self.l2_ = 0.096608

        self.L1_ = 0.093525
        self.L2_ = 0.129

        self.J1_ = 5.96237099e-04
        self.J2_ = 0.000014790595

        self.J1_hat_ = self.J1_ + self.m1_ * self.l1_ * self.l1_
        self.J2_hat_ = self.J2_ + self.m2_ * self.l2_ * self.l2_

        self.u_max_ = 0.4

        self.K = np.array([0.0, 1.4075, -0.070089, 0.1584])
        self.lqr_transition_angle_ = 0.4
        self.torque_multiplier_ = 1.0

        self._max_velocity_joint0 = 22.0
        self._max_velocity_joint1 = 50.0

    def SwingUpControl(self, obs: np.array, dtheta2):
    # def SwingUpControl(self, obs: np.array):
        theta2 = limit_minus_pi_pi(obs[1])
        # dtheta1 = obs[2]
        # dtheta2 = obs[3]

        E = 0.5 * self.m2_ * math.pow(self.l2_, 2) * math.pow(
            dtheta2, 2
        ) + self.m2_ * self.g_ * self.l2_ * (math.cos(theta2) + 1)

        # Ek1 = 0.5 * math.pow(dtheta1, 2) * (self.J1_hat_)
        # Ep2 = self.m2_ * self.g_ * self.l2_ * (math.cos(theta2) + 1)
        # Ek2 = (
        #     0.5
        #     * math.pow(dtheta1, 2)
        #     * (
        #         self.m2_ * math.pow(self.L2_, 2)
        #         + self.J2_hat_ * math.pow(math.sin(theta2 + math.pi), 2)
        #     )
        #     + 0.5 * math.pow(dtheta2, 2) * self.J2_hat_
        #     + self.m2_
        #     * self.L1_
        #     * self.l2_
        #     * math.cos(theta2 + math.pi)
        #     * dtheta1
        #     * dtheta2
        # )

        # E = Ek1 + Ep2 + Ek2
        # E = Ep2 + Ek2
        # E = Ek2

        E0 = 2.0 * self.m2_ * self.g_ * self.l2_
        x = (E0 - E) * dtheta2 * math.cos(theta2)

        if x < 0.0:
            return -self.u_max_
        else:
            return self.u_max_

    def LQRControl(self, obs: np.array):
        return -self.K.dot(obs)

    def __call__(self, observations: np.array, states=None, episode_start=None):
        actions = []

        for i in range(len(observations)):
            obs = [
                math.atan2(observations[i][0], observations[i][1]),
                math.atan2(observations[i][2], observations[i][3]),
                observations[i][4] * self._max_velocity_joint0,
                observations[i][5] * self._max_velocity_joint1,
            ]

            self._dtheta2_filtered[i] = (obs[3] * self._dtheta2_alpha) + (
                self._dtheta2_filtered[i] * (1.0 - self._dtheta2_alpha)
            )

            if math.fabs(limit_minus_pi_pi(obs[1])) < self.lqr_transition_angle_:
                u = self.LQRControl(obs)
            else:
                u = self.SwingUpControl(obs, self._dtheta2_filtered[i])
                # u = self.SwingUpControl(obs)

            u = np.clip(u, -0.4, 0.4)

            actions.append([u * self.torque_multiplier_])

        return np.array(actions), np.array([])

    def predict(self, observations: np.array, state, episode_start, deterministic):
        return self.__call__(observations, None, None)
