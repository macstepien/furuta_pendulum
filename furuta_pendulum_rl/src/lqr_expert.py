import math
import numpy as np
from utils import limit_minus_pi_pi


class LQRExpert:
    def __init__(self) -> None:
        self.dtheta2_filtered_swingup_ = 0.0
        self.dtheta1_filtered_ = 0.0
        self.dtheta2_filtered_ = 0.0

        self.l2_ = 0.096608
        self.m2_ = 0.009801
        self.g_ = 9.80665
        self.u_max_ = 0.169
        self.alpha_swingup_ = 0.01
        self.alpha_ = 0.1
        self.K = np.array([0.0, 1.4075, -0.070089, 0.1584])
        self.lqr_transition_angle_ = 0.075
        self.torque_multiplier_ = 0.8

    def SwingUpControl(self, obs: np.array):
        theta2 = limit_minus_pi_pi(obs[1])

        E = 0.5 * self.m2_ * pow(self.l2_, 2) * math.pow(
            self.dtheta2_filtered_swingup_, 2
        ) + self.m2_ * self.g_ * self.l2_ * math.cos(theta2)

        E0 = self.m2_ * self.g_ * self.l2_
        x = (E0 - E) * self.dtheta2_filtered_swingup_ * (1 - math.cos(theta2))

        if x > 0.0:
            return -self.u_max_
        else:
            return self.u_max_

    def LQRControl(self, obs: np.array):
        obs[2] = self.dtheta1_filtered_
        obs[3] = self.dtheta2_filtered_
        return -self.K.dot(obs)

    def __call__(self, observations: np.array, states=None, episode_start=None):
        actions = []

        for obs in observations:
            self.dtheta2_filtered_swingup_ = (
                self.alpha_swingup_ * obs[3]
                + (1.0 - self.alpha_swingup_) * self.dtheta2_filtered_swingup_
            )
            self.dtheta1_filtered_ = (
                self.alpha_ * obs[2] + (1.0 - self.alpha_) * self.dtheta1_filtered_
            )
            self.dtheta2_filtered_ = (
                self.alpha_ * obs[3] + (1.0 - self.alpha_) * self.dtheta2_filtered_
            )

            if math.fabs(limit_minus_pi_pi(obs[1])) < self.lqr_transition_angle_:
                u = self.LQRControl(obs)
            else:
                u = self.SwingUpControl(obs)

            actions.append([u * self.torque_multiplier_])

        return np.array(actions), np.array([])

    def predict(self, observations: np.array, state, episode_start, deterministic):
        return self.__call__(observations, None, None)
