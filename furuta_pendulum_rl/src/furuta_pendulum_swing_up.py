import os

import numpy as np
import math

from gym import utils
from gym.envs.mujoco import MujocoEnv
from gym.spaces import Box

from ament_index_python.packages import get_package_share_directory


# Based on inverted pendulum example world from gym
class FurutaPendulumEnv(MujocoEnv, utils.EzPickle):
    metadata = {
        "render_modes": [
            "human",
            "rgb_array",
            "depth_array",
        ],
        "render_fps": 25,
    }

    def __init__(self, **kwargs):
        utils.EzPickle.__init__(self)
        observation_space = Box(low=-np.inf, high=np.inf, shape=(4,), dtype=np.float64)

        MujocoEnv.__init__(
            self,
            os.path.join(
                get_package_share_directory("furuta_pendulum_rl"),
                "model",
                "furuta_pendulum.xml",
            ),
            40,
            observation_space=observation_space,
            **kwargs,
        )

        self._angle_threshold = 0.2

        # Same as in LQR
        theta1_weight = 0.0
        theta2_weight = 10.0
        dtheta1_weight = 1.0
        dtheta2_weight = 1.0
        u_weight = 1.0

        self._Q = np.array(
            [
                [theta1_weight, 0, 0, 0],
                [0, theta2_weight, 0, 0],
                [0, 0, dtheta1_weight, 0],
                [0, 0, 0, dtheta2_weight],
            ]
        )
        self._R = np.array([u_weight])

    def step(self, action):
        self.do_simulation(action, self.frame_skip)
        ob = self._get_obs()
        reward = self.calculate_reward(ob, action)
        terminated = bool(
            not np.isfinite(ob).all() or (np.abs(ob[1]) < self._angle_threshold)
        )

        if self.render_mode == "human":
            self.render()
        return ob, reward, terminated, False, {}

    def reset_model(self):
        qpos = self.init_qpos
        # set downward position
        qpos[1] = -3.14159
        qvel = self.init_qvel
        self.set_state(qpos, qvel)
        return self._get_obs()

    def _get_obs(self):
        obs = np.concatenate([self.data.qpos, self.data.qvel]).ravel()
        obs[1] = self.limit_minus_pi_pi(obs[1])
        return obs

    def calculate_reward(self, obs: np.array, a: np.array):
        return -(obs.transpose() @ self._Q @ obs + a * self._R * a)[0]

    def viewer_setup(self):
        assert self.viewer is not None
        v = self.viewer
        v.cam.trackbodyid = 0
        v.cam.distance = self.model.stat.extent

    def limit_minus_pi_pi(self, angle):
        angle = math.fmod(angle, 2 * math.pi)
        if angle > math.pi:
            angle = 2 * math.pi - angle
        elif angle < -math.pi:
            angle = 2 * math.pi + angle
        return angle