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
            # "single_rgb_array",
            # "single_depth_array",
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

        # self.action_space = spaces.Discrete(3)
        # self.frame_skip = 1
        self.upward_count = 0

    def step(self, action):
        self.do_simulation(action, self.frame_skip)
        ob = self._get_obs()
        reward = self.calculate_reward(ob, action)

        # Agent is much more robust when translating it to other
        # environments without this additional rewards
        # if np.abs(ob[1]) < 0.1:
        #     self.upward_count += 1
        #     reward += 10.0
        # else:
        #     self.upward_count = 0

        terminated = bool(not np.isfinite(ob).all())
        # if self.upward_count > 40:
        #     terminated = True

        if self.render_mode == "human":
            self.render()
        return ob, reward, terminated, False, {}

    def reset_model(self):
        qpos = self.init_qpos
        # set downward position
        qpos[1] = -3.14159265
        qvel = self.init_qvel
        self.set_state(qpos, qvel)
        return self._get_obs()

    def limit_minus_pi_pi(self, angle):
        angle = math.fmod(angle, 2 * math.pi)
        if angle > math.pi:
            angle = 2 * math.pi - angle
        elif angle < -math.pi:
            angle = 2 * math.pi + angle
        return angle

    def _get_obs(self):
        obs = np.concatenate([self.data.qpos, self.data.qvel]).ravel()
        obs[0] = self.limit_minus_pi_pi(obs[0])
        obs[1] = self.limit_minus_pi_pi(obs[1])
        return obs

    def calculate_reward(self, obs: np.array, a: np.array):
        theta1_weight = 0.0
        theta2_weight = 10.0
        dtheta1_weight = 1.0
        dtheta2_weight = 1.0
        u_weight = 1.0

        Q = np.array(
            [
                [theta1_weight, 0, 0, 0],
                [0, theta2_weight, 0, 0],
                [0, 0, dtheta1_weight, 0],
                [0, 0, 0, dtheta2_weight],
            ]
        )
        R = np.array([u_weight])

        reward = -(obs.transpose() @ Q @ obs + a * R * a)[0]
        return reward

    def viewer_setup(self):
        assert self.viewer is not None
        v = self.viewer
        v.cam.trackbodyid = 0
        v.cam.distance = self.model.stat.extent
