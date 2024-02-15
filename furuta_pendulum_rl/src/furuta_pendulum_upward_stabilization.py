import os

import numpy as np

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
        "render_fps": 500,
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
            2,
            observation_space=observation_space,
            **kwargs,
        )
        self._init_pos_high = 0.01
        self._init_pos_low = -0.01
        self._init_vel_high = 0.01
        self._init_vel_low = -0.01

        self._angle_threshold = 0.2
        self._reward = 1.0

    def step(self, action):
        self.do_simulation(action, self.frame_skip)
        ob = self._get_obs()
        terminated = bool(
            not np.isfinite(ob).all() or (np.abs(ob[1]) > self._angle_threshold)
        )
        if self.render_mode == "human":
            self.render()
        return ob, self._reward, terminated, False, {}

    def reset_model(self):
        qpos = self.init_qpos + self.np_random.uniform(
            size=self.model.nq, low=self._init_pos_low, high=self._init_pos_high
        )
        qvel = self.init_qvel + self.np_random.uniform(
            size=self.model.nv, low=self._init_vel_low, high=self._init_vel_high
        )
        self.set_state(qpos, qvel)
        return self._get_obs()

    def _get_obs(self):
        return np.concatenate([self.data.qpos, self.data.qvel]).ravel()

    def viewer_setup(self):
        assert self.viewer is not None
        v = self.viewer
        v.cam.trackbodyid = 0
        v.cam.distance = self.model.stat.extent
