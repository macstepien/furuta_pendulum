import os

import numpy as np
import math

from gym import utils
from gym.envs.mujoco import MujocoEnv
from gym.spaces import Box

from ament_index_python.packages import get_package_share_directory

from utils import limit_minus_pi_pi


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
        self._max_velocity_joint0 = 22.0
        self._max_velocity_joint1 = 50.0

        utils.EzPickle.__init__(self)
        observation_space = Box(
            low=np.array(
                [
                    -1.0,
                    -1.0,
                    -1.0,
                    -1.0,
                    -self._max_velocity_joint0,
                    -self._max_velocity_joint1,
                ]
            ),
            high=np.array(
                [
                    1.0,
                    1.0,
                    1.0,
                    1.0,
                    self._max_velocity_joint0,
                    self._max_velocity_joint1,
                ]
            ),
            shape=(6,),
            dtype=np.float64,
        )

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

        self._angle_threshold = 0.02
        self._dangle_threshold = 0.02

        theta1_weight = 0.0
        theta2_weight = 10.0
        dtheta1_weight = 1.0
        dtheta2_weight = 3.0

        self._desired_obs_values = [0.0, 1.0, 0.0, 1.0, 0.0, 0.0]
        self._obs_weights = [
            theta1_weight,
            theta1_weight,
            theta2_weight,
            theta2_weight,
            dtheta1_weight,
            dtheta2_weight,
        ]

        self._action_weight = 0.0

        self._init_pos_high = 3.0
        self._init_pos_low = -3.0
        self._init_vel_high = 3.0
        self._init_vel_low = -3.0

    def step(self, action):
        self.do_simulation(action, self.frame_skip)
        self.bound_velocities()
        ob = self._get_obs()
        reward = self.calculate_reward(ob, action)

        terminated = bool(not np.isfinite(ob).all())

        if (
            np.abs(limit_minus_pi_pi(self.data.qpos[1])) < self._angle_threshold
            and np.abs(self.data.qvel[0]) < self._dangle_threshold
            and np.abs(self.data.qvel[1]) < self._dangle_threshold
        ):
            # reward = 1000.0
            # terminated = True
            reward += 1.0

        if self.render_mode == "human":
            self.render()
        return ob, reward, terminated, False, {}

    def reset_model(self):
        qpos = self.init_qpos + self.np_random.uniform(
            size=self.model.nq, low=self._init_pos_low, high=self._init_pos_high
        )
        qpos[1] -= math.pi
        qvel = self.init_qvel + self.np_random.uniform(
            size=self.model.nv, low=self._init_vel_low, high=self._init_vel_high
        )

        # qpos = self.init_qpos
        # qpos[1] = -math.pi
        # qvel = self.init_qvel

        self.set_state(qpos, qvel)
        return self._get_obs()

    def bound_velocities(self):
        self.data.qvel[0] = np.clip(
            self.data.qvel[0], -self._max_velocity_joint0, self._max_velocity_joint0
        )
        self.data.qvel[1] = np.clip(
            self.data.qvel[1], -self._max_velocity_joint1, self._max_velocity_joint1
        )

    def _get_obs(self):
        # obs = np.concatenate([self.data.qpos, self.data.qvel]).ravel()

        # obs[0] = limit_minus_pi_pi(obs[0])
        # obs[1] = limit_minus_pi_pi(obs[1])

        # sin and cos instead of limit to get rid of discontinuities
        # scale angular velocities so that they won't dominate

        obs = np.array(
            [
                np.sin(self.data.qpos[0]),
                np.cos(self.data.qpos[0]),
                np.sin(self.data.qpos[1]),
                np.cos(self.data.qpos[1]),
                self.data.qvel[0] / self._max_velocity_joint0,
                self.data.qvel[1] / self._max_velocity_joint1,
            ]
        )

        return obs

    def calculate_reward(self, obs: np.array, a: np.array):
        observation_reward = np.sum(
            [
                -weight * np.power((desired_value - observation_value), 2)
                for (observation_value, desired_value, weight) in zip(
                    obs, self._desired_obs_values, self._obs_weights
                )
            ]
        )

        action_reward = -self._action_weight * np.power(a[0], 2)

        return observation_reward + action_reward

    def viewer_setup(self):
        assert self.viewer is not None
        v = self.viewer
        v.cam.trackbodyid = 0
        v.cam.distance = self.model.stat.extent
