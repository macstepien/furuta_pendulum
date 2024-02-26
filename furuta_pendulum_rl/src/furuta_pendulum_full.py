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

        state_size = 6
        number_of_stacked_states = 1
        # action_size = 1
        action_size = 0

        utils.EzPickle.__init__(self)
        observation_space = Box(
            low=-1.0,
            high=1.0,
            shape=(state_size * number_of_stacked_states + action_size,),
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

        self._angle_threshold = 0.2
        self._dangle_threshold = 0.5

        theta1_weight = 0.0
        theta2_weight = 1.0
        dtheta1_weight = 0.1
        dtheta2_weight = 1.0

        self._desired_obs_values = [0.0, 1.0, 0.0, 1.0, 0.0, 0.0]
        self._obs_weights = [
            theta1_weight,
            theta1_weight,
            theta2_weight,
            theta2_weight,
            dtheta1_weight,
            dtheta2_weight,
        ]

        self._action_weight = 0.05

        # self._daction_weight = 25.0
        # self._last_action = [0.0]

        # self._velocity_noise = 0.0

        self._init_pos_high = math.pi
        self._init_pos_low = -math.pi
        self._init_vel_high = 3.0
        self._init_vel_low = -3.0

        self._stabilized_count = 0
        self._stabilized_count_threshold = 1000

        self._last_observation = None

    def step(self, action):
        self.do_simulation(action, self.frame_skip)
        self.bound_velocities()
        ob = self._get_current_obs()
        reward = self.calculate_reward(ob, action)

        terminated = bool(not np.isfinite(ob).all())

        # theta1_diff = self._angle_threshold - np.abs(limit_minus_pi_pi(self.data.qpos[1]))
        # dtheta1_diff = self._dangle_threshold - np.abs(self.data.qvel[0])
        # dtheta2_diff = self._dangle_threshold - np.abs(self.data.qvel[1])

        if (
            np.abs(limit_minus_pi_pi(self.data.qpos[1])) < self._angle_threshold
            and np.abs(self.data.qvel[0]) < self._dangle_threshold
            and np.abs(self.data.qvel[1]) < self._dangle_threshold
        ):
            # reward = 1000.0
            # terminated = True
            reward += 0.1 * self._stabilized_count
            # reward += 0.1
            self._stabilized_count += 1
            # if self._stabilized_count > self._stabilized_count_threshold:
            #     terminated = True
            #     reward = 1000.0
        else:
            self._stabilized_count = 0

        if self.render_mode == "human":
            self.render()

        full_obs = self._get_obs()

        self._last_action = action
        self._last_observation = ob

        return full_obs, reward, terminated, False, {}

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
        self._last_observation = self._get_current_obs()
        self._last_action = [0.0]
        return self._get_obs()

    def bound_velocities(self):
        self.data.qvel[0] = np.clip(
            self.data.qvel[0], -self._max_velocity_joint0, self._max_velocity_joint0
        )
        self.data.qvel[1] = np.clip(
            self.data.qvel[1], -self._max_velocity_joint1, self._max_velocity_joint1
        )

    def _get_current_obs(self):
        # obs = np.concatenate([self.data.qpos, self.data.qvel]).ravel()

        # obs[0] = limit_minus_pi_pi(obs[0])
        # obs[1] = limit_minus_pi_pi(obs[1])

        # sin and cos instead of limit to get rid of discontinuities
        # scale angular velocities so that they won't dominate

        # vel0_with_noise = (
        #     self.data.qvel[0]
        #     + self.np_random.standard_normal() * self._velocity_noise
        # )
        # vel1_with_noise = (
        #     self.data.qvel[1]
        #       + self.np_random.standard_normal() * self._velocity_noise
        # )

        obs = np.array(
            [
                np.sin(self.data.qpos[0]),
                np.cos(self.data.qpos[0]),
                np.sin(self.data.qpos[1]),
                np.cos(self.data.qpos[1]),
                self.data.qvel[0] / self._max_velocity_joint0,
                self.data.qvel[1] / self._max_velocity_joint1,
                # vel0_with_noise / self._max_velocity_joint0,
                # vel1_with_noise / self._max_velocity_joint1,
            ]
        )

        return obs

    def _get_obs(self):
        # return np.concatenate(
        #     (self._get_current_obs(), self._last_observation, self._last_action)
        # )
        # return np.concatenate((self._get_current_obs(), self._last_observation))
        return self._get_current_obs()

    def calculate_reward(self, obs: np.array, a: np.array):
        observation_reward = np.sum(
            [
                -weight * np.power(np.abs(desired_value - observation_value), 2.0)
                # -weight * np.abs(desired_value - observation_value)
                for (observation_value, desired_value, weight) in zip(
                    obs, self._desired_obs_values, self._obs_weights
                )
            ]
        )

        # theta2_weight = 2.0
        # dtheta1_weight = 0.5
        # dtheta2_weight = 0.5
        # action_weight = 1.0

        # observation_reward = -np.sum(
        #     [
        #         theta2_weight * np.abs(limit_minus_pi_pi(self.data.qpos[1])),
        #         dtheta1_weight * np.abs(self.data.qvel[0]),
        #         dtheta2_weight * np.abs(self.data.qvel[1]),
        #     ]
        # )

        # action_reward = -action_weight * np.abs(a[0])
        action_reward = -self._action_weight * np.power(a[0], 2)

        # daction_reward = -self._daction_weight * np.power(
        #     (a[0] - self._last_action[0]), 2
        # )
        # daction_reward = -self._daction_weight * np.abs((a[0] - self._last_action[0]))
        # return observation_reward + action_reward + daction_reward
        return observation_reward + action_reward

    def viewer_setup(self):
        assert self.viewer is not None
        v = self.viewer
        v.cam.trackbodyid = 0
        v.cam.distance = self.model.stat.extent
