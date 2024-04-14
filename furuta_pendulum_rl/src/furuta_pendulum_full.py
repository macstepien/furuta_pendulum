import os

import numpy as np
import math

from gym import utils
from gym.envs.mujoco import MujocoEnv
from gym.spaces import Box

from ament_index_python.packages import get_package_share_directory


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
            low=-1.0,
            high=1.0,
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

        # Distribution of random initial states
        self._init_pos_high = math.pi
        self._init_pos_low = -math.pi
        self._init_vel_high = 3.0
        self._init_vel_low = -3.0

        # Reward function parameters
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

    def step(self, action):
        self.do_simulation(action, self.frame_skip)
        self.bound_velocities()
        obs = self._get_obs()
        reward = self.calculate_reward(obs, action)

        # Terminate if simulation become unstable
        terminated = bool(not np.isfinite(obs).all())

        if self.render_mode == "human":
            self.render()

        return obs, reward, terminated, False, {}

    def reset_model(self):
        # TODO: at parameter for selecting starting state (random or only downward)
        # Start at random state
        qpos = self.init_qpos + self.np_random.uniform(
            size=self.model.nq, low=self._init_pos_low, high=self._init_pos_high
        )
        qpos[1] -= math.pi
        qvel = self.init_qvel + self.np_random.uniform(
            size=self.model.nv, low=self._init_vel_low, high=self._init_vel_high
        )

        # Start at downward position
        # qpos = self.init_qpos
        # qpos[1] = -math.pi
        # qvel = self.init_qvel

        self.set_state(qpos, qvel)
        return self._get_obs()

    def bound_velocities(self):
        # Bound velocities to set ranges - it isn't possible to define it in xml model
        self.data.qvel[0] = np.clip(
            self.data.qvel[0], -self._max_velocity_joint0, self._max_velocity_joint0
        )
        self.data.qvel[1] = np.clip(
            self.data.qvel[1], -self._max_velocity_joint1, self._max_velocity_joint1
        )

    def _get_obs(self):
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
