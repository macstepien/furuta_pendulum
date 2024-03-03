from enum import Enum

import gym
from gym.envs.registration import register
# from gymnasium.envs.registration import register

from stable_baselines3 import PPO, SAC
from stable_baselines3.common.env_util import make_vec_env

from furuta_pendulum_swing_up import FurutaPendulumEnv

max_episode_steps = 2000
register(
    id="FurutaPendulum-v0",
    entry_point="furuta_pendulum_swing_up:FurutaPendulumEnv",
    max_episode_steps=max_episode_steps,
)

class Mode(Enum):
    FIRST_TRAIN_MODEL = 1
    CONTINUE_TRAINING_MODEL = 2
    TEST_MODEL = 3


mode = Mode.FIRST_TRAIN_MODEL
# mode = Mode.CONTINUE_TRAINING_MODEL
# mode = Mode.TEST_MODEL

if mode == Mode.FIRST_TRAIN_MODEL:
    # Parallel environments
    env = make_vec_env("FurutaPendulum-v0", n_envs=8)
    # env = make_vec_env(FurutaPendulumEnv, n_envs=8)
    model = SAC(
        "MlpPolicy",
        env,
        verbose=1,
        tensorboard_log="furuta_pendulum_rl/furuta_pendulum_logs",
        # policy_kwargs=dict(net_arch=[512, 512, 512]),
    )
    model.learn(total_timesteps=200000, progress_bar=True)
    model.save("furuta_pendulum_rl/trained_agents/ppo_furuta_pendulum_swing_up")
elif mode == Mode.CONTINUE_TRAINING_MODEL:
    env = make_vec_env("FurutaPendulum-v0", n_envs=8)
    model = SAC.load("furuta_pendulum_rl/trained_agents/ppo_furuta_pendulum_swing_up", env)
    model.learn(total_timesteps=200000, progress_bar=True)
    model.save("furuta_pendulum_rl/trained_agents/ppo_furuta_pendulum_swing_up")
elif mode == Mode.TEST_MODEL:
    env = gym.make("FurutaPendulum-v0", render_mode="human")
    model = SAC.load("furuta_pendulum_rl/trained_agents/ppo_furuta_pendulum_swing_up")

    env.reset()
    obs = env.reset_model()
    while True:
        action, _ = model.predict(obs)
        obs, rewards, done, info, _ = env.step(action)
        env.render()
        if done:
            break
