import gym
from gym.envs.registration import register

from stable_baselines3 import TD3
from stable_baselines3.common.env_util import make_vec_env

max_episode_steps = 700
register(
    id="FurutaPendulum-v0",
    entry_point="furuta_pendulum_full:FurutaPendulumEnv",
    max_episode_steps=max_episode_steps,
)


# train_model = False
train_model = True

if train_model:
    # Parallel environments
    # env = make_vec_env("FurutaPendulum-v0", n_envs=12)
    env = gym.make("FurutaPendulum-v0", render_mode="human")
    model = TD3(
        "MlpPolicy",
        env,
        verbose=1,
        tensorboard_log="furuta_pendulum_rl/ppo_furuta_pendulum_full_log",
        # policy_kwargs=dict(net_arch=[512, 512, 512]),
    )
    model.learn(total_timesteps=300000, progress_bar=True)
    model.save("furuta_pendulum_rl/trained_agents/furuta_pendulum_full")
else:
    env = gym.make("FurutaPendulum-v0", render_mode="human")
    model = TD3.load("furuta_pendulum_rl/trained_agents/furuta_pendulum_full")

    env.reset()
    obs = env.reset_model()
    i = 0
    while True:
        action, _states = model.predict(obs)
        obs, rewards, done, info, _ = env.step(action)
        # if i % 2 == 0:
        env.render()
        i += 1
        if done or i > max_episode_steps:
            break
