import gym
from gym.envs.registration import register

from stable_baselines3 import PPO
from stable_baselines3.common.env_util import make_vec_env

max_episode_steps = 200
register(
    id="FurutaPendulum-v0",
    entry_point="furuta_pendulum_swingup:FurutaPendulumEnv",
    max_episode_steps=max_episode_steps,
)


train_model = False
# train_model = True

if train_model:
    # Parallel environments
    env = make_vec_env("FurutaPendulum-v0", n_envs=4)
    model = PPO(
        "MlpPolicy",
        env,
        verbose=1,
        tensorboard_log="furuta_pendulum_rl/ppo_furuta_pendulum_swingup_log",
    )
    model.learn(total_timesteps=100000, progress_bar=True)
    model.save("furuta_pendulum_rl/trained_agents/ppo_furuta_pendulum_swingup")
else:
    env = gym.make("FurutaPendulum-v0", render_mode="human")
    model = PPO.load("furuta_pendulum_rl/trained_agents/ppo_furuta_pendulum_swingup")

    env.reset()
    obs = env.reset_model()
    while True:
        action, _states = model.predict(obs)
        obs, rewards, dones, info, _ = env.step(action)
        env.render()
        if dones:
            break
