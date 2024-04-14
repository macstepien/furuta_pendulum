import gym
from gym.envs.registration import register

from stable_baselines3 import SAC
from stable_baselines3.common.env_util import make_vec_env

from stable_baselines3.common.evaluation import evaluate_policy

import torch

torch.set_default_device("cuda")

max_episode_steps = 2500
register(
    id="FurutaPendulum-v0",
    entry_point="furuta_pendulum_full:FurutaPendulumEnv",
    max_episode_steps=max_episode_steps,
)


# train_model = False
train_model = True

if train_model:
    # Parallel environments
    env = make_vec_env("FurutaPendulum-v0", n_envs=12)

    model = SAC(
        "MlpPolicy",
        env,
        policy_kwargs=dict(net_arch=[512, 512, 512]),
    )

    # With pretraining
    # model = SAC.load(
    #     "furuta_pendulum_rl/trained_agents/furuta_pendulum_full_pretrained",
    #     env,
    #     learning_rate=0.0001,
    #     learning_starts=200_000,
    #     use_sde=True,
    #     sde_sample_freq = 1,
    #     # train_freq=10,
    # )

    model.tensorboard_log = "furuta_pendulum_rl/furuta_pendulum_logs"
    model.verbose = 1

    reward_before, _ = evaluate_policy(model, env, n_eval_episodes=10)

    model.learn(total_timesteps=300_000, progress_bar=True)
    model.save("furuta_pendulum_rl/trained_agents/furuta_pendulum_full")

    reward_after, _ = evaluate_policy(model, env, n_eval_episodes=10)

    print(f"Reward before training: {reward_before}, reward after {reward_after}")

    print("Press Enter to continue...")
    input()
    evaluation_env = gym.make("FurutaPendulum-v0", render_mode="human")
    reward, _ = evaluate_policy(model, evaluation_env, n_eval_episodes=5, render=True)

else:
    env = gym.make("FurutaPendulum-v0", render_mode="human")
    model = SAC.load("furuta_pendulum_rl/trained_agents/furuta_pendulum_full")

    reward, _ = evaluate_policy(model, env, n_eval_episodes=10)
    print(f"Reward: {reward}")
