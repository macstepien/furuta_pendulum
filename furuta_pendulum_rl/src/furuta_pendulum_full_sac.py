import gym
from gym.envs.registration import register

from stable_baselines3 import SAC, PPO
from stable_baselines3.common.env_util import make_vec_env

from stable_baselines3.common.evaluation import evaluate_policy


max_episode_steps = 1024
register(
    id="FurutaPendulum-v0",
    entry_point="furuta_pendulum_full:FurutaPendulumEnv",
    max_episode_steps=max_episode_steps,
)


train_model = False
# train_model = True

if train_model:
    # Parallel environments
    env = make_vec_env("FurutaPendulum-v0", n_envs=16)
    # env = gym.make("FurutaPendulum-v0", render_mode="human")
    # model = SAC(
    #     "MlpPolicy",
    #     env,
    #     verbose=1,
    #     tensorboard_log="furuta_pendulum_rl/ppo_furuta_pendulum_full_log",
    #     policy_kwargs=dict(net_arch=[512, 512, 512]),
    # )
    # model = SAC.load("furuta_pendulum_rl/trained_agents/ppo_furuta_pendulum_swing_up", env)
    model = PPO.load(
        # "furuta_pendulum_rl/trained_agents/furuta_pendulum_full_pretrained",
        "furuta_pendulum_rl/trained_agents/furuta_pendulum_full",
        env,
        device="cuda:0",
    )
    model.tensorboard_log = "furuta_pendulum_rl/ppo_furuta_pendulum_full_log"
    model.verbose = 1

    reward_before, _ = evaluate_policy(model, env, n_eval_episodes=10)

    model.learn(total_timesteps=500000, progress_bar=True)
    model.save("furuta_pendulum_rl/trained_agents/furuta_pendulum_full")

    reward_after, _ = evaluate_policy(model, env, n_eval_episodes=10)

    print(f"Reward before training: {reward_before}, reward after {reward_after}")
else:
    env = gym.make("FurutaPendulum-v0", render_mode="human")
    model = PPO.load("furuta_pendulum_rl/trained_agents/furuta_pendulum_full")
    # model = PPO.load("furuta_pendulum_rl/trained_agents/furuta_pendulum_full_pretrained")
    # model = SAC.load("furuta_pendulum_rl/trained_agents/ppo_furuta_pendulum_swing_up")

    env.reset()
    obs = env.reset_model()
    i = 0
    while True:
        action, _ = model.predict(obs)
        obs, rewards, done, info, _ = env.step(action)
        env.render()
        i += 1
        # if done or i > max_episode_steps:
        if i > max_episode_steps:
            break
