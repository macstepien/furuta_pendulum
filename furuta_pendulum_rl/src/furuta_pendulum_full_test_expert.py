import gym
from gym.envs.registration import register

from lqr_expert import LQRExpert

max_episode_steps = 10000
register(
    id="FurutaPendulum-v0",
    entry_point="furuta_pendulum_full:FurutaPendulumEnv",
    max_episode_steps=max_episode_steps,
)

env = gym.make("FurutaPendulum-v0", render_mode="human")

env.reset()
obs = env.reset_model()
i = 0
lqr_expert = LQRExpert()
while True:
    action = lqr_expert([obs])
    obs, rewards, done, info, _ = env.step(action[0][0])
    env.render()
    i += 1
    if done or i > max_episode_steps:
        break
