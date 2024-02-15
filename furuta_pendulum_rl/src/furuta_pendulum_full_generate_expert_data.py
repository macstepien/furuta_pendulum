import numpy as np
from enum import Enum

import gym
from gym.envs.registration import register

from stable_baselines3 import SAC, PPO
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.evaluation import evaluate_policy

from imitation.algorithms import bc
from imitation.algorithms.adversarial.gail import GAIL
from imitation.algorithms.adversarial.airl import AIRL
import imitation.data.rollout as rollout
from imitation.rewards.reward_nets import BasicRewardNet, BasicShapedRewardNet
from imitation.util.networks import RunningNorm

from lqr_expert import LQRExpert


max_episode_steps = 1024
register(
    id="FurutaPendulum-v0",
    entry_point="furuta_pendulum_full:FurutaPendulumEnv",
    max_episode_steps=max_episode_steps,
)

env = make_vec_env("FurutaPendulum-v0", n_envs=1, env_kwargs=dict(render_mode="human"))
# env = make_vec_env("FurutaPendulum-v0", n_envs=1)
evaluation_env = gym.make("FurutaPendulum-v0", render_mode="human")

lqr_expert = LQRExpert()

expert_reward, _ = evaluate_policy(lqr_expert, env, n_eval_episodes=1)

rng = np.random.default_rng(0)

# https://imitation.readthedocs.io/en/latest/main-concepts/trajectories.html
rollouts = rollout.rollout(
    lqr_expert,
    env,
    sample_until=rollout.make_sample_until(min_episodes=20),
    rng=rng,
    unwrap=False,
)

class ImitationMethod(Enum):
    BC = 1
    GAIL = 2
    AIRL = 3


imitation_method = ImitationMethod.BC

if imitation_method == ImitationMethod.BC:
    transitions = rollout.flatten_trajectories(rollouts)
    bc_trainer = bc.BC(
        observation_space=env.observation_space,
        action_space=env.action_space,
        demonstrations=transitions,
        rng=rng,
        device="cpu",
    )
    bc_trainer.train(n_epochs=10)
    model = bc_trainer.policy

elif imitation_method == ImitationMethod.GAIL:
    model = SAC(
        "MlpPolicy",
        env,
        # verbose=1,
        # tensorboard_log="furuta_pendulum_rl/ppo_furuta_pendulum_swing_up_log",
        # policy_kwargs=dict(net_arch=[512, 512, 512]),
        device="cpu",
    )
    reward_net = BasicRewardNet(
        observation_space=env.observation_space,
        action_space=env.action_space,
        normalize_input_layer=RunningNorm,
    )
    gail_trainer = GAIL(
        demonstrations=rollouts,
        demo_batch_size=1200,
        gen_replay_buffer_capacity=512,
        n_disc_updates_per_round=8,
        venv=env,
        gen_algo=model,
        reward_net=reward_net,
    )
    gail_trainer.train(2_000)
    model.save("furuta_pendulum_rl/trained_agents/furuta_pendulum_full_pretrained")

elif imitation_method == ImitationMethod.AIRL:
    model = PPO(
        env=env,
        policy="MlpPolicy",
    )
    reward_net = BasicShapedRewardNet(
        observation_space=env.observation_space,
        action_space=env.action_space,
        # normalize_input_layer=RunningNorm,
    )
    airl_trainer = AIRL(
        demonstrations=rollouts,
        demo_batch_size=1024,
        gen_replay_buffer_capacity=512,
        n_disc_updates_per_round=16,
        venv=env,
        gen_algo=model,
        reward_net=reward_net,
    )

    airl_trainer.train(50_000)
    model.save("furuta_pendulum_rl/trained_agents/furuta_pendulum_full_pretrained")


print("Press Enter to continue...")
input()

print("Evaluating the trained policy.")
reward, _ = evaluate_policy(model, evaluation_env, n_eval_episodes=10, render=True)

print(f"Expert reward: {expert_reward}")
print(f"Reward after training: {reward}")
