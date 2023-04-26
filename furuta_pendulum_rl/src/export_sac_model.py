# https://stable-baselines3.readthedocs.io/en/master/guide/export.html

import torch as th

from stable_baselines3 import SAC


class OnnxablePolicy(th.nn.Module):
    def __init__(self, actor: th.nn.Module):
        super().__init__()
        # Removing the flatten layer because it can't be onnxed
        self.actor = th.nn.Sequential(
            actor.latent_pi,
            actor.mu,
            # For gSDE
            # th.nn.Hardtanh(min_val=-actor.clip_mean, max_val=actor.clip_mean),
            # Squash the output
            th.nn.Tanh(),
        )

    def forward(self, observation: th.Tensor) -> th.Tensor:
        # NOTE: You may have to process (normalize) observation in the correct
        #       way before using this. See `common.preprocessing.preprocess_obs`
        return self.actor(observation)


model = SAC.load("furuta_pendulum_rl/trained_agents/furuta_pendulum_full", device="cpu")
# model = SAC.load("PathToTrainedModel.zip", device="cpu")
onnxable_model = OnnxablePolicy(model.policy.actor)

observation_size = model.observation_space.shape
dummy_input = th.randn(1, *observation_size)

# th.onnx.export(
#     onnxable_model,
#     dummy_input,
#     "my_sac_actor.onnx",
#     opset_version=9,
#     input_names=["input"],
# )

jit_path = "furuta_pendulum_rl/trained_agents/furuta_pendulum_full.pt"

# Trace and optimize the module
traced_module = th.jit.trace(onnxable_model.eval(), dummy_input)
frozen_module = th.jit.freeze(traced_module)
frozen_module = th.jit.optimize_for_inference(frozen_module)
th.jit.save(frozen_module, jit_path)
