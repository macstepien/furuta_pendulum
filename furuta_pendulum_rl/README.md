
## Installation
### Mujoco
Copy mujoco210 to ~/.mujoco/mujoco210

echo "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/.mujoco/mujoco210/bin" >> ~/.bashrc
echo "export LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libGLEW.so" >> ~/.bashrc
pip install git+https://github.com/carlosluis/stable-baselines3@fix_tests
pip install mujoco tensorboard stable-baselines3[extra] imageio

### pyTorch C++

!Necessary to get abi to use with ROS2!
wget https://download.pytorch.org/libtorch/cpu/libtorch-cxx11-abi-shared-with-deps-2.0.0%2Bcpu.zip
unzip libtorch-cxx11-abi-shared-with-deps-2.0.0+cpu.zip

Add to .bashrc:
export CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH:~/libtorch
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/libtorch/lib/

## Training

To solve furuta control, I created three environments with progressive level of difficulty:
 * **furuta_pendulum_upward_stabilization** - pendulum starts with some angle that is a little off upward position and with some velocity, agent's task is to stabilize in the 0 position. Reward is received for every episode, in which angle isn't greater than some threshold.
 * **furuta_pendulum_swing_up** - pendulum starts in downward stable position, agents task is to swing it up close to upward position
 * **furuta_pendulum_full** - this is full furuta pendulum control problem - pendulum starts in the downward stable position, and agent's task is to swing it up and stabilize it upward.

## Solutions

In the scripts with name of the environment and name of algorithm you find example solutions.
## Exporting model

Due to problems with ROS2 performance in Python I decided to use C++. To export Actor model from SAC used to solve full control problem, I used `export_sac_model.py` script.