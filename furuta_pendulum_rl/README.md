# furuta_pendulum_rl

## Setup
### Mujoco
Copy mujoco210 to `~/.mujoco/mujoco210`

## Training

To solve Furuta control, I created three environments with progressive level of difficulty:
 * **furuta_pendulum_upward_stabilization** - pendulum starts with some angle that is a little off upward position and with some velocity, the agent's task is to stabilize in the 0 position. A reward is received for every episode, in which the angle isn't greater than some threshold.
 * **furuta_pendulum_swing_up** - pendulum starts in the downward stable position, the agent's task is to swing it up close to the upward position
 * **furuta_pendulum_full** - this is a full Furuta pendulum control problem - the pendulum starts in the downward stable position, and the agent's task is to swing it up and stabilize it upward.

## Solutions

In scripts with the name of the environment and the name of the algorithm you can find example solutions.

## Exporting model

Due to problems with ROS2 performance in Python I decided to use C++. To export the actor model from SAC used to solve the full control problem, I used `export_sac_model.py` script.