
Packages in this repository can be divided into two subgroups: 

1. Different approaches to solving control problem:
 * **furuta_pendulum_de** - uses manually derived Differential Equations (hence the name) with manually calculated linearization, which is then used for calculating LQR controller with swing-up
 * **furuta_pendulum_drake** - in this package the same LQR with swing-up approach is used, but this time manually derived equations are replaced. Instead [Drake library](https://drake.mit.edu/) is used to load URDF of pendulum, and solely based on it, linearization is calculated, which is once again used to calculate K gain.  
 * **furuta_pendulum_control_toolbox** - in this approach different library is used - [Control Toolbox](https://github.com/ethz-adrl/control-toolbox). It also automatically derives simulation of the pendulum based on this model (which is in form of code generated based on URDF model). There are two approaches used here, one is the same, as in previous ones - LQR with swing-up, but this time Control Toolbox is used to get K. Another approach uses MPC to get optimal solution.
 * **furuta_pendulum_rl** - in this package Deep Reinforcement Learning approach is used. Pendulum is modeled in the Mujoco, which then is used to create Gym environment. It is then used to train agent. Model is then exported and run using C++ controller. 

2. General utility packages:
 * **furuta_pendulum_bringup** - general launch files, which combines controllers from different packages, with different simulation environments. Check out description details below.
 * **furuta_pendulum_description** - URDF of the pendulum and config file with physical properties
 * **furuta_pendulum_gazebo** - simulation config in Gazebo, which uses ros2_control with torque control

## Building
Some packages require installing additional dependencies, if they can't be installed easily with rosdep, I usually add them to the devcontainer. I didn't have time to automate some of them, so you may have to do it manually, for details refer to README file of each package.

## Usage
To run pendulum simulation use one of the launch files from **furuta_pendulum_bringup**, which will be described below.

## Packages

### furuta_pendulum_bringup
Launch files have naming convention:

*simulator_type*_with_*controller_type*.launch.py

Available simulator_types:
 * gazebo
 * de_simulation

Available controllers:
 * de_controller
 * drake_controller
 * ct_controller
 * rl_controller


### furuta_pendulum_de
### furuta_pendulum_drake
### furuta_pendulum_control_toolbox
### furuta_pendulum_rl
### furuta_pendulum_description
### furuta_pendulum_gazebo