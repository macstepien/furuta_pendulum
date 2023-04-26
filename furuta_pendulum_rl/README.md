# Mujoco
Copy mujoco210 to ~/.mujoco/mujoco210

echo "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/.mujoco/mujoco210/bin" >> ~/.bashrc
echo "export LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libGLEW.so" >> ~/.bashrc
pip install git+https://github.com/carlosluis/stable-baselines3@fix_tests
pip install mujoco tensorboard stable-baselines3[extra] imageio

## pyTorch C++

!Necessary to get abi to use with ROS2!
wget https://download.pytorch.org/libtorch/cpu/libtorch-cxx11-abi-shared-with-deps-2.0.0%2Bcpu.zip
unzip libtorch-cxx11-abi-shared-with-deps-2.0.0+cpu.zip

Add to .bashrc:
export CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH:~/libtorch
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/libtorch/lib/
