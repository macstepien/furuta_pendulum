FROM osrf/ros:humble-desktop-full

ARG user=pendulum

RUN apt-get update && \
    apt-get install --no-install-recommends -y sudo && \ 
    useradd -ms /bin/bash ${user} && \
    echo "$user:$user" | chpasswd && \
    adduser ${user} sudo && \
    echo "$user ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

USER ${user}
WORKDIR /home/${user}

RUN sudo apt-get update && \
    sudo apt-get install --no-install-recommends -y \
    nano gdb clang clang-format cppcheck bash-completion python3-pip unzip \
    git htop ros-$ROS_DISTRO-rmw-fastrtps-cpp ros-$ROS_DISTRO-plotjuggler \
    ros-$ROS_DISTRO-plotjuggler-ros ros-$ROS_DISTRO-rviz2 ros-$ROS_DISTRO-teleop-twist-keyboard \
    ros-$ROS_DISTRO-cyclonedds ros-$ROS_DISTRO-rmw-cyclonedds-cpp \
    python3-vcstool python3-rosdep python3-colcon-common-extensions doxygen && \
    /usr/bin/python3 -m pip install -U black control --user
    # control - python control toolbox

RUN pip install git+https://github.com/carlosluis/stable-baselines3@fix_tests && \
    pip install mujoco tensorboard stable-baselines3[extra] imageio

# pyTorch C++
# Necessary to get an ABI version to use with ROS2!
RUN wget https://download.pytorch.org/libtorch/cpu/libtorch-cxx11-abi-shared-with-deps-2.0.0%2Bcpu.zip && \
    unzip libtorch-cxx11-abi-shared-with-deps-2.0.0+cpu.zip

RUN sudo apt-get update && \
    sudo apt-get install --no-install-recommends -y ca-certificates gnupg lsb-release wget && \
    wget -qO- https://drake-apt.csail.mit.edu/drake.asc | gpg --dearmor - \
        | sudo tee /etc/apt/trusted.gpg.d/drake.gpg >/dev/null && \
    echo "deb [arch=amd64] https://drake-apt.csail.mit.edu/$(lsb_release -cs) $(lsb_release -cs) main" \
        | sudo tee /etc/apt/sources.list.d/drake.list >/dev/null && \
    sudo apt-get update && \
    sudo apt-get install --no-install-recommends drake-dev

RUN sudo apt-get update && \
    sudo apt install liblapack-dev libeigen3-dev coinor-libipopt-dev libboost-all-dev libomp-dev clang python3 python3-dev python3-numpy python3-matplotlib

RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /home/${user}/.bashrc && \
    echo "source /home/$user/ros2_ws/install/setup.bash --extend" >> /home/${user}/.bashrc && \
    echo "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/.mujoco/mujoco210/bin" >> /home/${user}/.bashrc && \
    echo "export LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libGLEW.so" >> /home/${user}/.bashrc  && \
    echo "export CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH:~/libtorch" >> /home/${user}/.bashrc  && \
    echo "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/libtorch/lib/" >> /home/${user}/.bashrc  && \
    echo "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/drake/lib/" >> /home/${user}/.bashrc  && \
    echo "export PATH=\"/opt/drake/bin${PATH:+:${PATH}}\"" >> /home/${user}/.bashrc  && \
    echo "export PYTHONPATH=\"/opt/drake/lib/python$(python3 -c 'import sys; print("{0}.{1}".format(*sys.version_info))')/site-packages${PYTHONPATH:+:${PYTHONPATH}}\"" >> /home/${user}/.bashrc  && \

# /opt/drake/lib/ - To have a working executable after installation

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp