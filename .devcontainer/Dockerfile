FROM husarion/rosbot-xl:humble-simulation

ARG user=maciej

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
    nano gdb clang clang-format cppcheck bash-completion python3-pip \
    git htop ros-$ROS_DISTRO-rmw-fastrtps-cpp ros-$ROS_DISTRO-plotjuggler \
    ros-$ROS_DISTRO-plotjuggler-ros ros-$ROS_DISTRO-rviz2 ros-$ROS_DISTRO-teleop-twist-keyboard \
    python3-vcstool python3-rosdep python3-colcon-common-extensions && \
    /usr/bin/python3 -m pip install -U black control --user
    # control - python control toolbox
    

RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /home/${user}/.bashrc && \
    echo "source /home/$user/ros2_ws/install/setup.bash --extend" >> /home/${user}/.bashrc

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# https://drake.mit.edu/apt.html#stable-releases