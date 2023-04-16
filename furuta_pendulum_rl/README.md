## pyTorch C++

<!-- wget https://download.pytorch.org/libtorch/nightly/cpu/libtorch-shared-with-deps-latest.zip -->
<!-- https://download.pytorch.org/libtorch/cu117/libtorch-cxx11-abi-shared-with-deps-2.0.0%2Bcu117.zip -->
<!-- unzip libtorch-shared-with-deps-latest.zip -->

!Necessary to get abi to use with ROS2!
wget https://download.pytorch.org/libtorch/cpu/libtorch-cxx11-abi-shared-with-deps-2.0.0%2Bcpu.zip
unzip libtorch-cxx11-abi-shared-with-deps-2.0.0+cpu.zip

export CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH:/home/maciej/libtorch