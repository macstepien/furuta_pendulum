
## Setup

Based on [tutorial from control_toolbox](https://ethz-adrl.github.io/ct/ct_doc/doc/html/rbd_tut_modelling.html).

### Generating dtdsl and kindsl
[urdf2robcogen](https://github.com/leggedrobotics/urdf2robcogen) is used. Clone this to repos into your ROS1 workspace:

```
git clone https://github.com/leggedrobotics/urdf2robcogen.git
git clone https://github.com/ANYbotics/kindr_ros.git
```
Build and then run following command to generate `dtdsl` and `kindsl`:
```
rosrun urdf2robcogen urdf2robcogen_script FurutaPendulum /PATH_TO_URDF/furuta_pendulum.urdf
```
### Generating code

[RobCoGen](https://robcogenteam.bitbucket.io/usage.html) is used in this step. First install dependency:
```
sudo apt install maxima
```

Then download version `0.4ad.0` of RobCoGen from [this page](https://robcogenteam.bitbucket.io/binary.html). 

Then, before generating code, it is necessary to fix some problems.
First ons is described in this [github issue](https://github.com/ethz-adrl/control-toolbox/issues/166), as in the issue I removed a "/" sign from the framework.properties (`generator.maxima.libs.path = ../etc/maxima-libs`, without "/" at the end).

<!-- Fix:
fatal error: iit/rbd/scalar_traits.h: No such file or directory
    5 | #include <iit/rbd/scalar_traits.h>
      |          ^~~~~~~~~~~~~~~~~~~~~~~~~ -->
Now install `iit`:
```
cd /PATH_TO_ROBCOGEN/robcogen-0.4ad.0/etc/cpp-iitrbd
sudo ./install.sh 
```

Install dependencies (based on `install_deps.sh` control_toolbox, without running `install_cmake.sh`, as it caused problems with ROS2 building):
```
sudo apt-get update
sudo apt install liblapack-dev libeigen3-dev coinor-libipopt-dev libboost-all-dev libomp-dev clang python3 python3-dev python3-numpy python3-matplotlib
```
Then navigate to your control_toolbox directory and:
```
cd ct
sudo ./install_cppadcg.sh
```

Finally to generate code run:
```
./robcogen.sh /PATH_TO_KINDSL/FurutaPendulum.kindsl /PATH_TO_DTDSL/FurutaPendulum.dtdsl
```
You should see following menu:
```
 0 - CTDSL      - The coordinate transforms description file .ctdsl

 1 - ALLMx      - [All the Maxima targets]
 2 - TMx        - Maxima coordinate transforms
 3 - JMx        - Maxima geometric Jacobians

 4 - ALLC++     - [All the C++ targets]
 5 - CC++       - C++ common code
 6 - TC++       - C++ coordinate Transform
 7 - JC++       - C++ geometric Jacobians
 8 - DC++       - C++ common code for dynamics
 9 - IDC++      - C++ Inverse-Dynamics implementation
10 - FDC++      - C++ Forward-Dynamics implementation
11 - JSIMC++    - C++ Joint-Space-Inertia-Matrix calculation
12 - MKC++      - CMake file for C++ code

13 - ALLMt      - [All the Matlab targets]
14 - CMt        - Matlab common code
15 - TMt        - Matlab coordinate transforms
16 - JMt        - Matlab geometric Jacobians
17 - IDMt       - Matlab Forward/Inverse-Dynamics implementation
18 - JSIMMt     - Matlab Joint-Space-Inertia-Matrix calculation

19 - SL - [All the SL targets]
20 - SLR        - SL, robot code (includes the robot model .dyn)
21 - SLU        - SL, robot-user code

22 - RB - [All the robot models]
23 - RB_ROS     - XML URDF format
24 - RB_Feath   - Matlab structure for Featherstone's code
25 - RB_SDFAST  - SD/FAST model descritpion

26 - Test       - Support code to launch the Octave tests for Octave/C++

27 - Reload     - Reload the input files
28 - Quit       - Exit robcogen

What would you like to generate? Please enter the integer code:
```

Now select option 1, then 4 and 28 to exit. Model sources will be generated in the `robcogen-0.4ad.0/run/gen_code` directory.

## Building

For some reason, it works correctly only in `RelWithDebInfo`
```
colcon build --symlink-install --packages-select furuta_pendulum_control_toolbox --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

in Release i got: 
```
waiting 1 second for begin
furuta_pendulum_nloc: /usr/include/eigen3/Eigen/src/Core/Block.h:146: Eigen::Block<XprType, BlockRows, BlockCols, InnerPanel>::Block(XprType&, Eigen::Index, Eigen::Index, Eigen::Index, Eigen::Index) [with XprType = Eigen::Matrix<double, 1, 1, 0, 1, 1>; int BlockRows = 2; int BlockCols = 1; bool InnerPanel = false; Eigen::Index = long int]: Assertion `startRow >= 0 && blockRows >= 0 && startRow <= xpr.rows() - blockRows && startCol >= 0 && blockCols >= 0 && startCol <= xpr.cols() - blockCols' failed.
```