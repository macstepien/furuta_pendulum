
https://ethz-adrl.github.io/ct/ct_doc/doc/html/rbd_tut_modelling.html

Generating kindsl
https://github.com/leggedrobotics/urdf2robcogen

git clone https://github.com/leggedrobotics/urdf2robcogen.git
git clone  https://github.com/ANYbotics/kindr_ros.git
rosrun urdf2robcogen urdf2robcogen_script FurutaPendulum /home/roomac/catkin_ws/src/roomac_ros/furuta_pendulum.urdf

https://robcogenteam.bitbucket.io/binary.html
https://robcogenteam.bitbucket.io/usage.html

sudo apt install maxima

! has to bo version 0.4ad.0
https://github.com/ethz-adrl/control-toolbox/issues/166
Simply removing a "/" sign from the framework.properties configuration resolved this issue, like:
generator.maxima.libs.path = ../etc/maxima-libs (do NOT use "/" at the end).

./robcogen.sh /home/maciej/ros2_ws/src/furuta_pendulum/furuta_pendulum_control_toolbox/model/FurutaPendulum.kindsl
# you may now input 1, 4, and 28 to exit

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

Fix:
fatal error: iit/rbd/scalar_traits.h: No such file or directory
    5 | #include <iit/rbd/scalar_traits.h>
      |          ^~~~~~~~~~~~~~~~~~~~~~~~~

cd ~/ros2_ws/robcogen-0.5.2/etc/cpp-iitrbd
sudo ./install.sh 

! in tmp it is generated for newer 0.5 version
<!-- /tmp/gen$ cp -r cpp/ /home/maciej/ros2_ws/src/furuta_pendulum/furuta_pendulum_control_toolbox/include/furuta_pendulum/ -->
maciej@maciej:~/ros2_ws/robcogen-0.4ad.0/run/gen_code$ cp -r cpp ~/ros2_ws/src/furuta_pendulum/furuta_pendulum_control_toolbox/include/furuta_pendulum/