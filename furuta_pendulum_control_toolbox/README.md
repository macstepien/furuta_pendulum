# furuta_pendulum_control_toolbox

## Files

`ct_demo_simulation` - simple simulation of Furuta pendulum with some hardcoded initial state, state trajectories are then plotted. Its main purpose is the initial verification of generated model
`ros2 run furuta_pendulum_control_toolbox ct_demo_simulation`

`ct_demo_simulation_node` - similar to the previous one, but this time initial state is read from parameters, and results are published in a `joint_state` message, which can be later used to visualize the model in RViz. Used for further model verification.
`ros2 launch furuta_pendulum_control_toolbox ct_simulation.launch.py`

`ct_lqr` - linearizes system and calculates LQR gains, which can be later used in the `lqr_with_swing_up_controller`
`ros2 run furuta_pendulum_control_toolbox ct_lqr`
`ros2 launch furuta_pendulum_control_toolbox ct_lqr_controller.launch.py`

`ct_demo_simulation_mpc` - first attempt at creating an MPC controller for the Furuta pendulum. After creating the controller, it is then verified using simulation and results are printed.
`ros2 run furuta_pendulum_control_toolbox ct_demo_simulation_mpc`

`ct_demo_simulation_mpc_node` - it is a previous demo, but with publishing `joint_state` messages, so that everything could be visualized in RViz.
`ros2 launch furuta_pendulum_control_toolbox ct_simulation_with_mpc_controller.launch.py`

`ct_mpc_controller_node` - MPC controller from previous demos, extracted into a standalone node. It subscribes to `joint_state` messages (as this time it isn't combined with the simulator) and calculates controls.
`ros2 launch furuta_pendulum_control_toolbox ct_mpc_controller.launch.py`

## Minimal setup
Navigate to your control_toolbox directory and:
```
cd ct
sudo ./install_cppadcg.sh
sudo ./install_hpipm.sh
```
## Setup

Based on [tutorial from control_toolbox](https://ethz-adrl.github.io/ct/ct_doc/doc/html/rbd_tut_modelling.html).

### Generating dtdsl and kindsl
[urdf2robcogen](https://github.com/leggedrobotics/urdf2robcogen) is used. Clone this to repos into your ROS1 workspace:

```
git clone https://github.com/leggedrobotics/urdf2robcogen.git
git clone https://github.com/ANYbotics/kindr_ros.git
```
Build and then run the following command to generate `dtdsl` and `kindsl`:
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
First one is described in this [github issue](https://github.com/ethz-adrl/control-toolbox/issues/166), as in the issue I removed a "/" sign from the framework.properties (`generator.maxima.libs.path = ../etc/maxima-libs`, without "/" at the end).

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
You should see the following menu:
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

For bounded problem it is necessary to `install_hpipm.sh`
!Warning! Don't use install_hpipm script, it is necessary to look what is done and repeat it manually, because it is necessary to disable blasfeo examples.

I got a problem that blasfeo examples didn't build and I had to disable them in the CMakeLists.txt of blasfeo:
```
# set(BLASFEO_EXAMPLES ON CACHE BOOL "Examples enabled")
set(BLASFEO_EXAMPLES OFF CACHE BOOL "Examples disabled")
```

Then I had to modify `ct_models` CMakeLists to add blasfeo and HPIPM.
```
cmake_minimum_required (VERSION 3.5)

if(NOT TARGET clang-format)
  include(${CMAKE_CURRENT_SOURCE_DIR}/../ct/cmake/compilerSettings.cmake)
  include(${CMAKE_CURRENT_SOURCE_DIR}/../ct/cmake/explicitTemplateHelpers.cmake)
  include(${CMAKE_CURRENT_SOURCE_DIR}/../ct/cmake/clang-cxx-dev-tools.cmake)
endif()
include(${CMAKE_CURRENT_SOURCE_DIR}/../ct/cmake/ct-cmake-helpers.cmake)


project(ct_models VERSION 3.0.2 LANGUAGES CXX)

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wfatal-errors -std=c++14 -Wall -Wno-unknown-pragmas")
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

if(NOT TARGET ${ct_rbd})
  find_package(ct_rbd REQUIRED)
endif()
find_package(Boost REQUIRED system filesystem)

## include blasfeo and hpipm, assumed to be installed in "/opt"
list(APPEND CMAKE_PREFIX_PATH "/opt")
find_package(blasfeo QUIET)
find_package(hpipm QUIET)
if(blasfeo_FOUND AND hpipm_FOUND)
    message(STATUS "Found HPIPM and BLASFEO")
    set(HPIPM ON)
    list(APPEND HPIPM_LIBS hpipm blasfeo)
    list(APPEND ct_optcon_COMPILE_DEFINITIONS HPIPM)
else()
    message(WARNING "Could not find HPIPM or BLASFEO")
endif()

# extract interface compile definitions from previous ct packages as options
importInterfaceCompileDefinitionsAsOptions(ct_core)
importInterfaceCompileDefinitionsAsOptions(ct_optcon)
importInterfaceCompileDefinitionsAsOptions(ct_rbd)


## define the directories to be included in all ct_rbd targets
set(ct_models_target_include_dirs
    ${ct_rbd_INCLUDE_DIRS}
    ${blasfeo_INCLUDE_DIRS}
    ${hpipm_INCLUDE_DIRS}
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)


## define placeholder for ct model libraries
set(CT_MODELS_LIBRARIES "")

set(IP_CODEGEN_OUTPUT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/include/ct/models/InvertedPendulum/codegen")
set(HYA_CODEGEN_OUTPUT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/include/ct/models/HyA/codegen")
set(HYQ_CODEGEN_OUTPUT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/include/ct/models/HyQ/codegen")
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/include/ct/models/CodegenOutputDirs.h.in ${CMAKE_CURRENT_SOURCE_DIR}/include/ct/models/CodegenOutputDirs.h)

################ HyQ #################
if(CPPADCG)
  add_executable(HyQLinearizationCodegen src/HyQ/codegen/HyQLinearizationCodegen.cpp)
  target_include_directories(HyQLinearizationCodegen PUBLIC ${ct_models_target_include_dirs})
  target_link_libraries(HyQLinearizationCodegen ct_rbd)
  list(APPEND CT_MODELS_BINARIES HyQLinearizationCodegen)
endif()

if (BUILD_HYQ_FULL)
    add_library(HyQWithContactModelLinearizedForward include/ct/models/HyQ/codegen/HyQWithContactModelLinearizedForward.cpp)
    target_link_libraries(HyQWithContactModelLinearizedForward ct_rbd)
    list(APPEND CT_MODELS_LIBRARIES HyQWithContactModelLinearizedForward)

    ## Forward Dynamics Forward Zero
    add_library(HyQForwardZero include/ct/models/HyQ/codegen/HyQForwardZero.cpp)
    target_link_libraries(HyQForwardZero ct_rbd)
    list(APPEND CT_MODELS_LIBRARIES HyQForwardZero)
endif(BUILD_HYQ_FULL)

if(BUILD_HYQ_LINEARIZATION_TIMINGS)
  if (NOT USE_CLANG)
    MESSAGE(WARNING "HyQ Linearization Timings need to be build with CLANG")
  endif()

  ## Forward Dynamics
  add_library(HyQWithContactModelLinearizedReverse include/ct/models/HyQ/codegen/HyQWithContactModelLinearizedReverse.cpp)
  target_link_libraries(HyQWithContactModelLinearizedReverse ct_rbd)
  list(APPEND CT_MODELS_LIBRARIES HyQWithContactModelLinearizedReverse)

  add_library(HyQBareModelLinearizedForward include/ct/models/HyQ/codegen/HyQBareModelLinearizedForward.cpp)
  target_link_libraries(HyQBareModelLinearizedForward ct_rbd )
  list(APPEND CT_MODELS_LIBRARIES HyQBareModelLinearizedForward)

  add_library(HyQBareModelLinearizedReverse include/ct/models/HyQ/codegen/HyQBareModelLinearizedReverse.cpp)
  target_link_libraries(HyQBareModelLinearizedReverse ct_rbd )
  list(APPEND CT_MODELS_LIBRARIES HyQBareModelLinearizedReverse)

  ## Inverse Dynamics
  add_library(HyQJacInverseDynamicsForward include/ct/models/HyQ/codegen/HyQInverseDynJacForward.cpp)
  target_link_libraries(HyQJacInverseDynamicsForward ct_rbd )
  list(APPEND CT_MODELS_LIBRARIES HyQJacInverseDynamicsForward)

  add_library(HyQJacInverseDynamicsReverse include/ct/models/HyQ/codegen/HyQInverseDynJacReverse.cpp)
  target_link_libraries(HyQJacInverseDynamicsReverse ct_rbd )
  list(APPEND CT_MODELS_LIBRARIES HyQJacInverseDynamicsReverse)

  ## ForwardKinematics
  add_library(HyQJacForwardKinForward include/ct/models/HyQ/codegen/HyQForwardKinJacForward.cpp)
  target_link_libraries(HyQJacForwardKinForward ct_rbd )
  list(APPEND CT_MODELS_LIBRARIES HyQJacForwardKinForward)

  add_library(HyQJacForwardKinReverse include/ct/models/HyQ/codegen/HyQForwardKinJacReverse.cpp)
  target_link_libraries(HyQJacForwardKinReverse ct_rbd )
  list(APPEND CT_MODELS_LIBRARIES HyQJacForwardKinReverse)

  add_executable(HyQcompareForwardReverseFD src/HyQ/codegen/compareForwardReverseFD.cpp)
  target_link_libraries(HyQcompareForwardReverseFD
                        HyQWithContactModelLinearizedForward
                        HyQWithContactModelLinearizedReverse
                        HyQBareModelLinearizedForward
                        HyQBareModelLinearizedReverse
                        ct_rbd)
  list(APPEND CT_MODELS_BINARIES HyQcompareForwardReverseFD)

  add_executable(HyQcompareForwardReverseID src/HyQ/codegen/compareForwardReverseID.cpp)
  target_link_libraries(HyQcompareForwardReverseID
                        HyQJacInverseDynamicsForward
                        HyQJacInverseDynamicsReverse
                        ct_rbd)
  list(APPEND CT_MODELS_BINARIES HyQcompareForwardReverseID)

  add_executable(HyQcompareForwardReverseKin src/HyQ/codegen/compareForwardReverseKin.cpp)
  target_link_libraries(HyQcompareForwardReverseKin
                        HyQJacForwardKinForward
                        HyQJacForwardKinReverse
                        ct_rbd)
  list(APPEND CT_MODELS_BINARIES HyQcompareForwardReverseKin)

  add_executable(HyQcompareForwardZero src/HyQ/codegen/compareForwardZero.cpp)
  target_link_libraries(HyQcompareForwardZero HyQForwardZero ct_rbd)
  list(APPEND CT_MODELS_BINARIES HyQcompareForwardZero)
endif(BUILD_HYQ_LINEARIZATION_TIMINGS)

########## Inverted Pendulum #########
if(CPPADCG)
  add_executable(InvertedPendulumWithActuatorCodeGen src/InvertedPendulum/codegen/InvertedPendulumWithActuatorCodeGen.cpp)
  target_include_directories(InvertedPendulumWithActuatorCodeGen PUBLIC ${ct_models_target_include_dirs})
  target_link_libraries(InvertedPendulumWithActuatorCodeGen ct_rbd)
  list(APPEND CT_MODELS_BINARIES InvertedPendulumWithActuatorCodeGen)
endif()

add_library(InvertedPendulumActDynLinearizedForward include/ct/models/InvertedPendulum/codegen/InvertedPendulumActDynLinearizedForward.cpp)
target_include_directories(InvertedPendulumActDynLinearizedForward PUBLIC ${ct_models_target_include_dirs})
target_link_libraries(InvertedPendulumActDynLinearizedForward ct_rbd)
list(APPEND CT_MODELS_LIBRARIES InvertedPendulumActDynLinearizedForward)


################ HyA #################
if(CPPADCG)
  add_executable(HyALinearizationCodegen src/HyA/codegen/HyALinearizationCodeGen.cpp)
  target_include_directories(HyALinearizationCodegen PUBLIC ${ct_models_target_include_dirs})
  target_link_libraries(HyALinearizationCodegen ct_rbd)
  list(APPEND CT_MODELS_BINARIES HyALinearizationCodegen)
endif()

add_library(HyALinearizedForward include/ct/models/HyA/codegen/HyALinearizedForward.cpp)
target_include_directories(HyALinearizedForward PUBLIC ${ct_models_target_include_dirs})
target_link_libraries(HyALinearizedForward ct_rbd )
list(APPEND CT_MODELS_LIBRARIES HyALinearizedForward)

add_library(HyAJacInverseDynamicsReverse include/ct/models/HyA/codegen/HyAInverseDynJacReverse.cpp)
target_include_directories(HyAJacInverseDynamicsReverse PUBLIC ${ct_models_target_include_dirs})
target_link_libraries(HyAJacInverseDynamicsReverse ct_rbd)
list(APPEND CT_MODELS_LIBRARIES HyAJacInverseDynamicsReverse)

if(BUILD_HYA_LINEARIZATION_TIMINGS)
    add_library(HyALinearizedReverse include/ct/models/HyA/codegen/HyALinearizedReverse.cpp)
    target_include_directories(HyALinearizedReverse PUBLIC ${ct_models_target_include_dirs})
    target_link_libraries(HyALinearizedReverse ct_rbd )
    list(APPEND CT_MODELS_LIBRARIES HyALinearizedReverse)

    add_library(HyAJacInverseDynamicsForward include/ct/models/HyA/codegen/HyAInverseDynJacForward.cpp)
    target_include_directories(HyAJacInverseDynamicsForward PUBLIC ${ct_models_target_include_dirs})
    target_link_libraries(HyAJacInverseDynamicsForward ct_rbd )
    list(APPEND CT_MODELS_LIBRARIES HyAJacInverseDynamicsForward)

    add_executable(HyAcompareForwardReverse src/HyA/codegen/compareForwardReverse.cpp)
    target_include_directories(HyAcompareForwardReverse PUBLIC ${ct_models_target_include_dirs})
    target_link_libraries(HyAcompareForwardReverse
        HyALinearizedForward
        HyALinearizedReverse
        HyAJacInverseDynamicsForward
        HyAJacInverseDynamicsReverse
        ct_rbd
        )
    list(APPEND CT_MODELS_BINARIES HyAcompareForwardReverse)
endif(BUILD_HYA_LINEARIZATION_TIMINGS)


## Declare a cpp library for the ordinary quadrotor
add_library(quadrotorDynamics
    src/Quadrotor/quadrotor_ode.cpp
    src/Quadrotor/A_quadrotor.cpp
    src/Quadrotor/B_quadrotor.cpp
    src/Quadrotor/C_quadrotor.cpp
    )
target_include_directories(quadrotorDynamics PUBLIC ${ct_models_target_include_dirs})
target_link_libraries(quadrotorDynamics ct_core)
list(APPEND CT_MODELS_LIBRARIES quadrotorDynamics)


## Inverse Kinematics

add_library(hya_ik src/HyA/transform6d.cpp)
target_include_directories(hya_ik PUBLIC ${ct_models_target_include_dirs})
set_target_properties(hya_ik PROPERTIES COMPILE_FLAGS "-std=c++98 -fPIC -DIKFAST_NAMESPACE=hya_ik -DIKFAST_NO_MAIN -Wno-unused-variable")
target_link_libraries(hya_ik ct_rbd)
list(APPEND CT_MODELS_LIBRARIES hya_ik)

add_library(irb4600_ik src/Irb4600/transform6d.cpp)
target_include_directories(irb4600_ik PUBLIC ${ct_models_target_include_dirs})
set_target_properties(irb4600_ik PROPERTIES COMPILE_FLAGS "-std=c++98 -fPIC -DIKFAST_NAMESPACE=irb4600_ik -DIKFAST_NO_MAIN -Wno-unused-variable")
target_link_libraries(irb4600_ik ct_rbd)
list(APPEND CT_MODELS_LIBRARIES irb4600_ik)


## convenience library for passing on includes
add_library(ct_models INTERFACE)
target_include_directories(ct_models INTERFACE ${ct_models_target_include_dirs})
target_link_libraries(ct_models INTERFACE
  ct_rbd
  ${CT_MODELS_LIBRARIES}
  )
list(APPEND CT_MODELS_LIBRARIES ct_models)


############
# EXAMPLES #
############
if(BUILD_EXAMPLES)
  add_subdirectory(examples)
endif()


###########
# TESTING #
###########

if(BUILD_TESTS)
    #find_package(GTest QUIET)
    enable_testing()
    add_subdirectory(test)
endif()


#################
# INSTALLATION  #
#################

# for correct libraries locations across platforms
include(GNUInstallDirs)

## copy the header files
install(DIRECTORY include/ct/models DESTINATION include/ct)

## copy the cmake files required for find_package()
install(FILES "cmake/ct_modelsConfig.cmake" DESTINATION "share/ct_models/cmake")

## install library and targets
install(
    TARGETS ${CT_MODELS_LIBRARIES} ${CT_MODELS_BINARIES}
    EXPORT ct_models_export
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_LIBDIR}/ct_models
    )

## create the ct_models.cmake file which holds target includes and dependencies
install (EXPORT ct_models_export DESTINATION share/ct_models/cmake)

## add uninstall target
if(NOT TARGET uninstall)
    configure_file(
        "${CMAKE_CURRENT_SOURCE_DIR}/../ct/cmake/cmake_uninstall.cmake.in"
        "${CMAKE_CURRENT_BINARY_DIR}/../ct/cmake/cmake_uninstall.cmake"
        IMMEDIATE @ONLY)

    add_custom_target(uninstall
        COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/../ct/cmake/cmake_uninstall.cmake)
endif()


#################
# DOCUMENTATION #
#################
add_subdirectory(doc)

```

## Building

It has to be built in release mode: `Release`
```
colcon build --symlink-install --packages-select furuta_pendulum_control_toolbox --cmake-args -DCMAKE_BUILD_TYPE=Release
```
Otherwise I got an error: 
```
waiting 1 second for begin
furuta_pendulum_nloc: /usr/include/eigen3/Eigen/src/Core/Block.h:146: Eigen::Block<XprType, BlockRows, BlockCols, InnerPanel>::Block(XprType&, Eigen::Index, Eigen::Index, Eigen::Index, Eigen::Index) [with XprType = Eigen::Matrix<double, 1, 1, 0, 1, 1>; int BlockRows = 2; int BlockCols = 1; bool InnerPanel = false; Eigen::Index = long int]: Assertion `startRow >= 0 && blockRows >= 0 && startRow <= xpr.rows() - blockRows && startCol >= 0 && blockCols >= 0 && startCol <= xpr.cols() - blockCols' failed.
```