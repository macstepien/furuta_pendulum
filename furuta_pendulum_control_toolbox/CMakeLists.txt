cmake_minimum_required(VERSION 3.8)
project(furuta_pendulum_control_toolbox)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(PACKAGE_INCLUDE_DEPENDS
  ament_cmake
  Eigen3
  rclcpp
  rclcpp_components
  std_msgs
  sensor_msgs
  ct_core
  ct_optcon
  ct_rbd
  ct_models
)

foreach(Dependency IN ITEMS ${PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

find_package(blasfeo QUIET)
find_package(hpipm QUIET)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${blasfeo_INCLUDE_DIRS}
  ${hpipm_INCLUDE_DIRS}
)

add_executable(ct_demo_simulation_mpc src/ct_demo_simulation_mpc.cpp)
ament_target_dependencies(ct_demo_simulation_mpc ament_index_cpp)
target_link_libraries(ct_demo_simulation_mpc ct_core ct_rbd ct_models)

add_executable(ct_lqr src/ct_lqr.cpp)
ament_target_dependencies(ct_lqr ament_index_cpp)
target_link_libraries(ct_lqr ct_core ct_rbd ct_models)

add_executable(ct_demo_simulation src/ct_demo_simulation.cpp)
target_link_libraries(ct_demo_simulation ct_core ct_rbd)

install(
  TARGETS ct_demo_simulation_mpc ct_lqr ct_demo_simulation
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

add_library(ct_demo_simulation_ros SHARED 
  src/ct_demo_simulation_node.cpp)

ament_target_dependencies(ct_demo_simulation_ros 
  ${PACKAGE_INCLUDE_DEPENDS} ct_core ct_rbd)
rclcpp_components_register_nodes(ct_demo_simulation_ros
  "furuta_pendulum_control_toolbox::CtSimulationNode")
rclcpp_components_register_node(ct_demo_simulation_ros
  PLUGIN "furuta_pendulum_control_toolbox::CtSimulationNode"
  EXECUTABLE ct_demo_simulation_node
)

install(
  TARGETS ct_demo_simulation_ros
  LIBRARY DESTINATION lib
)

add_library(ct_mpc_controller SHARED 
  src/ct_mpc_controller_node.cpp)
ament_target_dependencies(ct_mpc_controller 
  ${PACKAGE_INCLUDE_DEPENDS} ct_core ct_rbd)
rclcpp_components_register_nodes(ct_mpc_controller
  "furuta_pendulum_control_toolbox::CtMPCControllerNode")
rclcpp_components_register_node(ct_mpc_controller
  PLUGIN "furuta_pendulum_control_toolbox::CtMPCControllerNode"
  EXECUTABLE ct_mpc_controller_node
)
install(
  TARGETS ct_mpc_controller
  LIBRARY DESTINATION lib
)

add_executable(ct_demo_simulation_mpc_node src/ct_demo_simulation_mpc_node.cpp)
ament_target_dependencies(ct_demo_simulation_mpc_node ${PACKAGE_INCLUDE_DEPENDS} ct_core ct_rbd ct_models)

install(
  TARGETS ct_demo_simulation_mpc_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
