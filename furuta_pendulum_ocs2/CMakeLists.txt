cmake_minimum_required(VERSION 3.8)
project(furuta_pendulum_ocs2)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

set(PACKAGE_INCLUDE_DEPENDS
  ament_cmake
  Eigen3
  Boost
  rclcpp
  rclcpp_components
  std_msgs
  sensor_msgs
  ocs2_core
  ocs2_mpc
  ocs2_ddp
  ocs2_slp
  ocs2_sqp
  ocs2_robotic_tools
  hpipm_catkin
)

foreach(Dependency IN ITEMS ${PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_library(ocs2_demo_ros SHARED
  src/DummyFurutaPendulumNode.cpp
  src/FurutaPendulumInterface.cpp
  src/dynamics/FurutaPendulumSystemDynamics.cpp
)

ament_target_dependencies(ocs2_demo_ros 
  ${PACKAGE_INCLUDE_DEPENDS})
rclcpp_components_register_nodes(ocs2_demo_ros
  "furuta_pendulum_ocs2::FurutaPendulumOCS2")
rclcpp_components_register_node(ocs2_demo_ros
  PLUGIN "furuta_pendulum_ocs2::FurutaPendulumOCS2"
  EXECUTABLE ocs2_demo_ros_node
)

install(
  TARGETS ocs2_demo_ros
  LIBRARY DESTINATION lib
)


install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
