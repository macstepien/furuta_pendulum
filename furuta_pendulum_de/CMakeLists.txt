cmake_minimum_required(VERSION 3.8)
project(furuta_pendulum_de)

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
)

foreach(Dependency IN ITEMS ${PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(simulation SHARED 
  src/simulation_node.cpp)
ament_target_dependencies(simulation ${PACKAGE_INCLUDE_DEPENDS})
rclcpp_components_register_nodes(simulation "furuta_pendulum_de::SimulationNode")
rclcpp_components_register_node(simulation
  PLUGIN "furuta_pendulum_de::SimulationNode"
  EXECUTABLE simulation_node
)

install(
  TARGETS simulation
  LIBRARY DESTINATION lib
)

add_library(lqr_controller SHARED 
  src/controller_node.cpp)
ament_target_dependencies(lqr_controller ${PACKAGE_INCLUDE_DEPENDS})
rclcpp_components_register_nodes(lqr_controller "furuta_pendulum_de::ControllerNode")
rclcpp_components_register_node(lqr_controller
  PLUGIN "furuta_pendulum_de::ControllerNode"
  EXECUTABLE lqr_controller_node
)

install(
  TARGETS lqr_controller
  LIBRARY DESTINATION lib
)

install(DIRECTORY
  config
  launch
  rviz
  urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
