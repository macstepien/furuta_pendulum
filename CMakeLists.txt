cmake_minimum_required(VERSION 3.8)
project(furuta_pendulum)

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
rclcpp_components_register_nodes(simulation "furuta_pendulum::SimulationNode")
rclcpp_components_register_node(simulation
  PLUGIN "furuta_pendulum::SimulationNode"
  EXECUTABLE simulation_node
)

install(
  TARGETS simulation
  LIBRARY DESTINATION lib
)

install(DIRECTORY
  config
  launch
  urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
