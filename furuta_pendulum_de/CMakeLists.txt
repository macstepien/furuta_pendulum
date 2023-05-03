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

add_library(de_simulation SHARED
  src/de_simulation_node.cpp)
ament_target_dependencies(de_simulation ${PACKAGE_INCLUDE_DEPENDS})
rclcpp_components_register_nodes(de_simulation "furuta_pendulum_de::DeSimulationNode")
rclcpp_components_register_node(de_simulation
  PLUGIN "furuta_pendulum_de::DeSimulationNode"
  EXECUTABLE de_simulation_node
)

install(
  TARGETS de_simulation
  LIBRARY DESTINATION lib
)

add_library(lqr_with_swing_up_controller SHARED 
  src/lqr_with_swing_up_controller_node.cpp)
ament_target_dependencies(lqr_with_swing_up_controller ${PACKAGE_INCLUDE_DEPENDS})
rclcpp_components_register_nodes(lqr_with_swing_up_controller "furuta_pendulum_de::LqrWithSwingUpControllerNode")
rclcpp_components_register_node(lqr_with_swing_up_controller
  PLUGIN "furuta_pendulum_de::LqrWithSwingUpControllerNode"
  EXECUTABLE lqr_with_swing_up_controller_node
)

install(
  TARGETS lqr_with_swing_up_controller
  LIBRARY DESTINATION lib
)

install(DIRECTORY
  config
  launch
  rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
