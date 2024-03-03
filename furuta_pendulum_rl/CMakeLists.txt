cmake_minimum_required(VERSION 3.8)
project(furuta_pendulum_rl)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(PACKAGE_INCLUDE_DEPENDS
  ament_cmake
  rcl
  rclcpp
  rclcpp_components
  rcutils
  std_msgs
  sensor_msgs
)

foreach(Dependency IN ITEMS ${PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

find_package(Torch REQUIRED)

include_directories(
  include
)

add_library(rl_controller SHARED 
  src/rl_controller_node.cpp)
ament_target_dependencies(rl_controller ${PACKAGE_INCLUDE_DEPENDS})
rclcpp_components_register_nodes(rl_controller "furuta_pendulum_rl::RlControllerNode")
rclcpp_components_register_node(rl_controller
PLUGIN "furuta_pendulum_rl::RlControllerNode"
EXECUTABLE rl_controller_node
)
target_link_libraries(rl_controller ${TORCH_LIBRARIES})

install(
  TARGETS rl_controller
  LIBRARY DESTINATION lib
)

install(DIRECTORY
  config  
  launch
  model
  trained_agents
  DESTINATION share/${PROJECT_NAME}
)

ament_package()