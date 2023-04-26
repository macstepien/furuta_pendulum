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

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(ct_furuta_pendulum_nloc src/ct_furuta_pendulum_nloc.cpp)
ament_target_dependencies(ct_furuta_pendulum_nloc ament_index_cpp)
target_link_libraries(ct_furuta_pendulum_nloc ct_core ct_rbd ct_models)

add_executable(ct_furuta_pendulum_lqr src/ct_furuta_pendulum_lqr.cpp)
ament_target_dependencies(ct_furuta_pendulum_lqr ament_index_cpp)
target_link_libraries(ct_furuta_pendulum_lqr ct_core ct_rbd ct_models)

add_executable(ct_furuta_pendulum_simulation src/ct_furuta_pendulum_simulation.cpp)
target_link_libraries(ct_furuta_pendulum_simulation ct_core ct_rbd)

install(
  TARGETS ct_furuta_pendulum_simulation ct_furuta_pendulum_lqr ct_furuta_pendulum_nloc
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

add_library(ct_furuta_pendulum_simulation_ros SHARED 
  src/ct_furuta_pendulum_simulation_ros_node.cpp)

ament_target_dependencies(ct_furuta_pendulum_simulation_ros 
  ${PACKAGE_INCLUDE_DEPENDS} ct_core ct_rbd)
rclcpp_components_register_nodes(ct_furuta_pendulum_simulation_ros
  "furuta_pendulum_control_toolbox::CtSimulationNode")
rclcpp_components_register_node(ct_furuta_pendulum_simulation_ros
  PLUGIN "furuta_pendulum_control_toolbox::CtSimulationNode"
  EXECUTABLE ct_furuta_pendulum_simulation_ros_node
)

install(
  TARGETS ct_furuta_pendulum_simulation_ros
  LIBRARY DESTINATION lib
)

add_library(controller SHARED 
  src/controller_node.cpp)
ament_target_dependencies(controller 
  ${PACKAGE_INCLUDE_DEPENDS} ct_core ct_rbd)
rclcpp_components_register_nodes(controller
  "furuta_pendulum_control_toolbox::CtControllerNode")
rclcpp_components_register_node(controller
  PLUGIN "furuta_pendulum_control_toolbox::CtControllerNode"
  EXECUTABLE controller_node
)
install(
  TARGETS controller
  LIBRARY DESTINATION lib
)

add_executable(ct_furuta_pendulum_nloc_ros_node src/ct_furuta_pendulum_nloc_ros_node.cpp)
ament_target_dependencies(ct_furuta_pendulum_nloc_ros_node ${PACKAGE_INCLUDE_DEPENDS} ct_core ct_rbd ct_models)

install(
  TARGETS ct_furuta_pendulum_nloc_ros_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
