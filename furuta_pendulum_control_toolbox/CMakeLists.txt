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

# add_executable(ct_test src/ct_test.cpp)
# target_link_libraries(ct_test ct_core)

# add_executable(NLOC_MPC src/NLOC_MPC.cpp)
# target_link_libraries(NLOC_MPC ct_core ct_rbd ct_models)

add_executable(furuta_pendulum_nloc src/furuta_pendulum_nloc.cpp)
target_link_libraries(furuta_pendulum_nloc ct_core ct_rbd ct_models)

add_executable(furuta_pendulum_lqr src/furuta_pendulum_lqr.cpp)
target_link_libraries(furuta_pendulum_lqr ct_core ct_rbd ct_models)

add_executable(simulate_furuta_pendulum src/simulate_furuta_pendulum.cpp)
target_link_libraries(simulate_furuta_pendulum ct_core ct_rbd)

install(
  TARGETS simulate_furuta_pendulum furuta_pendulum_lqr furuta_pendulum_nloc
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

add_library(simulate_furuta_pendulum_ros SHARED 
  src/simulate_furuta_pendulum_ros_node.cpp)

ament_target_dependencies(simulate_furuta_pendulum_ros 
  ${PACKAGE_INCLUDE_DEPENDS} ct_core ct_rbd)
rclcpp_components_register_nodes(simulate_furuta_pendulum_ros
  "furuta_pendulum_control_toolbox::CtSimulationNode")
rclcpp_components_register_node(simulate_furuta_pendulum_ros
  PLUGIN "furuta_pendulum_control_toolbox::CtSimulationNode"
  EXECUTABLE simulate_furuta_pendulum_ros_node
)

install(
  TARGETS simulate_furuta_pendulum_ros
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

add_executable(furuta_pendulum_nloc_ros_node src/furuta_pendulum_nloc_ros_node.cpp)
ament_target_dependencies(furuta_pendulum_nloc_ros_node ${PACKAGE_INCLUDE_DEPENDS} ct_core ct_rbd ct_models)

install(
  TARGETS furuta_pendulum_nloc_ros_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
