cmake_minimum_required(VERSION 3.8)
project(furuta_pendulum_ocs2_ros)

# Generate compile_commands.json for clang tools
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(AMENT_PACKAGE_DEPENDENCIES
  ament_cmake
  rclcpp
  rclcpp_components
  ocs2_core
  ocs2_ddp
  ocs2_mpc
  ocs2_ros_interfaces
  ocs2_robotic_tools
  furuta_pendulum_ocs2
)


foreach(Dependency IN ITEMS ${AMENT_PACKAGE_DEPENDENCIES})
  find_package(${Dependency} REQUIRED)
endforeach()

find_package(Boost REQUIRED COMPONENTS
  system
  filesystem
)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

###########
## Build ##
###########

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

# Mpc node
add_executable(furuta_pendulum_mpc
  src/FurutaPendulumMpcNode.cpp
)
# add_dependencies(furuta_pendulum_mpc
# )
ament_target_dependencies(furuta_pendulum_mpc rclcpp furuta_pendulum_ocs2 ocs2_ros_interfaces ocs2_mpc ocs2_ddp
)
# target_link_libraries(furuta_pendulum_mpc ${LIBS}
# )

# Dummy node
add_executable(furuta_pendulum_dummy_test
  src/DummyFurutaPendulumNode.cpp
  src/FurutaPendulumDummyVisualization.cpp
)
# add_dependencies(furuta_pendulum_dummy_test
# )
ament_target_dependencies(furuta_pendulum_dummy_test
rclcpp furuta_pendulum_ocs2 ocs2_ros_interfaces ocs2_mpc ocs2_ddp
)
# target_link_libraries(furuta_pendulum_dummy_test ${LIBS}
# )

add_executable(furuta_pendulum_mpc_mrt
  src/FurutaPendulumMpcMrtNode.cpp
  src/FurutaPendulumDummyVisualization.cpp
)
ament_target_dependencies(furuta_pendulum_mpc_mrt rclcpp furuta_pendulum_ocs2 ocs2_ros_interfaces ocs2_mpc ocs2_ddp
)

add_library(ocs2_test_simulation SHARED 
  src/FurutaPendulumSimulateTestNode.cpp)
ament_target_dependencies(ocs2_test_simulation rclcpp furuta_pendulum_ocs2 ocs2_core sensor_msgs rclcpp_components)
rclcpp_components_register_nodes(ocs2_test_simulation
  "furuta_pendulum_ocs2_ros::OCS2SimulationNode")
rclcpp_components_register_node(ocs2_test_simulation
  PLUGIN "furuta_pendulum_ocs2_ros::OCS2SimulationNode"
  EXECUTABLE ocs2_test_simulation_node
)
install(
  TARGETS ocs2_test_simulation
  LIBRARY DESTINATION lib
)

# TODO rclcpp_components multithreaded executor
# add_library(ocs2_mpc_controller SHARED 
#   src/FurutaPendulumMpcMrtControllerNode.cpp
#   src/FurutaPendulumDummyVisualization.cpp)
# ament_target_dependencies(ocs2_mpc_controller rclcpp furuta_pendulum_ocs2 ocs2_ros_interfaces ocs2_mpc ocs2_ddp rclcpp_components)
# rclcpp_components_register_nodes(ocs2_mpc_controller
#   "furuta_pendulum_ocs2_ros::OCS2MPCControllerNode")
# rclcpp_components_register_node(ocs2_mpc_controller
#   PLUGIN "furuta_pendulum_ocs2_ros::OCS2MPCControllerNode"
#   EXECUTABLE ocs2_mpc_controller_node
# )
# install(
#   TARGETS ocs2_mpc_controller
#   LIBRARY DESTINATION lib
# )

add_executable(ocs2_mpc_controller_node
  src/FurutaPendulumMpcMrtControllerNode.cpp
  src/FurutaPendulumDummyVisualization.cpp)
ament_target_dependencies(ocs2_mpc_controller_node rclcpp furuta_pendulum_ocs2 ocs2_ros_interfaces ocs2_mpc ocs2_ddp)
install(TARGETS ocs2_mpc_controller_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

#########################
###   CLANG TOOLING   ###
#########################
find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
  message(STATUS "Running clang tooling.")
  add_clang_tooling(
    TARGETS
        furuta_pendulum_mpc
        furuta_pendulum_dummy_test
    SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include
    CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
    CT_HEADER_EXCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/generated
    CF_WERROR
  )
endif(cmake_clang_tools_FOUND)

#############
## Install ##
#############

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)
install(TARGETS furuta_pendulum_mpc furuta_pendulum_dummy_test furuta_pendulum_mpc_mrt
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch rviz config
  DESTINATION share/${PROJECT_NAME}
)

###################################
## ament specific configuration ##
###################################

ament_export_dependencies(Boost ${AMENT_PACKAGE_DEPENDENCIES})

ament_export_include_directories(include
  ${EIGEN3_INCLUDE_DIRS})

ament_export_libraries(${PROJECT_NAME})

ament_package()