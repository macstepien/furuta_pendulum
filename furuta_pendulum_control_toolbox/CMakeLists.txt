cmake_minimum_required(VERSION 3.8)
project(furuta_pendulum_control_toolbox)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(PACKAGE_INCLUDE_DEPENDS
  ament_cmake
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

add_executable(ct_test src/ct_test.cpp)
target_link_libraries(ct_test ct_core)

add_executable(NLOC_MPC src/NLOC_MPC.cpp)
target_link_libraries(NLOC_MPC ct_core ct_rbd ct_models)

add_executable(furuta_pendulum src/furuta_pendulum.cpp)
target_link_libraries(furuta_pendulum ct_core ct_rbd ct_models)

install(
  TARGETS ct_test NLOC_MPC furuta_pendulum
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()
