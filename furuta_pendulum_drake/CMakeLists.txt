cmake_minimum_required(VERSION 3.8)
project(furuta_pendulum_drake)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(PACKAGE_INCLUDE_DEPENDS
  ament_cmake
  ament_index_cpp
)

foreach(Dependency IN ITEMS ${PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

find_package(drake CONFIG REQUIRED PATHS /opt/drake)

add_executable(furuta_pendulum_urdf_lqr src/furuta_pendulum_urdf_lqr.cpp)
ament_target_dependencies(furuta_pendulum_urdf_lqr ament_index_cpp)
target_link_libraries(furuta_pendulum_urdf_lqr drake::drake)

install(
  TARGETS furuta_pendulum_urdf_lqr
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()