cmake_minimum_required(VERSION 3.8)
project(furuta_pendulum_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  dev_utils/record_system_response.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
