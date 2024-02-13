cmake_minimum_required(VERSION 3.8)
project(furuta_pendulum_model)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  data
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
