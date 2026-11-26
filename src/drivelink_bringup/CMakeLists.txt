cmake_minimum_required(VERSION 3.16)
project(drivelink_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config urdf
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  scripts/sensor_validator.py
  scripts/bno055_pi.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
