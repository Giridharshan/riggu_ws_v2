cmake_minimum_required(VERSION 3.16)
project(drivelink_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(controller_manager REQUIRED)
find_package(ament_cmake_gtest QUIET)

add_library(${PROJECT_NAME} SHARED
  src/drivelink_comm_io.cpp
  src/drivelink_comm_rx.cpp
  src/drivelink_comm_registry.cpp
  src/drivelink_system_lifecycle.cpp
  src/drivelink_system_runtime.cpp
  src/drivelink_imu_sensor_lifecycle.cpp
  src/drivelink_imu_sensor_parser.cpp)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

ament_target_dependencies(${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle)

# pluginlib export is kept active so package metadata is complete from task 1.
pluginlib_export_plugin_description_file(hardware_interface drivelink_hardware.xml)

if(BUILD_TESTING)
  add_executable(test_crc16
    test/test_crc16.cpp)
  target_include_directories(test_crc16
    PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/include)

  add_executable(test_comm
    test/test_comm.cpp)
  target_include_directories(test_comm
    PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_link_libraries(test_comm
    ${PROJECT_NAME})

  add_executable(test_plugin_list
    test/test_plugin_list.cpp)
  target_link_libraries(test_plugin_list
    ${PROJECT_NAME})

  add_executable(test_system_lifecycle
    test/test_system_lifecycle.cpp)
  target_link_libraries(test_system_lifecycle
    ${PROJECT_NAME})

  install(TARGETS test_crc16
    RUNTIME DESTINATION lib/${PROJECT_NAME})

  install(TARGETS test_comm
    RUNTIME DESTINATION lib/${PROJECT_NAME})

  install(TARGETS test_plugin_list
    RUNTIME DESTINATION lib/${PROJECT_NAME})

  install(TARGETS test_system_lifecycle
    RUNTIME DESTINATION lib/${PROJECT_NAME})
endif()

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY include/
  DESTINATION include)

install(FILES drivelink_hardware.xml
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS tools/mock_firmware.py
  DESTINATION lib/${PROJECT_NAME}/tools)

install(DIRECTORY config launch urdf
  DESTINATION share/${PROJECT_NAME})

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  controller_manager)

ament_package()
