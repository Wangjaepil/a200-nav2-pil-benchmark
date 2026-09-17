cmake_minimum_required(VERSION 3.8)
project(a200_dynamic_scan_filter)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(prox_mpc_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(dynamic_scan_filter_core STATIC
  src/dynamic_scan_filter_core.cpp
)
set_target_properties(dynamic_scan_filter_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

target_include_directories(dynamic_scan_filter_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(dynamic_scan_filter
  src/dynamic_scan_filter_node.cpp
)

target_include_directories(dynamic_scan_filter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(dynamic_scan_filter
  dynamic_scan_filter_core
)

ament_target_dependencies(dynamic_scan_filter
  builtin_interfaces
  geometry_msgs
  prox_mpc_msgs
  rclcpp
  sensor_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

install(
  TARGETS
    dynamic_scan_filter_core
    dynamic_scan_filter
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include/
)

install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

if(BUILD_TESTING)
  add_executable(dynamic_scan_filter_core_standalone_test
    test/test_dynamic_scan_filter_core_standalone.cpp
  )
  target_link_libraries(
    dynamic_scan_filter_core_standalone_test
    dynamic_scan_filter_core
  )
  add_test(
    NAME dynamic_scan_filter_core_standalone_test
    COMMAND dynamic_scan_filter_core_standalone_test
  )

  add_executable(dynamic_scan_filter_motion_stress_test
    test/test_motion_gate_stress.cpp
  )
  target_link_libraries(
    dynamic_scan_filter_motion_stress_test
    dynamic_scan_filter_core
  )
  add_test(
    NAME dynamic_scan_filter_motion_stress_test
    COMMAND dynamic_scan_filter_motion_stress_test
  )
endif()

ament_export_include_directories(include)
ament_export_libraries(dynamic_scan_filter_core)

ament_package()
