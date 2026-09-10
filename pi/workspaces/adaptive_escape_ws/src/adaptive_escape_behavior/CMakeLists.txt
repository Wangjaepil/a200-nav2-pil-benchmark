cmake_minimum_required(VERSION 3.8)
project(adaptive_escape_behavior)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_behaviors REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pluginlib REQUIRED)
find_package(adaptive_escape_msgs REQUIRED)


# AdaptiveEscape Behavior Plugin
add_library(adaptive_escape_behavior_plugin SHARED
  src/adaptive_escape.cpp
)

target_include_directories(adaptive_escape_behavior_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(adaptive_escape_behavior_plugin
  rclcpp
  rclcpp_lifecycle
  nav2_core
  nav2_behaviors
  nav2_costmap_2d
  nav2_msgs
  nav2_util
  geometry_msgs
  tf2
  tf2_ros
  pluginlib
  adaptive_escape_msgs
)


# pluginlib에 Behavior plugin 등록
pluginlib_export_plugin_description_file(
  nav2_core
  behavior_plugin.xml
)


# Plugin library 설치
install(
  TARGETS adaptive_escape_behavior_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


# Header 설치
install(
  DIRECTORY include/
  DESTINATION include
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()
endif()

ament_package()