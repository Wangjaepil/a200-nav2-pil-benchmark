cmake_minimum_required(VERSION 3.8)
project(adaptive_escape_bt)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(adaptive_escape_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)


# AdaptiveEscape BT Node library
add_library(adaptive_escape_bt_node SHARED
  src/adaptive_escape_action.cpp
)

target_compile_definitions(
  adaptive_escape_bt_node
  PRIVATE BT_PLUGIN_EXPORT
)

target_include_directories(adaptive_escape_bt_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(adaptive_escape_bt_node
  rclcpp
  nav2_behavior_tree
  behaviortree_cpp
  adaptive_escape_msgs
  geometry_msgs
  nav_msgs
)


# shared library 설치
install(
  TARGETS adaptive_escape_bt_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


# header 설치
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