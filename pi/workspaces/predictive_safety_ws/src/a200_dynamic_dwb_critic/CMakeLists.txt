cmake_minimum_required(VERSION 3.8)
project(a200_dynamic_dwb_critic)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(dwb_core REQUIRED)
find_package(dwb_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_2d_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(pluginlib REQUIRED)
find_package(prox_mpc_msgs REQUIRED)
find_package(rclcpp REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/dynamic_obstacle_critic.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  dwb_core
  dwb_msgs
  geometry_msgs
  nav_2d_msgs
  nav2_costmap_2d
  nav2_util
  pluginlib
  prox_mpc_msgs
  rclcpp
)

pluginlib_export_plugin_description_file(dwb_core plugins.xml)

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include/
)

install(
  FILES plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  dwb_core
  dwb_msgs
  geometry_msgs
  nav_2d_msgs
  nav2_costmap_2d
  nav2_util
  pluginlib
  prox_mpc_msgs
  rclcpp
)

ament_package()
