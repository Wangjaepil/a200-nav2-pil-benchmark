cmake_minimum_required(VERSION 3.16)
project(a200_predictive_collision_monitor)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(prox_mpc_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PredictiveCollisionState.msg"
  DEPENDENCIES std_msgs
)

add_library(predictive_collision_core STATIC
  src/predictive_collision_core.cpp
)
target_include_directories(predictive_collision_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_library(velocity_filter_core STATIC
  src/velocity_filter_core.cpp
)
target_include_directories(velocity_filter_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(velocity_filter_core
  predictive_collision_core
)

add_library(crossing_yield_core STATIC
  src/crossing_yield_core.cpp
)
target_include_directories(crossing_yield_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(crossing_yield_core
  predictive_collision_core
)

add_executable(predictive_collision_monitor
  src/predictive_collision_monitor_node.cpp
)
target_link_libraries(predictive_collision_monitor
  crossing_yield_core
  predictive_collision_core
  velocity_filter_core
)
ament_target_dependencies(predictive_collision_monitor
  geometry_msgs
  prox_mpc_msgs
  rclcpp
  std_msgs
  tf2
  tf2_ros
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(predictive_collision_monitor "${cpp_typesupport_target}")

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS
    crossing_yield_core
    predictive_collision_core
    predictive_collision_monitor
    velocity_filter_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Uncrustify is the enforced C++ formatter. The package uses short SPDX
  # headers, so the duplicate cpplint/copyright checks are disabled.
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_predictive_collision_core
    test/test_predictive_collision_core.cpp
  )
  target_link_libraries(test_predictive_collision_core
    predictive_collision_core
  )
  target_include_directories(test_predictive_collision_core PRIVATE
    include
  )

  ament_add_gtest(test_velocity_filter_core
    test/test_velocity_filter_core.cpp
  )
  target_link_libraries(test_velocity_filter_core
    velocity_filter_core
  )
  target_include_directories(test_velocity_filter_core PRIVATE
    include
  )

  ament_add_gtest(test_crossing_yield_core
    test/test_crossing_yield_core.cpp
  )
  target_link_libraries(test_crossing_yield_core
    crossing_yield_core
  )
  target_include_directories(test_crossing_yield_core PRIVATE
    include
  )
endif()

ament_export_include_directories(include)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
