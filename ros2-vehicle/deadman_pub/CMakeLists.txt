cmake_minimum_required(VERSION 3.8)
project(deadman_pub)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)

add_library(deadman_component SHARED
  src/deadman_node.cpp
)

target_include_directories(deadman_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(deadman_component
  rclcpp
  rclcpp_components
  std_msgs
)

# Loadable into a component container; also generates the standalone deadman_node executable.
rclcpp_components_register_node(deadman_component
  PLUGIN "deadman_pub::DeadmanNode"
  EXECUTABLE deadman_node
)

install(TARGETS deadman_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
