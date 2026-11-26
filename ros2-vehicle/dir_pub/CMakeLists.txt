cmake_minimum_required(VERSION 3.8)
project(dir_pub)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(car_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_library(dir_component SHARED
  src/dir_node.cpp
)

target_include_directories(dir_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(dir_component
  rclcpp
  rclcpp_components
  car_msgs
  std_msgs
)

# Loadable into a component container; also generates the standalone dir_node executable.
rclcpp_components_register_node(dir_component
  PLUGIN "dir_pub::DirNode"
  EXECUTABLE dir_node
)

install(TARGETS dir_component
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
