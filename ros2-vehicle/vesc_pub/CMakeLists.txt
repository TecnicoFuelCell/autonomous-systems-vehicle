cmake_minimum_required(VERSION 3.8)
project(vesc_pub)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(car_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_library(vesc_component SHARED
  src/vesc_node.cpp
)

target_include_directories(vesc_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(vesc_component
  rclcpp
  rclcpp_components
  car_msgs
  std_msgs
)

# Loadable into a component container; also generates the standalone vesc_node executable.
rclcpp_components_register_node(vesc_component
  PLUGIN "vesc_pub::VesceNode"
  EXECUTABLE vesc_node
)

install(TARGETS vesc_component
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
