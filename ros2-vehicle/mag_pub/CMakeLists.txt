cmake_minimum_required(VERSION 3.8)
project(mag_pub)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)

add_library(mag_component SHARED
  src/mag_node.cpp
)

target_include_directories(mag_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(mag_component
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
)

target_link_libraries(mag_component nlohmann_json::nlohmann_json)

# Loadable into a component container; also generates the standalone mag_node executable.
rclcpp_components_register_node(mag_component
  PLUGIN "mag_pub::MagNode"
  EXECUTABLE mag_node
)

install(TARGETS mag_component
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
