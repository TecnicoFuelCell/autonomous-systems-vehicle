cmake_minimum_required(VERSION 3.8)
project(imu_pub)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_library(imu_component SHARED
  src/imu_node.cpp
)

target_include_directories(imu_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(imu_component
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
  geometry_msgs
)

# Loadable into a component container; also generates the standalone imu_node executable.
rclcpp_components_register_node(imu_component
  PLUGIN "imu_pub::ImuNode"
  EXECUTABLE imu_node
)

install(TARGETS imu_component
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
