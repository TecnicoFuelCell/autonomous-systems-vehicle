cmake_minimum_required(VERSION 3.8)
project(vesc_pub)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(car_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(vesc_node
  src/vesc_node.cpp
)

target_include_directories(vesc_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(vesc_node
  rclcpp
  car_msgs
  std_msgs
)

install(TARGETS vesc_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
