cmake_minimum_required(VERSION 3.8)
project(deadman_actuator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(car_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(deadman_actuator
  src/deadman_actuator.cpp
)

target_include_directories(deadman_actuator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(deadman_actuator
  rclcpp
  car_msgs
  std_msgs
)

install(TARGETS deadman_actuator
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
