cmake_minimum_required(VERSION 3.8)
project(uart_reader)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)

add_library(uart_reader_component SHARED
  src/uart_reader_node.cpp
)

target_include_directories(uart_reader_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(uart_reader_component
  rclcpp
  rclcpp_components
  std_msgs
)

# Loadable into a component container; also generates the standalone uart_reader_node executable.
rclcpp_components_register_node(uart_reader_component
  PLUGIN "uart_reader::UartReaderNode"
  EXECUTABLE uart_reader_node
)

install(TARGETS uart_reader_component
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
