cmake_minimum_required(VERSION 3.8)
project(joystick_pub)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Encontrar as dependências do ROS 2
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(car_msgs REQUIRED)

# Executável do nó
add_executable(joystick_node src/joystick_reader.cpp)

target_include_directories(joystick_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Lincar dependências
ament_target_dependencies(joystick_node
  rclcpp
  sensor_msgs
  car_msgs
)

# Instalar o executável na pasta de binários do pacote
install(TARGETS joystick_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
