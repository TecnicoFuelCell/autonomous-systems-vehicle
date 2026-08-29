cmake_minimum_required(VERSION 3.8)
project(deadman_vesc_direction_imu_mag_pub)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Encontrar as dependências do ROS 2 e de terceiros
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(car_msgs REQUIRED)

# Incluir o diretório de cabeçalhos
include_directories(include)

# Executável do nó a partir do deploy_joystick.cpp
add_executable(deploy_joystick 
  src/deploy_joystick.cpp
  src/deadman_alive.cpp
  src/vesc_parser.cpp
  src/dir_parser.cpp
  src/imu_parser.cpp
  src/mag_parser.cpp
)

# Lincar dependências
ament_target_dependencies(deploy_joystick
  rclcpp
  std_msgs
  sensor_msgs
  example_interfaces
  car_msgs
)

# Lincar a biblioteca yaml-cpp
target_link_libraries(deploy_joystick yaml-cpp)

# Instalar o executável na pasta de binários do pacote
install(TARGETS deploy_joystick
  DESTINATION lib/${PROJECT_NAME}
)

# Instalar a pasta include
install(DIRECTORY include/
  DESTINATION include
)

ament_package()
