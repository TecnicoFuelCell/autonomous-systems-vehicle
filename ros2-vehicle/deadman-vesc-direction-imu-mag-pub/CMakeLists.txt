cmake_minimum_required(VERSION 3.8)
project(Deadman-Vesc-Direction-IMU-Mag-Pub)

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
  # Adiciona src/uart_utils.cpp se tiveres a implementação num ficheiro separado
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

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_active_tests()
endif()

ament_package()
