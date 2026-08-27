cmake_minimum_required(VERSION 3.8)
project(odom_pub)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Encontrar as dependências do ROS 2 e de sistema
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(wechat REQUIRED)

# Executável do nó
add_executable(odom_node src/odom_node.cpp)

# Incluir diretórios de inclusão do Eigen e dependências
target_include_directories(odom_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)

# Lincar dependências
ament_target_dependencies(odom_node
  rclcpp
  std_msgs
  nav_msgs
  geometry_msgs
  tf2_ros
  wechat
)

# Instalar o executável na pasta de binários do pacote
install(TARGETS odom_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_active_tests()
endif()

ament_package()
