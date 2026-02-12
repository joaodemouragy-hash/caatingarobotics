cmake_minimum_required(VERSION 3.8)
project(agro_robot_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependências
find_package(ament_cmake REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(xacro REQUIRED)
find_package(gazebo_ros REQUIRED)

# ========================================================
# COMANDO CRÍTICO: INSTALAÇÃO DOS ARQUIVOS
# Adicionamos a pasta 'maps' aqui para o ROS encontrar o mapa salvo
# ========================================================
install(DIRECTORY
  urdf
  launch
  config
  worlds
  maps
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_flake8 REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  set(AMENT_LINT_AUTO_EXCLUDE ament_cmake_flake8)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  # Register flake8 explicitly with a pinned config file to avoid
  # environment-dependent defaults in CI.
  ament_flake8(CONFIG_FILE "${CMAKE_CURRENT_SOURCE_DIR}/ament_flake8.ini")
endif()

ament_package()
