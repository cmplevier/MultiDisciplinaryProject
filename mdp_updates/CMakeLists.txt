cmake_minimum_required(VERSION 3.8)
project(mdp_updates)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(std_srvs REQUIRED)

ament_get_python_install_dir(PYTHON_INSTALL_DIR)
_ament_cmake_python_register_environment_hook()

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.py"
)

install(DIRECTORY
  loop
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.py"
)

install(DIRECTORY
  loop/
  DESTINATION ${PYTHON_INSTALL_DIR}/${PROJECT_NAME}/loop
  FILES_MATCHING PATTERN "*.py"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
