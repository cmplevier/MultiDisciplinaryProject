cmake_minimum_required(VERSION 3.8)
project(mdp_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY
  launch
  config
  rviz
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  scripts/wait_for_lifecycle_active.py
  DESTINATION share/${PROJECT_NAME}/scripts)

ament_package()
