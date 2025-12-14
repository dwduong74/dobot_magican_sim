cmake_minimum_required(VERSION 3.5)
project(dobot_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch meshes rviz model config dobot_description
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS dobot_description/kinematic_supervisor.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
