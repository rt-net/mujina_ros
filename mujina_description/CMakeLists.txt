cmake_minimum_required(VERSION 3.8)
project(mujina_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  meshes
  urdf
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE) # Skip copyright checks
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
