cmake_minimum_required(VERSION 3.5)
project(robco_description)
find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)
endif()

install(DIRECTORY meshes urdf DESTINATION share/${PROJECT_NAME})

install(DIRECTORY launch worlds
  DESTINATION share/${PROJECT_NAME}
)


ament_package()
