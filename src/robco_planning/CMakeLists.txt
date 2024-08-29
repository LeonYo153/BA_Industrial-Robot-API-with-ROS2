cmake_minimum_required(VERSION 3.8)
project(robco_planning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)


add_executable(robco_planning src/robco_planning.cpp)
add_executable(robco_circle src/robco_circle.cpp)
add_executable(end_speed src/end_speed.cpp)
add_executable(angle_speed src/angle_speed.cpp)

target_include_directories(robco_planning PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(robco_planning PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(
  robco_planning
  "moveit_ros_planning_interface"
  "moveit_visual_tools"
  "rclcpp"
  "tf2"
  "tf2_geometry_msgs"
)

ament_target_dependencies(
  robco_circle
  "moveit_ros_planning_interface"
  "moveit_visual_tools"
  "rclcpp"
  "tf2"
  "tf2_geometry_msgs"
)

ament_target_dependencies(
  end_speed
  "moveit_ros_planning_interface"
  "moveit_visual_tools"
  "rclcpp"
  "tf2"
  "tf2_geometry_msgs"
)

ament_target_dependencies(
  angle_speed
  "moveit_ros_planning_interface"
  "rclcpp"
  "tf2"
  "tf2_geometry_msgs"
)



install(TARGETS robco_planning
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS robco_circle
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS angle_speed
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS end_speed
  DESTINATION lib/${PROJECT_NAME}) 



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
