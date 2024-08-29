cmake_minimum_required(VERSION 3.8)
project(robco_hw)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(robcomm REQUIRED)
find_package(robco_moveit_config REQUIRED)

# Include directories
include_directories(
  include
  ${robcomm_INCLUDE_DIRS}  # 确保包含robcomm的头文件路径
)

# Add library
add_library(
  robco_hw
  SHARED
  src/robco_hw.cpp
)

# Link libraries
target_link_libraries(
  robco_hw
  ${hardware_interface_LIBRARIES}
  ${rclcpp_LIBRARIES}
  ${rclcpp_lifecycle_LIBRARIES}
  ${robcomm_LIBRARIES}  # 链接robcomm库
  ${robco_moveit_config_LIBRARIES}
)

ament_target_dependencies(
  robco_hw
  hardware_interface
  rclcpp
  rclcpp_lifecycle
  robcomm
  robco_moveit_config
)

# Prevent pluginlib from using boost
target_compile_definitions(robco_hw PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(hardware_interface robco_hw.xml)

# Install targets
install(
  TARGETS robco_hw
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

# Install directories
install(
  DIRECTORY include/
  DESTINATION include
)
install(
  DIRECTORY config launch/
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(robco_hw)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

ament_package()
