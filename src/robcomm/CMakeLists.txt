cmake_minimum_required(VERSION 3.8)
project(robcomm VERSION 0.1.0 LANGUAGES CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# specify include directories
include_directories(include)

# 获取源文件和头文件
file(GLOB ROBCOMM_SOURCES src/*.cpp)
file(GLOB ROBCOMM_HEADERS include/robcomm/*.hpp)

# 添加共享库
add_library(robcomm SHARED ${ROBCOMM_SOURCES})

# 设置头文件目录
target_include_directories(robcomm PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# 链接依赖包
ament_target_dependencies(robcomm rclcpp geometry_msgs tf2_ros)

# 安装目标和配置
install(TARGETS robcomm EXPORT robcommConfig DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)

# 包配置
ament_export_include_directories(include)
ament_export_libraries(robcomm)
ament_export_dependencies(rclcpp geometry_msgs tf2_ros)

ament_package()
