cmake_minimum_required(VERSION 3.8)
project(detect_obs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(livox_ros_driver2 REQUIRED)

# 创建可执行节点
add_executable(obstacle_processor src/obstacle_processor.cpp)

target_include_directories(obstacle_processor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(obstacle_processor
  rclcpp
  geometry_msgs
  sensor_msgs
  nav_msgs
  pcl_conversions
  PCL
  livox_ros_driver2
)


install(TARGETS obstacle_processor
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()