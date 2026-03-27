cmake_minimum_required(VERSION 3.8)
project(goal_pub)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# 创建可执行节点
add_executable(multigoal_pub_node src/multigoal_pub.cpp)

target_include_directories(multigoal_pub_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(multigoal_pub_node
  rclcpp
  geometry_msgs
  std_msgs
  tf2
  tf2_ros
)

install(TARGETS multigoal_pub_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY cfg
  DESTINATION share/${PROJECT_NAME}
)

ament_package()