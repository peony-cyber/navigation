cmake_minimum_required(VERSION 3.5)
project(path_following)

# ROS2 基础包
find_package(ament_cmake REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)           # roscpp -> rclcpp
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
# 外部库
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
# 如果你后面要用 OpenCV，取消下面这行注释
# find_package(OpenCV REQUIRED)

add_executable(path_follower 
  src/PathFollower.cpp
)

# 现代 CMake 用法：给特定 target 加 include 路径
target_include_directories(path_follower PUBLIC
  include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

# ROS2 推荐用 ament_target_dependencies 处理 ROS 包依赖
ament_target_dependencies(path_follower
  rclcpp
  nav_msgs
  geometry_msgs
  sensor_msgs
  tf2_geometry_msgs
  tf2_ros
  tf2
  pcl_conversions
  visualization_msgs
)

# 外部库（PCL、OpenCV）还是用 target_link_libraries
target_link_libraries(path_follower
  ${PCL_LIBRARIES}
  # ${OpenCV_LIBS}  # 如果用 OpenCV 加上这行
)

# 安装目标（ROS2 必须，不然 colcon 找不到可执行文件）
install(TARGETS path_follower
  DESTINATION lib/${PROJECT_NAME}
)

install (DIRECTORY cfg
  DESTINATION share/${PROJECT_NAME}/cfg
) 
install (DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()