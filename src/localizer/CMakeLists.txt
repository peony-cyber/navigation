cmake_minimum_required(VERSION 3.8)
project(localizer)

# 开启 C++17 支持
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# -----------------------------------------------------------------------------
# 1. 查找依赖项
# -----------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(nav_msgs REQUIRED)

# 查找 PCL (必须使用 COMPONENTS 指定模块)
find_package(PCL REQUIRED COMPONENTS common io filters registration)

# 查找 Eigen3
find_package(Eigen3 REQUIRED)

# -----------------------------------------------------------------------------
# 2. 包含头文件
# -----------------------------------------------------------------------------
# 1. 创建可执行文件
add_executable(localizer_node src/localizer.cpp)

# 2. 包含非 ROS 的第三方头文件 (PCL 和 Eigen)
target_include_directories(localizer_node PRIVATE 
  ${PCL_INCLUDE_DIRS} 
  ${EIGEN3_INCLUDE_DIR}
)

# 3. 链接非 ROS 的第三方库 (PCL)
target_link_libraries(localizer_node
  ${PCL_LIBRARIES}
)

# 4. 关键：使用 ament 宏来链接所有 ROS 2 依赖
# 这会自动处理 rclcpp, sensor_msgs, geometry_msgs, tf2_ros, pcl_conversions
ament_target_dependencies(localizer_node
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2_ros
  tf2_eigen
  pcl_conversions
)

# 5. 安装
install(TARGETS
  localizer_node
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()