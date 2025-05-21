cmake_minimum_required(VERSION 3.8)
project(safety_limiter)

# 默认使用C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找必需的依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(nav_2d_msgs REQUIRED)
find_package(nav_2d_utils REQUIRED)

# 查找Eigen3
find_package(Eigen3 REQUIRED)

# 生成消息
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Status.msg"
  DEPENDENCIES std_msgs
)

# 包含目录
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

# 构建可执行文件
add_executable(safe_limiter src/safety_limiter.cpp)
ament_target_dependencies(safe_limiter
  rclcpp
  nav_msgs
  geometry_msgs
  tf2_ros
  tf2
  sensor_msgs
  laser_geometry
  nav_2d_msgs
  nav_2d_utils
)
rosidl_get_typesupport_target(cpp_typesupport_target_limiter "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(safe_limiter "${cpp_typesupport_target_limiter}")

# 安装目标
install(TARGETS
  safe_limiter
  DESTINATION lib/${PROJECT_NAME}
)

# 安装启动文件、配置文件等（如果有的话）
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

# 导出依赖
ament_export_dependencies(
  rclcpp
  nav_msgs
  geometry_msgs
  tf2_ros
  sensor_msgs
  laser_geometry
  nav_2d_msgs
  nav_2d_utils
)

ament_export_include_directories(include)
ament_package()
