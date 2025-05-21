cmake_minimum_required(VERSION 3.8)
project(laser_to_pointcloud)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(laser_to_pointcloud_node src/laser_to_pointcloud_node.cpp)
target_include_directories(laser_to_pointcloud_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(laser_to_pointcloud_node PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  laser_to_pointcloud_node
  "rclcpp"
  "sensor_msgs"
  "tf2"
  "tf2_ros"
)

install(TARGETS laser_to_pointcloud_node
  DESTINATION lib/${PROJECT_NAME})

ament_package() 