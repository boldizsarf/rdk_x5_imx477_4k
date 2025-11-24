cmake_minimum_required(VERSION 3.5)
project(imx477_dual_camera)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(imx477_dual_camera_node
  src/imx477_dual_camera_node.cpp
)

target_include_directories(imx477_dual_camera_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(imx477_dual_camera_node
  spcdev
)

ament_target_dependencies(imx477_dual_camera_node
  rclcpp
  sensor_msgs
)

install(TARGETS imx477_dual_camera_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
