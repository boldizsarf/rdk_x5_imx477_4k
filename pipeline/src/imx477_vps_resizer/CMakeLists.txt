cmake_minimum_required(VERSION 3.5)
project(imx477_vps_resizer)

# Use at least C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(vps_resizer_node
  src/vps_resizer_node.cpp
)

ament_target_dependencies(vps_resizer_node
  rclcpp
  sensor_msgs
)

# D-Robotics hardver lib
target_link_libraries(vps_resizer_node
  spcdev
)

install(TARGETS
  vps_resizer_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

