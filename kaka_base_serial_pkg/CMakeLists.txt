cmake_minimum_required(VERSION 2.8.3)
project(kaka_base_serial_pkg)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  serial
  std_msgs
)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS} "/usr/include/eigen3/")

add_dependencies(${catkin_EXPORTED_TARGETS})

add_executable(kaka_base_serial_node src/kaka_base_serial.cpp src/kaka_base_serial.h)

target_link_libraries(kaka_base_serial_node ${catkin_LIBRARIES})
