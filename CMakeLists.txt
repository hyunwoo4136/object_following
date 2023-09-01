cmake_minimum_required(VERSION 2.8.3)
project(object_following)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(object_following src/object_following.cpp)

target_link_libraries(object_following
  ${catkin_LIBRARIES}
)

