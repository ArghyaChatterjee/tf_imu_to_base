cmake_minimum_required(VERSION 3.0.2)
project(tf_imu_to_base)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  tf
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(imu_transform_node src/imu_transform_node.cpp)
target_link_libraries(imu_transform_node ${catkin_LIBRARIES})
