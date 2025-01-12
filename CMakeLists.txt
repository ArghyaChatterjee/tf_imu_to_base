cmake_minimum_required(VERSION 3.8)
project(tf_imu_to_base)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Add the executable
add_executable(imu_transform_node src/imu_transform_node.cpp)

# Link against dependencies
ament_target_dependencies(imu_transform_node
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# # Explicitly link tf2_geometry_msgs
# target_link_libraries(imu_transform_node
#   tf2_geometry_msgs
# )

# Install the node
install(TARGETS imu_transform_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install the launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
