cmake_minimum_required(VERSION 3.16)
project(ekf_ground_truth)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)

add_executable(gazebo_ground_truth_node src/gazebo_ground_truth_node.cpp)
target_compile_features(gazebo_ground_truth_node PUBLIC cxx_std_17)
ament_target_dependencies(gazebo_ground_truth_node rclcpp geometry_msgs)
target_link_libraries(gazebo_ground_truth_node
  gz-transport13::gz-transport13
  gz-msgs10::gz-msgs10
)

install(TARGETS gazebo_ground_truth_node
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
