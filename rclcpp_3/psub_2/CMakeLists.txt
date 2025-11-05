cmake_minimum_required(VERSION 3.5)
project(psub_2)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(pub src/pub.cpp)
ament_target_dependencies(pub rclcpp std_msgs)

add_executable(sub src/sub.cpp)
ament_target_dependencies(sub rclcpp std_msgs)

install(TARGETS
  pub
  sub
  DESTINATION lib/${PROJECT_NAME})

ament_package()
