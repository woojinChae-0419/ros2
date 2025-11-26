cmake_minimum_required(VERSION 3.16)
project(camera2_examples)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(pub src/pub.cpp)
ament_target_dependencies(pub rclcpp sensor_msgs cv_bridge OpenCV)

add_executable(sub1 src/sub_camera2_1.cpp)
ament_target_dependencies(sub1 rclcpp sensor_msgs OpenCV)

add_executable(sub_rec src/sub_camera2_2.cpp)
ament_target_dependencies(sub_rec rclcpp sensor_msgs OpenCV)

add_executable(pub_class src/pmain.cpp src/pub.cpp)
ament_target_dependencies(pub_class rclcpp sensor_msgs cv_bridge OpenCV)

add_executable(sub_class src/smain.cpp src/sub.cpp)
ament_target_dependencies(sub_class rclcpp sensor_msgs OpenCV)

install(TARGETS
  pub sub1 sub_rec pub_class sub_class
  DESTINATION lib/${PROJECT_NAME})

ament_package()
