cmake_minimum_required(VERSION 3.16)
project(pub1_3)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(pub1_3 src/pub1_3.cpp)
ament_target_dependencies(pub1_3 rclcpp geometry_msgs)

install(TARGETS
  pub1_3
  DESTINATION lib/${PROJECT_NAME})

ament_package()
