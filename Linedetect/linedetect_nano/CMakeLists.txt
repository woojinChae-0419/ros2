# CMake 최소 요구 버전 설정 (ROS2 Foxy 등에서 기본적으로 3.5 이상 사용)
cmake_minimum_required(VERSION 3.5)

# 프로젝트 이름 지정
project(linedetect_nano)

# ============================================================
#                   C / C++ 표준 설정
# ============================================================

# C 언어 표준(C99) 사용
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# C++ 표준(C++14) 사용
# Jetson Nano + ROS2 환경에서는 C++14가 가장 호환성이 좋음
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# GCC 또는 Clang 컴파일러 사용 시 경고 옵션 추가
# -Wall: 일반적인 모든 경고
# -Wextra: 추가적인 경고
# -Wpedantic: 표준 위반 코드 경고
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ============================================================
#                ROS2 / OpenCV 라이브러리 찾기
# ============================================================

# ament 빌드 시스템 필수
find_package(ament_cmake REQUIRED)

# ROS2 rclcpp (C++ 기반 노드)
find_package(rclcpp REQUIRED)

# ROS2 이미지 메시지 타입(sensor_msgs/msg/Image)
find_package(sensor_msgs REQUIRED)

# ROS 이미지 ↔ OpenCV 이미지 변환(cv_bridge)
find_package(cv_bridge REQUIRED)

# 기본 메시지(std_msgs) 사용 — 헤더 포함용
find_package(std_msgs REQUIRED)

# 이미지 전송 기능(image_transport): 성능 개선용
find_package(image_transport REQUIRED)

# OpenCV 라이브러리 찾기
find_package(OpenCV REQUIRED)

# ============================================================
#               과제 1: Video Publisher 5
#     기존 pub.cpp → pub_video5.cpp로 변경된 버전
# ============================================================

# 실행파일 video_publisher_5 생성
# 사용되는 소스 파일: src/pub_video5.cpp
add_executable(video_publisher_5 src/pub_video5.cpp)

# video_publisher_5가 필요한 패키지 연결
ament_target_dependencies(
  video_publisher_5
  rclcpp           # ROS2 C++ 노드
  sensor_msgs      # 이미지 메시지
  cv_bridge        # OpenCV 변환
  std_msgs         # 기본 메시지
  image_transport  # 이미지 퍼블리싱 최적화
)

# OpenCV 라이브러리 링크 연결
target_link_libraries(video_publisher_5 ${OpenCV_LIBS})

# ============================================================
#               과제 2: Video Publisher 7
#           추가된 pub_video7.cpp로 노드 생성
# ============================================================

# 실행파일 video_publisher_7 생성
add_executable(video_publisher_7 src/pub_video7.cpp)

# 의존성 패키지 연결
ament_target_dependencies(
  video_publisher_7
  rclcpp
  sensor_msgs
  cv_bridge
  std_msgs
  image_transport
)

# OpenCV 링크 추가
target_link_libraries(video_publisher_7 ${OpenCV_LIBS})

# ============================================================
#                   설치 규칙 (빌드 후 경로)
# ============================================================

# 빌드 후 실행 파일을 lib/<프로젝트명> 에 설치
install(TARGETS
  video_publisher_5
  video_publisher_7
  DESTINATION lib/${PROJECT_NAME}
)

# ============================================================
#                     테스트 설정
# ============================================================

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # 아래 두 줄은 필요할 때만 사용
  # 라이선스가 없거나 Git repo가 아닐 때를 위한 예외 처리 기능
  #set(ament_cmake_copyright_FOUND TRUE)
  #set(ament_cmake_cpplint_FOUND TRUE)

  # lint(코드 규약 검사) 관련 의존성 자동 추가
  ament_lint_auto_find_test_dependencies()
endif()

# ament 패키지 설정 (필수)
ament_package()
