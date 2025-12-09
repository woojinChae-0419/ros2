# CMake 최소 요구 버전 설정
# 프로젝트를 빌드하기 위해 필요한 CMake의 최소 버전을 지정
cmake_minimum_required(VERSION 3.8)

# 프로젝트 이름 설정
project(linedetect_wsl)

# GCC 또는 Clang 컴파일러를 사용하는 경우 경고 옵션 추가
# -Wall: 기본적인 모든 경고 표시
# -Wextra: 추가 경고 표시
# -Wpedantic: 표준에 어긋나는 코드 경고
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ============================================================
#                필요한 ROS2 / OpenCV 패키지 찾기
# ============================================================

# ament 빌드 시스템 사용
find_package(ament_cmake REQUIRED)

# ROS2 기본 C++ 노드 기능 패키지
find_package(rclcpp REQUIRED)

# ROS2 이미지 메시지 타입(sensor_msgs/msg/Image)
find_package(sensor_msgs REQUIRED)

# ROS 이미지 <-> OpenCV 이미지 변환 라이브러리(cv_bridge)
find_package(cv_bridge REQUIRED)

# OpenCV 라이브러리
find_package(OpenCV REQUIRED)

# ============================================================
#          기존 line_detector 노드는 사용하지 않음
# ============================================================
# add_executable(line_detector src/sub.cpp)
# → 기존 파일(sub.cpp)을 사용하는 노드는 제거하거나 주석 처리

# ============================================================
#         1. 과제 1용 라인 검출 노드 (sub_video5.cpp)
# ============================================================

# 실행파일(line_detector_5) 생성
# 소스 파일: src/sub_video5.cpp
add_executable(line_detector_5 src/sub_video5.cpp)

# 실행 파일이 필요로 하는 의존 패키지 연결
ament_target_dependencies(
  line_detector_5     # 타겟 이름
  rclcpp              # ROS2 노드 기능
  sensor_msgs         # 이미지 메시지
  cv_bridge           # OpenCV 변환
  OpenCV              # OpenCV 라이브러리
)

# ============================================================
#         2. 과제 2용 라인 검출 노드 (sub_video7.cpp)
# ============================================================

# 실행파일(line_detector_7) 생성
add_executable(line_detector_7 src/sub_video7.cpp)

# 해당 노드에 필요한 패키지 의존성 추가
ament_target_dependencies(
  line_detector_7
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
)

# ============================================================
#                   설치 규칙 (Install Rules)
# ============================================================
# 빌드 후 생성된 실행 파일을
# 설치 경로(lib/<패키지명>)에 배치
install(TARGETS
  line_detector_5
  line_detector_7
  DESTINATION lib/${PROJECT_NAME}
)

# ============================================================
#                   테스트 관련 설정
# ============================================================

# 테스트 옵션(BUILD_TESTING)이 켜져 있는 경우
if(BUILD_TESTING)
  # 코드 스타일 자동 검사(ament_lint)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# ament 패키지 선언 (필수)
ament_package()
