# CMake 최소 요구 버전 설정 (ROS 2 Humble/Iron 등에서 일반적으로 사용되는 버전)
cmake_minimum_required(VERSION 3.8) 

# 프로젝트 이름 설정. ROS 2 패키지 이름과 일치해야 함
project(linedetect_wsl)

# C++ 컴파일러가 GCC 또는 Clang 계열일 경우에만 추가 컴파일 옵션을 설정
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # 경고를 활성화하고 엄격한 코딩 표준을 적용하는 옵션
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# -----------------------------------------------------------
# 필수 패키지 및 라이브러리 찾기 (find dependencies)
# -----------------------------------------------------------

# ROS 2 빌드 시스템(ament) 관련 패키지: ROS 2 환경에서 필수
find_package(ament_cmake REQUIRED) 

# ROS Client Library C++: ROS 2 노드 개발에 필수
find_package(rclcpp REQUIRED) 

# ROS 센서 메시지 타입: 이미지 메시지(sensor_msgs/msg/Image) 사용에 필수
find_package(sensor_msgs REQUIRED) 

# OpenCV와 ROS 이미지 메시지 간 변환 라이브러리
find_package(cv_bridge REQUIRED) 

# OpenCV 라이브러리: 영상 처리 기능 사용에 필수
find_package(OpenCV REQUIRED) 

# =======================================================
# 실행 파일 정의 (add_executable) 및 의존성 연결
# =======================================================

# [기존 line_detector 제거]
# add_executable(line_detector src/sub.cpp) 
# 위 라인은 현재 파일에서 제거되었거나 주석 처리되어 실행되지 않음

# 1. 과제 1용 Subscriber 노드 정의 (sub_video5.cpp)
# 'line_detector_5'라는 실행 파일 이름을 정의하고 소스 파일 지정
add_executable(line_detector_5 src/sub_video5.cpp) 
# 정의된 실행 파일에 필요한 ROS 2 및 OpenCV 라이브러리 의존성 연결
ament_target_dependencies(line_detector_5 rclcpp sensor_msgs cv_bridge OpenCV) 

# 2. 과제 2용 Subscriber 노드 정의 (sub_video7.cpp)
# 'line_detector_7'라는 실행 파일 이름을 정의하고 소스 파일 지정
add_executable(line_detector_7 src/sub_video7.cpp)
# 정의된 실행 파일에 필요한 ROS 2 및 OpenCV 라이브러리 의존성 연결
ament_target_dependencies(line_detector_7 rclcpp sensor_msgs cv_bridge OpenCV)
# =======================================================

# -----------------------------------------------------------
# 설치 규칙 (install)
# -----------------------------------------------------------
# 빌드된 실행 파일들을 ROS 2 환경에 설치할 위치를 정의
install(TARGETS
  # 설치 대상 실행 파일 1
  line_detector_5
  # 설치 대상 실행 파일 2
  line_detector_7
  # 설치 경로: ROS 2 패키지의 'lib/linedetect_wsl' 디렉토리
  DESTINATION lib/${PROJECT_NAME} 
)

# -----------------------------------------------------------
# 테스트 환경 설정 (선택 사항)
# -----------------------------------------------------------
if(BUILD_TESTING)
  # 테스트 코드의 정적 분석(Linting)을 위한 패키지 찾기
  find_package(ament_lint_auto REQUIRED)
  # 테스트 의존성 찾기
  ament_lint_auto_find_test_dependencies()
endif()

# ROS 2 패키지의 빌드 프로세스 완료 선언
ament_package()
