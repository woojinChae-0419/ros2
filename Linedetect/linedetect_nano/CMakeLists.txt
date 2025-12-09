# CMake를 사용하기 위한 최소 요구 버전 설정 (3.5 버전 지정)
cmake_minimum_required(VERSION 3.5)
# **프로젝트 이름** 설정 (ROS 2 패키지 이름과 일치해야 함)
project(linedetect_nano)

# -----------------------------------------------------------
# C/C++ 표준 버전 설정
# -----------------------------------------------------------
# C 표준이 설정되지 않은 경우 C99로 기본 설정
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# C++ 표준이 설정되지 않은 경우 C++14로 기본 설정
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# GCC 또는 Clang 컴파일러를 사용하는 경우 컴파일 옵션 추가
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # -Wall, -Wextra: 모든 일반적인 경고와 추가적인 경고를 활성화
  # -Wpedantic: C/C++ 표준을 엄격하게 준수하도록 경고 활성화
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# -----------------------------------------------------------
# 필수 패키지 및 라이브러리 찾기 (find dependencies)
# -----------------------------------------------------------

# ROS 2 빌드 시스템(ament) 관련 패키지 찾기
find_package(ament_cmake REQUIRED)
# ROS Client Library C++ 찾기 (ROS 2 노드 개발에 필수)
find_package(rclcpp REQUIRED)
# ROS 센서 메시지 타입 찾기 (Image 메시지 사용에 필수)
find_package(sensor_msgs REQUIRED)
# ROS 메시지 <-> OpenCV Mat 간 변환 라이브러리 찾기
find_package(cv_bridge REQUIRED)
# ROS 표준 메시지 타입 찾기 (std_msgs::msg::Header 사용에 필요)
find_package(std_msgs REQUIRED)
# 이미지 전송 최적화 라이브러리 찾기
find_package(image_transport REQUIRED)

# OpenCV 라이브러리 찾기 (OpenCV 비디오 캡처 및 영상 처리에 필수)
find_package(OpenCV REQUIRED) 

# =======================================================
# 실행 파일 정의 및 의존성 연결
# =======================================================

# [과제 1용 노드] pub_video5.cpp 정의
# 'video_publisher_5'라는 실행 파일 이름 정의 및 소스 파일 지정
add_executable(video_publisher_5 src/pub_video5.cpp)
# ROS 2 및 관련 라이브러리 의존성 연결 (ament_cmake를 통해 자동으로 연결됨)
ament_target_dependencies(video_publisher_5 rclcpp sensor_msgs cv_bridge std_msgs image_transport)
# **OpenCV 라이브러리 연결**: 비디오 캡처 기능을 사용하기 위해 OpenCV 라이브러리를 명시적으로 링크
target_link_libraries(video_publisher_5 ${OpenCV_LIBS})

# [과제 2용 노드] pub_video7.cpp 정의
# 'video_publisher_7'라는 실행 파일 이름 정의 및 소스 파일 지정
add_executable(video_publisher_7 src/pub_video7.cpp)
# ROS 2 및 관련 라이브러리 의존성 연결
ament_target_dependencies(video_publisher_7 rclcpp sensor_msgs cv_bridge std_msgs image_transport)
# **OpenCV 라이브러리 연결**: 비디오 캡처 기능을 사용하기 위해 OpenCV 라이브러리를 명시적으로 링크
target_link_libraries(video_publisher_7 ${OpenCV_LIBS})
# =======================================================

# -----------------------------------------------------------
# 설치 규칙 (install)
# -----------------------------------------------------------
# 빌드된 실행 파일들을 ROS 2 환경에 설치할 위치를 정의
install(TARGETS
  video_publisher_5
  video_publisher_7
  # 설치 경로: ROS 2 패키지의 'lib/linedetect_nano' 디렉토리
  DESTINATION lib/${PROJECT_NAME}
)

# -----------------------------------------------------------
# 테스트 환경 설정 (선택 사항)
# -----------------------------------------------------------
if(BUILD_TESTING)
  # 테스트 코드의 정적 분석(Linting)을 위한 패키지 찾기
  find_package(ament_lint_auto REQUIRED)
  
  # 정적 분석 관련 옵션 주석 처리 (기본 설정 유지)
  #set(ament_cmake_copyright_FOUND TRUE)
  #set(ament_cmake_cpplint_FOUND TRUE)
  
  # 테스트 의존성 찾기 및 연결
  ament_lint_auto_find_test_dependencies()
endif()

# ROS 2 패키지의 빌드 프로세스 완료 선언 (필수)
ament_package()
