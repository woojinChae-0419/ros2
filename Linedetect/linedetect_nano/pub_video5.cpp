#include "rclcpp/rclcpp.hpp"                  // ROS 2 C++ 클라이언트 라이브러리 (핵심)
#include "sensor_msgs/msg/image.hpp"          // ROS 2 이미지 메시지 타입 정의
#include "cv_bridge/cv_bridge.h"              // ROS 이미지와 OpenCV Mat 간 변환 라이브러리
#include <opencv2/opencv.hpp>                 // OpenCV 라이브러리 (비디오 읽기 및 Mat 사용)
#include <chrono>                             // 시간 관련 라이브러리

using namespace std::chrono_literals;         // 시간 상수(예: 100ms) 사용을 위한 네임스페이스

class VideoPublisher5 : public rclcpp::Node // rclcpp::Node를 상속받아 ROS 2 노드 정의
{
public:
    VideoPublisher5()
    // 생성자 초기화 리스트
    : Node("video_publisher_5"), // **노드 이름 정의**
      cap_(0),                  // cv::VideoCapture 객체를 초기화 (0은 일반적으로 웹캠이지만, 아래에서 경로로 변경됨)
      frame_count_(0)           // 발행한 프레임 카운트 초기화
    {
        // **퍼블리셔 생성**: 'video/image_raw' 토픽으로 sensor_msgs::msg::Image 메시지 발행 (QoS 10)
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("video/image_raw", 10);

        // **비디오 파일 경로 정의**
        const std::string video_path = "/home/linux/simulation/5_lt_cw_100rpm_out.mp4"; 
        
        // **OpenCV VideoCapture로 비디오 파일 열기**
        cap_.open(video_path);

        // **비디오 파일 열기 성공/실패 확인**
        if (!cap_.isOpened()) {
            // 실패 시 ROS 에러 로그 출력
            RCLCPP_ERROR(this->get_logger(), "Error opening video file 5: %s", video_path.c_str());
        } else {
            // 성공 시 ROS 정보 로그 출력
            RCLCPP_INFO(this->get_logger(), "Successfully opened video file 5: %s", video_path.c_str());
        }

        // **타이머 주기 설정**: FPS 기반
        double fps = 30.0; // 목표 발행 속도 30 프레임/초
        // 주기 계산: (1.0 초 / FPS)를 std::chrono::duration 타입으로 변환
        auto period = std::chrono::duration<double>(1.0 / fps);
        
        // **타이머 생성**: 설정된 주기로 timer_callback 함수를 반복 호출
        timer_ = this->create_wall_timer(
            period, std::bind(&VideoPublisher5::timer_callback, this));
    }

private:
    // **타이머 콜백 함수**: 주기적으로 호출되어 비디오 프레임을 읽고 발행
    void timer_callback()
    {
        cv::Mat frame;
        // **비디오에서 다음 프레임 읽기** (cap_ >> frame; 은 cap_.read(frame)과 동일)
        cap_ >> frame; 

        // **프레임 끝 도달 확인**
        if (frame.empty()) {
            RCLCPP_INFO(this->get_logger(), "Video 5 loop end. Rewinding...");
            // **비디오 루프**: 프레임 위치를 0 (처음)으로 설정하여 비디오 반복 재생
            cap_.set(cv::CAP_PROP_POS_FRAMES, 0); 
            // 첫 프레임을 다시 읽어옴
            cap_ >> frame; 
            // 재시작 후에도 프레임이 비어있으면 (예외 상황) 함수 종료
            if (frame.empty()) return;
        }

        // **OpenCV Mat -> ROS Image 메시지로 변환**
        // 헤더: 비어있는 헤더 사용, 인코딩: BGR 3채널(bgr8)
        sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(
            std_msgs::msg::Header(), "bgr8", frame).toImageMsg();

        // **메시지 발행**
        publisher_->publish(*msg);
        frame_count_++; // 발행된 프레임 수 증가
    }

    // **멤버 변수 선언**
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_; // 이미지 메시지 퍼블리셔 객체
    rclcpp::TimerBase::SharedPtr timer_;                             // 주기적으로 콜백을 호출하는 타이머
    cv::VideoCapture cap_;                                          // OpenCV 비디오 캡처 객체
    int frame_count_;                                               // 발행된 프레임 수
};

int main(int argc, char * argv[])
{
    // **ROS 2 시스템 초기화** (필수)
    rclcpp::init(argc, argv);
    
    // **노드 실행**: VideoPublisher5 객체를 생성하고 spin (이벤트 루프 시작)
    rclcpp::spin(std::make_shared<VideoPublisher5>());
    
    // **ROS 2 시스템 종료** (필수)
    rclcpp::shutdown();
    return 0;
}
