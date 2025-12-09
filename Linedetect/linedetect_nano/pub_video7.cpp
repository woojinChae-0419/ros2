#include "rclcpp/rclcpp.hpp"            // ROS2의 핵심 기능(노드, 로그, 스핀 등)을 사용하기 위한 헤더
#include "sensor_msgs/msg/image.hpp"   // ROS2 이미지 메시지 타입 정의 (sensor_msgs::msg::Image)
#include "cv_bridge/cv_bridge.h"       // ROS 이미지 <-> OpenCV cv::Mat 변환을 돕는 라이브러리
#include <opencv2/opencv.hpp>          // OpenCV의 모든 기능(영상 읽기/처리/표시 등)
#include <chrono>                      // 시간 관련(타이머, 시간 측정 등)에 사용

using namespace std::chrono_literals;  // 100ms 같은 리터럴(suffix)을 편하게 쓰기 위해

// -------------------------------------------------------------
// VideoPublisher7 클래스: 비디오 파일을 읽어 ROS 토픽으로 퍼블리시하는 노드
//  - 이름 규칙: 클래스명 VideoPublisher7, 노드이름 "video_publisher_7"
//  - 실제 카메라 대신 특정 비디오 파일을 "가짜 센서"처럼 사용하려는 목적
// -------------------------------------------------------------
class VideoPublisher7 : public rclcpp::Node
{
public:
    // 생성자: 노드 초기화, 퍼블리셔 생성, 비디오 파일 열기, 타이머 설정
    VideoPublisher7()
    : Node("video_publisher_7"), cap_(0), frame_count_(0)
    {
        // -------------------------------
        // 1) 퍼블리셔 생성
        //    - 토픽명: "video/image_raw"
        //    - 메시지 타입: sensor_msgs::msg::Image
        //    - 큐 사이즈: 10 (QoS에서 내부 버퍼 크기와 유사한 개념)
        // -------------------------------
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("video/image_raw", 10);

        // -------------------------------
        // 2) 재생할 비디오 파일 경로 지정 (절대 경로 권장)
        //    - 사용 환경에 맞게 경로를 변경해야 함
        // -------------------------------
        const std::string video_path = "/home/linux/simulation/7_lt_ccw_100rpm_in.mp4"; 

        // -------------------------------
        // 3) OpenCV VideoCapture로 파일 열기
        //    - cap_.open(path): 파일이 존재하고 코덱이 지원되면 true 반환
        // -------------------------------
        cap_.open(video_path);

        // -------------------------------
        // 4) 파일 열기 성공 여부 확인 및 로그 출력
        //    - 실패 시에는 파일 경로/권한/코덱 문제를 의심
        // -------------------------------
        if (!cap_.isOpened()) {
            // RCLCPP_ERROR: 심각한 오류 로그(붉은 색으로 출력되는 로그 레벨)
            RCLCPP_ERROR(this->get_logger(), "Error opening video file 7: %s", video_path.c_str());
        } else {
            // RCLCPP_INFO: 일반 정보 로그
            RCLCPP_INFO(this->get_logger(), "Successfully opened video file 7: %s", video_path.c_str());
        }

        // -------------------------------
        // 5) 퍼블리시 주기 설정 (타이머)
        //    - 목표 FPS: 30.0
        //    - period = 1 / fps 초 단위 (예: 33.33ms)
        //    - create_wall_timer에 period와 콜백 바인딩
        // -------------------------------
        double fps = 30.0;
        auto period = std::chrono::duration<double>(1.0 / fps);

        // 타이머 생성: period마다 timer_callback 실행
        timer_ = this->create_wall_timer(
            period, std::bind(&VideoPublisher7::timer_callback, this));
    }

private:
    // -------------------------------------------------------------
    // timer_callback:
    //  - 타이머가 호출할 때마다 비디오에서 프레임을 읽고
    //  - cv::Mat -> sensor_msgs::msg::Image로 변환 후 퍼블리시
    // -------------------------------------------------------------
    void timer_callback()
    {
        cv::Mat frame;          // OpenCV에서 프레임을 저장하는 객체
        cap_ >> frame;          // 다음 프레임을 읽음. 읽을 프레임이 없으면 frame.empty() == true

        // ---------------------------------------------------------
        // 비디오가 끝에 도달한 경우 (frame.empty() 참)
        //  - 재생을 반복하려면 프레임 포지션을 0으로 되돌림
        //  - 재시도 후에도 비어있다면(파일 문제) 함수 종료
        // ---------------------------------------------------------
        if (frame.empty()) {
            RCLCPP_INFO(this->get_logger(), "Video 7 loop end. Rewinding...");
            cap_.set(cv::CAP_PROP_POS_FRAMES, 0); // 프레임 포인터를 처음으로 이동
            cap_ >> frame;                        // 처음 프레임 다시 읽기
            if (frame.empty()) return;            // 여전히 비어있다면(파일 오류), 퍼블리시 중단
        }

        // ---------------------------------------------------------
        // OpenCV Mat -> ROS Image 메시지 변환
        //  - cv_bridge::CvImage를 사용하여 헤더, 인코딩, Mat을 묶고
        //    toImageMsg()로 sensor_msgs::msg::Image::SharedPtr 생성
        //  - 인코딩 "bgr8"은 OpenCV의 기본 컬러 포맷(B,G,R 각 8비트)
        // ---------------------------------------------------------
        sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(
            std_msgs::msg::Header(), "bgr8", frame).toImageMsg();

        // ---------------------------------------------------------
        // 퍼블리시: 토픽에 메시지 전송
        // ---------------------------------------------------------
        publisher_->publish(*msg);

        // 디버깅/통계용 카운터 증가
        frame_count_++;
    }

    // -------------------------------
    // 멤버 변수 설명
    // -------------------------------
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_; // 이미지 퍼블리셔 핸들
    rclcpp::TimerBase::SharedPtr timer_;   // 주기 실행용 타이머 핸들
    cv::VideoCapture cap_;                 // OpenCV 비디오 캡처 객체 (파일 또는 카메라)
    int frame_count_;                      // 퍼블리시한 프레임 수 카운터
};

// -------------------------------------------------------------
// main: ROS2 초기화 → 노드 생성 및 스핀 → 종료 처리
// -------------------------------------------------------------
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);                                 // ROS2 클라이언트 라이브러리 초기화
    rclcpp::spin(std::make_shared<VideoPublisher7>());        // 노드 실행(이벤트 루프)
    rclcpp::shutdown();                                       // ROS2 정리/종료
    return 0;
}
