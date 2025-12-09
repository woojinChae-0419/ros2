#include "rclcpp/rclcpp.hpp"            // ROS2의 핵심 기능(노드, 로그, 스핀 등)을 사용하기 위한 헤더
#include "sensor_msgs/msg/image.hpp"   // ROS2 이미지 메시지 타입 정의 (sensor_msgs::msg::Image)
#include "cv_bridge/cv_bridge.h"       // ROS 이미지 <-> OpenCV cv::Mat 변환을 돕는 라이브러리
#include <opencv2/opencv.hpp>          // OpenCV의 모든 기능(영상 읽기/처리/표시 등)
#include <chrono>                      // 시간 관련(타이머, 시간 측정 등)에 사용

using namespace std::chrono_literals;  // 100ms 같은 리터럴을 편하게 쓰기 위함

// ================================================================
//  VideoPublisher5
//  - 지정한 동영상 파일을 읽어서
//  - ROS2 토픽(video/image_raw)으로 프레임을 실시간 발행하는 노드
//  - 실제 카메라 대신 비디오 파일을 센서처럼 사용하려는 목적
// ================================================================
class VideoPublisher5 : public rclcpp::Node
{
public:
    // 생성자: 노드 이름 설정, 멤버 초기화, 퍼블리셔/타이머/비디오 열기 설정
    VideoPublisher5()
    : Node("video_publisher_5"), cap_(0), frame_count_(0)
    {
        // ------------------------------------------------------------
        // 1) Image Publisher 생성 (토픽: video/image_raw)
        //    - create_publisher<T>(토픽명, 큐사이즈)
        //    - 큐사이즈는 메시지 버퍼 크기와 유사한 역할(여기선 10)
        // ------------------------------------------------------------
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
            "video/image_raw", 10);

        // ------------------------------------------------------------
        // 2) 재생할 비디오 파일 절대경로 지정
        //    - 시뮬레이션 환경에서 특정 영상을 반복 재생하기 위해 절대경로 사용
        //    - 실제 환경에 따라 경로를 변경해야 함
        // ------------------------------------------------------------
        const std::string video_path = "/home/linux/simulation/5_lt_cw_100rpm_out.mp4";

        // ------------------------------------------------------------
        // 3) VideoCapture로 파일 열기
        //    - cap_.open(path) 또는 cap_(path)로 열 수 있음
        //    - 성공하면 cap_.isOpened()가 true
        // ------------------------------------------------------------
        cap_.open(video_path);

        // ------------------------------------------------------------
        // 4) 파일 오픈 성공 여부 확인 및 로그 출력
        //    - 실패하면 경로/파일 존재/코덱 문제를 의심해야 함
        //    - RCLCPP_ERROR / RCLCPP_INFO를 사용해 ROS 로그로 출력
        // ------------------------------------------------------------
        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(),
                         "Error opening video file 5: %s",
                         video_path.c_str());
        } else {
            RCLCPP_INFO(this->get_logger(),
                        "Successfully opened video file 5: %s",
                        video_path.c_str());
        }

        // ------------------------------------------------------------
        // 5) 타이머 설정 (30 FPS로 프레임 발행)
        //    - 목표 프레임레이트를 fps로 설정
        //    - period = 1 / fps 초 간격으로 콜백 호출
        //    - create_wall_timer(period, callback) 사용
        // ------------------------------------------------------------
        double fps = 30.0; // 고정 FPS(필요 시 파일의 실제 FPS로 자동 맞추기로 개선 가능)
        auto period = std::chrono::duration<double>(1.0 / fps);

        timer_ = this->create_wall_timer(
            period, std::bind(&VideoPublisher5::timer_callback, this));
    }

private:

    // ================================================================
    // timer_callback()
    //  - 타이머 주기마다 호출됨
    //  - 비디오에서 프레임을 읽고 ROS Image 메시지로 변환·퍼블리시
    // ================================================================
    void timer_callback()
    {
        cv::Mat frame;         // OpenCV에서 프레임을 담을 객체 (행렬)
        cap_ >> frame;         // VideoCapture에서 다음 프레임 읽기 (operator>> 사용)

        // ------------------------------------------------------------
        // (A) 영상 끝에 도달하면 다시 처음으로 되감기 (루프 재생)
        //    - frame.empty()가 true이면 읽을 프레임이 없음
        //    - cap_.set(cv::CAP_PROP_POS_FRAMES, 0)로 프레임 인덱스를 0으로 되돌림
        //    - 다시 읽고, 그래도 비어있으면(파일 문제) 함수 종료
        // ------------------------------------------------------------
        if (frame.empty()) {
            RCLCPP_INFO(this->get_logger(), "Video 5 loop end. Rewinding...");
            cap_.set(cv::CAP_PROP_POS_FRAMES, 0);  // 처음 프레임으로 이동
            cap_ >> frame;  // 처음 프레임 다시 읽기
            if (frame.empty()) return;  // 파일 자체 문제면 퍼블리시 중단
        }

        // ------------------------------------------------------------
        // (B) OpenCV Mat -> ROS Image 메시지로 변환
        //    - cv_bridge::CvImage를 사용
        //    - std_msgs::msg::Header()는 빈 헤더 (필요 시 stamp, frame_id 설정 가능)
        //    - 인코딩 "bgr8"은 OpenCV 기본 BGR 8비트 포맷
        //    - toImageMsg()로 sensor_msgs::msg::Image::SharedPtr 생성
        // ------------------------------------------------------------
        sensor_msgs::msg::Image::SharedPtr msg =
            cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame)
            .toImageMsg();

        // ------------------------------------------------------------
        // (C) 퍼블리시: 토픽에 메시지 전송
        //    - publisher_->publish(msg) 형식으로 발행
        //    - 여기는 퍼블리셔가 SharedPtr을 받으므로 *msg로 복사 전달
        // ------------------------------------------------------------
        publisher_->publish(*msg);

        // ------------------------------------------------------------
        // (D) 통계용 카운터 증가
        //    - 디버깅 또는 퍼포먼스 체크 시 유용
        // ------------------------------------------------------------
        frame_count_++;
    }

    // -------------------------------
    // 멤버 변수 설명
    // -------------------------------
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_; // 이미지 퍼블리셔 핸들
    rclcpp::TimerBase::SharedPtr timer_;   // 주기 실행용 타이머 핸들
    cv::VideoCapture cap_;                 // 비디오 파일을 읽는 OpenCV 객체
    int frame_count_;                      // 퍼블리시한 프레임 수 카운터
};


// ================================================================
// main()
//  - ROS2 환경 초기화 후 VideoPublisher5 노드 실행
//  - rclcpp::spin()은 노드의 콜백(타이머 포함)을 계속 호출함
// ================================================================
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);                                // ROS2 라이브러리 초기화
    rclcpp::spin(std::make_shared<VideoPublisher5>());       // 노드 생성 후 이벤트 루프 진입
    rclcpp::shutdown();                                      // 노드 종료 시 ROS2 정리
    return 0;
}
