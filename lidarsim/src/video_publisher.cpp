#include <rclcpp/rclcpp.hpp>               // ROS2 C++ 노드 라이브러리
#include <sensor_msgs/msg/image.hpp>       // ROS2 이미지 메시지 타입
#include <cv_bridge/cv_bridge.h>           // OpenCV와 ROS2 이미지 변환
#include <opencv2/opencv.hpp>              // OpenCV 라이브러리
using namespace std;
using namespace cv;

class VideoPublisher : public rclcpp::Node {
public:
    VideoPublisher() : Node("video_publisher") { // 노드 생성자, 이름 지정
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/image_topic", 10); // 이미지 퍼블리셔 생성, 큐 크기 10
        string video_path = "/home/jetson/ros2_ws/src/lidarsim/avoid123.mp4"; // 비디오 파일 경로
        cap_.open(video_path, CAP_FFMPEG); // FFMPEG으로 비디오 파일 열기
        if (!cap_.isOpened()) { // 비디오 파일 열기 실패 시
            RCLCPP_ERROR(this->get_logger(), "Could not open video file");
            rclcpp::shutdown(); // ROS2 종료
            return;
        }
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), // 33ms마다 콜백 호출 (약 30Hz)
            std::bind(&VideoPublisher::timer_callback, this) // 콜백 함수 바인딩
        );
    }
private:
    void timer_callback() {
        Mat frame; // 프레임 저장 변수
        cap_ >> frame; // 비디오에서 프레임 읽기
        if (frame.empty()) { // 프레임이 비어있으면(비디오 종료)
            RCLCPP_INFO(this->get_logger(), "Video ended.");
            rclcpp::shutdown(); // ROS2 종료
            return;
        }
        // OpenCV 프레임을 ROS2 이미지 메시지로 변환
        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
        publisher_->publish(*msg); // 이미지 메시지 발행
    }
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_; // 이미지 퍼블리셔
    rclcpp::TimerBase::SharedPtr timer_; // 타이머
    VideoCapture cap_; // OpenCV 비디오 캡처 객체
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv); // ROS2 초기화
    rclcpp::spin(std::make_shared<VideoPublisher>()); // 노드 실행 및 콜백 대기
    rclcpp::shutdown(); // ROS2 종료
    return 0;
}

