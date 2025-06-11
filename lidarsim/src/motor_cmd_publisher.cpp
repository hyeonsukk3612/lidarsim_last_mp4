#include <rclcpp/rclcpp.hpp>           // ROS2 C++ 노드 관련 라이브러리
#include <geometry_msgs/msg/vector3.hpp> // geometry_msgs의 Vector3 메시지 타입
#include <algorithm>                     // min, max 등 알고리즘 함수

class MotorCmdPublisher : public rclcpp::Node {
private: // 멤버 변수 선언을 클래스 상단으로 이동
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr publisher_; // 퍼블리셔 포인터
    rclcpp::TimerBase::SharedPtr timer_; // 타이머 포인터
    double left_line_length_;            // 왼쪽 라인 길이
    double right_line_length_;           // 오른쪽 라인 길이
    int dir_;                            // 시뮬레이션용 방향 플래그
    int BASE_SPEED;                      // 기본 속도 (길이차 미만일 때)
    int MAX_SPEED;                       // 속도 상한
    int MIN_SPEED;                       // 속도 하한
    int ACCEL_STEP;                      // 가속/감속 단계(더 부드러운 변화)
    int DIFF_THRESHOLD;                  // 길이차 임계값(변경 가능)
    int MAX_PROP_DIFF;                   // 최대 속도차(비례값)
    double prev_vel1_;                   // 이전 속도1(왼쪽 바퀴)
    double prev_vel2_;                   // 이전 속도2(오른쪽 바퀴)

public:
    MotorCmdPublisher() : Node("motor_cmd_publisher") { // 노드 생성자, 이름 지정
        publisher_ = this->create_publisher<geometry_msgs::msg::Vector3>("topic_dxlpub", 10); // 토픽 퍼블리셔 생성, 큐 크기 10
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), // 10Hz(100ms마다 콜백 호출)
            std::bind(&MotorCmdPublisher::timer_callback, this) // 콜백 함수 바인딩
        );
        left_line_length_ = 0;   // 왼쪽 라인 길이 초기화
        right_line_length_ = 0;  // 오른쪽 라인 길이 초기화
        dir_ = 1;                // 시뮬레이션용 방향 플래그 초기화
        BASE_SPEED = 50;         // 기본 속도
        MAX_SPEED = 150;         // 속도 상한
        MIN_SPEED = -150;        // 속도 하한
        ACCEL_STEP = 5;          // 가속/감속 단계
        prev_vel1_ = BASE_SPEED; // 이전 속도1(왼쪽 바퀴) 초기화
        prev_vel2_ = BASE_SPEED; // 이전 속도2(오른쪽 바퀴) 초기화
        DIFF_THRESHOLD = 10;     // 길이차 임계값
        MAX_PROP_DIFF = 200;     // 최대 속도차(비례값)
    }

    // clamp 함수 (C++14 호환)
    double clamp(double val, double min_val, double max_val) {
        return std::max(min_val, std::min(max_val, val)); // 값이 min~max 범위 내로 유지
    }

    void timer_callback() {
        // 실제 환경에서는 left_line_length_, right_line_length_를 영상처리 결과로 실시간 갱신해야 함
        // 아래는 시뮬레이션용: 왼쪽/오른쪽 길이가 교대로 커졌다 작아졌다 반복
        if (dir_ == 1) {
            left_line_length_ += 30;      // 왼쪽 길이 증가
            right_line_length_ -= 30;     // 오른쪽 길이 감소
            if (left_line_length_ >= 250) dir_ = 0; // 왼쪽 길이 최대 도달 시 방향 전환
        } else {
            left_line_length_ -= 30;      // 왼쪽 길이 감소
            right_line_length_ += 30;     // 오른쪽 길이 증가
            if (right_line_length_ >= 250) dir_ = 1; // 오른쪽 길이 최대 도달 시 방향 전환
        }
        if (left_line_length_ < 0) left_line_length_ = 0; // 음수 방지
        if (right_line_length_ < 0) right_line_length_ = 0; // 음수 방지

        double diff = left_line_length_ - right_line_length_; // 왼쪽-오른쪽 길이차
        double abs_diff = std::abs(diff); // 절대값 차이

        double vel1 = prev_vel1_; // 왼쪽 바퀴 속도
        double vel2 = prev_vel2_; // 오른쪽 바퀴 속도

        if (abs_diff < DIFF_THRESHOLD) {
            // 길이 차이가 임계값 미만이면 속도 변화 없이 이전 속도 유지
            vel1 = BASE_SPEED;
            vel2 = BASE_SPEED;
        } else {
            // 비례 제어: 오차(속도차)는 -MAX_PROP_DIFF~+MAX_PROP_DIFF로 선형 변화
            double error = clamp(diff, -250, 250); // 최대/최소 오차 제한 (예: 250픽셀)
            double prop = (error / 250.0) * MAX_PROP_DIFF; // -MAX_PROP_DIFF~+MAX_PROP_DIFF로 변환
            vel1 = clamp(BASE_SPEED - prop/2, MIN_SPEED, MAX_SPEED); // 왼쪽 바퀴 속도 계산
            vel2 = clamp(BASE_SPEED + prop/2, MIN_SPEED, MAX_SPEED); // 오른쪽 바퀴 속도 계산
        }

        // 가속/감속(부드러운 변화)
        if (vel1 > prev_vel1_) prev_vel1_ = std::min(vel1, prev_vel1_ + ACCEL_STEP); // 가속(최대 ACCEL_STEP만큼)
        else if (vel1 < prev_vel1_) prev_vel1_ = std::max(vel1, prev_vel1_ - ACCEL_STEP); // 감속(최대 ACCEL_STEP만큼)
        else prev_vel1_ = vel1; // 변화 없음

        if (vel2 > prev_vel2_) prev_vel2_ = std::min(vel2, prev_vel2_ + ACCEL_STEP); // 가속(최대 ACCEL_STEP만큼)
        else if (vel2 < prev_vel2_) prev_vel2_ = std::max(vel2, prev_vel2_ - ACCEL_STEP); // 감속(최대 ACCEL_STEP만큼)
        else prev_vel2_ = vel2; // 변화 없음

        geometry_msgs::msg::Vector3 vel; // 메시지 객체 생성
        vel.x = prev_vel1_;              // 왼쪽 바퀴 속도
        vel.y = prev_vel2_;              // 오른쪽 바퀴 속도
        vel.z = 0;                       // 사용하지 않음(0으로 고정)

        publisher_->publish(vel); // 메시지 발행
        RCLCPP_INFO(this->get_logger(), "Publish: left=%.1f, right=%.1f (길이: %.1f, %.1f, diff: %.1f)", vel.x, vel.y, left_line_length_, right_line_length_, diff); // 로그 출력
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv); // ROS2 초기화
    rclcpp::spin(std::make_shared<MotorCmdPublisher>()); // 노드 실행 및 대기
    rclcpp::shutdown(); // ROS2 종료
    return 0;
}
