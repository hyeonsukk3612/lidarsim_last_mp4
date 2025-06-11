#include <rclcpp/rclcpp.hpp>                    // ROS2 C++ 노드 라이브러리
#include <sensor_msgs/msg/image.hpp>            // ROS2 이미지 메시지 타입
#include <geometry_msgs/msg/vector3.hpp>        // ROS2 바퀴 속도 메시지 타입
#include <cv_bridge/cv_bridge.h>                // ROS <-> OpenCV 이미지 변환
#include <opencv2/opencv.hpp>                   // OpenCV 라이브러리
#include <vector>
#include <queue>
#include <algorithm>
using namespace std;
using namespace cv;

// 8방향 BFS로 연결된 빨간 점(픽셀)들을 모두 찾아 connected_points에 저장 (빨간점 군집 레이블링용)
void find_connected_points(const Mat &mask, Point start, vector<Point> &connected_points) {
    int rows = mask.rows, cols = mask.cols;
    Mat visited = Mat::zeros(rows, cols, CV_8U); // (방문 여부 저장)
    queue<Point> q;
    q.push(start);
    visited.at<uchar>(start) = 1;
    connected_points.push_back(start);

    int dx[] = {-1, -1, -1, 0, 0, 1, 1, 1}; // (8방향 x좌표 변화)
    int dy[] = {-1, 0, 1, -1, 1, -1, 0, 1}; // (8방향 y좌표 변화)

    while (!q.empty()) {
        Point p = q.front(); q.pop();
        for (int i = 0; i < 8; ++i) {
            int nx = p.x + dx[i], ny = p.y + dy[i];
            if (nx >= 0 && nx < cols && ny >= 0 && ny < rows) {
                // (미방문 빨간점이면 방문 처리 및 큐 삽입)
                if (mask.at<uchar>(ny, nx) > 0 && visited.at<uchar>(ny, nx) == 0) {
                    visited.at<uchar>(ny, nx) = 1;
                    Point np(nx, ny);
                    connected_points.push_back(np);
                    q.push(np);
                }
            }
        }
    }
}

// 특정 사각형 영역(region_contour) 내에서 빨간점을 찾아 시각화 및 제어에 필요한 정보 반환
// - 가장 가까운 빨간점 및 연결된 점들을 초록색 박스
// - 중심점(center)과 가장 가까운 점(없으면 default_point) 파란선 표시
// - (closest_pt)가장 가까운 점 좌표, (dist_center)중심점과의 거리 반환
void process_region(Mat &frame, const vector<Point> &region_contour, Point center, Point default_point, Scalar box_color, Point &closest_pt, double &dist_center) {
    Mat hsv, mask1, mask2, mask;
    cvtColor(frame, hsv, COLOR_BGR2HSV); // (BGR → HSV 변환)
    inRange(hsv, Scalar(0, 20, 20), Scalar(15, 255, 255), mask1);    // 빨간색 범위1 마스크 하나만으로는힘들다
    inRange(hsv, Scalar(165, 20, 20), Scalar(179, 255, 255), mask2); // 빨간색 범위2 마스크
    mask = mask1 | mask2; // 두 마스크 합침

    // 영역 마스크 생성: 사각형 내부만 255
    Mat region_mask = Mat::zeros(mask.size(), CV_8U);
    vector<vector<Point>> contours = {region_contour};
    fillPoly(region_mask, contours, Scalar(255));

    // 영역 내 빨간점만 추출
    Mat red_region;
    bitwise_and(mask, region_mask, red_region);

    vector<Point> points;
    findNonZero(red_region, points); // 빨간점 좌표 추출

    if (points.empty()) {
        // 빨간점이 없으면 중심-기본점 파란선
        line(frame, center, default_point, Scalar(255, 0, 0), 2);
        closest_pt = default_point;
        dist_center = norm(center - default_point);
        return;
    }

    // 중심점에서 가장 가까운 빨간점 찾기
    Point closest;
    double min_dist = 1e9; //1000000000 무한대
    for (const auto &pt : points) {
        double dist = norm(pt - center);
        if (dist < min_dist) {
            min_dist = dist;
            closest = pt;
        }
    }

    // 가장 가까운 점에서 연결된 빨간점들
    vector<Point> connected_points;
    find_connected_points(red_region, closest, connected_points); // 빨간 점들을 모두 찾기

    // 연결된 빨간점들을 초록색 박스로 표시
    Rect box = boundingRect(connected_points);
    rectangle(frame, box, box_color, 2);

    // 중심점과 가장 가까운 빨간점 파란선
    line(frame, center, closest, Scalar(255, 0, 0), 2);

    closest_pt = closest;
    dist_center = norm(center - closest);
}

// 두 점(start→target) 방향으로 길이 len의 선분 끝점 좌표 반환 (검은선 끝점 계산)
Point get_line_endpoint(Point start, Point target, int len) {
    double dx = target.x - start.x;
    double dy = target.y - start.y;
    double norm_val = sqrt(dx*dx + dy*dy);
    if (norm_val < 1e-6) norm_val = 1; // 0 나눗셈 방지 0에가까운값
    double nx = dx / norm_val, ny = dy / norm_val;
    return Point(start.x + int(nx * len), start.y + int(ny * len));
}

// 장애물 회피 및 라인 추적 노드 클래스
class ObstacleAvoidance : public rclcpp::Node {
public:
    // 생성자: ROS2 토픽 구독/발행, 영상 저장 준비
    ObstacleAvoidance() : Node("obstacle_avoidance") {
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/image_topic", 10,
            std::bind(&ObstacleAvoidance::image_callback, this, std::placeholders::_1)); // (이미지 토픽 구독자)
        vel_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("topic_dxlpub", 10); // (바퀴 속도 퍼블리셔)
        writer_open_ = false; // (영상 저장 여부)
        namedWindow("Obstacle Avoidance", WINDOW_NORMAL); // (시각화 창 생성)
    }

    ~ObstacleAvoidance() {
        if (writer_open_) writer_.release(); // (영상 저장 종료)
        destroyAllWindows(); // (시각화 창 종료)
    }

private:
    // 멤버 변수 선언부 (최상단에 위치)
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_; // (이미지 구독자)
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr vel_pub_;  // (바퀴 속도 퍼블리셔)
    VideoWriter writer_;    // (결과 영상 저장 객체)
    bool writer_open_;      // (영상 저장 활성화 여부)
    // 이미지 콜백: 프레임 처리, 빨간점 탐지, 속도 계산 및 발행, 시각화
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, "bgr8"); // (ROS→OpenCV 변환)
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }
        Mat frame = cv_ptr->image; // (현재 프레임)

        // 중심점(250,250), 왼쪽/오른쪽 기본점
        Point center(250, 250);           // (화면 중앙점)
        Point left_default(125, 250);     // (왼쪽 빨간점 없을 때 연결점)
        Point right_default(375, 250);    // (오른쪽 빨간점 없을 때 연결점)

        // 왼쪽, 오른쪽 사각형 영역 정의 (빨간점 탐지용)
        vector<Point> left_region = {Point(250,125), Point(125,125), Point(125,250), Point(250,250)};
        vector<Point> right_region = {Point(250,125), Point(375,125), Point(375,250), Point(250,250)};

        // 각 영역 내 가장 가까운 빨간점 좌표 및 중심과의 거리
        Point left_closest, right_closest;
        double left_dist, right_dist;

        // 왼쪽/오른쪽 빨간점 탐지 및 시각화 (초록박스, 파란선)
        process_region(frame, left_region, center, left_default, Scalar(0,255,0), left_closest, left_dist);
        process_region(frame, right_region, center, right_default, Scalar(0,255,0), right_closest, right_dist);

        // 두 영역에서 찾은 점의 평균좌표 계산 (검은선 방향)
        Point avg_point((left_closest.x + right_closest.x) / 2, (left_closest.y + right_closest.y) / 2);

        // 검은선 끝점 결정: 평균 y=250이면 위로, 아니면 평균좌표 방향 길이 125
        Point black_end;
        if (avg_point.y == 250) {
            black_end = Point(250, 125); // (위로)
        } else {
            black_end = get_line_endpoint(center, avg_point, 125); // (평균좌표 방향)
        }
        line(frame, center, black_end, Scalar(0,0,0), 2); // (검은선 그리기)

        // 검은선 각도(라디안, degree) 계산
        double dx = black_end.x - center.x;
        double dy = black_end.y - center.y;
        double angle = atan2(dy, dx); // (검은선 방향 각도, 라디안)

        // 모터 제어용 속도 계산 (moter.txt 참고)
        double base_speed = 50;      // (기본 전진 속도, 왼쪽 양수/오른쪽 음수)
        double max_diff = 80;        // (diff(회전 보정)의 최대 절대값)
        double Kp = 20.0;            // (각도(라디안) 비례 계수, 조향 민감도)
        double diff = Kp * angle;    // (각도에 비례한 회전 보정값)
        diff = std::max(-max_diff, std::min(max_diff, diff)); // (-max_diff~+max_diff로 제한)

        // 왼쪽: 양수, 오른쪽: 음수 → 전진, diff로 회전 보정
        double left_speed = base_speed - diff;   // (왼쪽 바퀴 속도)
        double right_speed = -base_speed + diff; // (오른쪽 바퀴 속도)

        // 바퀴 속도 제한 (-150~150)
        left_speed = std::max(-150.0, std::min(150.0, left_speed));
        right_speed = std::max(-150.0, std::min(150.0, right_speed));

        // ROS2 메시지 생성 및 발행 (sub.cpp/dxl.cpp에서 바로 사용)
        geometry_msgs::msg::Vector3 vel_msg;
        vel_msg.x = left_speed;   // (왼쪽 바퀴 속도, 양수: 전진)
        vel_msg.y = right_speed;  // (오른쪽 바퀴 속도, 음수: 전진)
        vel_msg.z = 0;            // (사용 안함)
        vel_pub_->publish(vel_msg);

        // 터미널에 속도, 각도값 모두 실시간 출력 (sub.cpp에서 받은 값과 동일)
        RCLCPP_INFO(this->get_logger(),
            "Publish: left=%.1f, right=%.1f, angle=%.2f deg",
            left_speed, right_speed, angle * 180.0 / CV_PI);

        // 결과 영상 저장 (mp4)
        if (!writer_open_) {
            writer_.open("result_obstacle_avoidance.mp4", VideoWriter::fourcc('m','p','4','v'), 30, Size(frame.cols, frame.rows));
            writer_open_ = true;
        }
        writer_.write(frame);

        // 실시간 시각화
        imshow("Obstacle Avoidance", frame);
        waitKey(1);
    }


};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv); // (ROS2 초기화)
    rclcpp::spin(std::make_shared<ObstacleAvoidance>()); // (노드 실행 및 콜백)
    rclcpp::shutdown(); // (ROS2 종료)
    return 0;
}

