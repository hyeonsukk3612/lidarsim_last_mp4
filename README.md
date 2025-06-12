***

장애물 회피알고리즘을 작성하고 lidarsave 패키지에서 저장한 동영상 파일을 이용하여 
시뮬레이션을 수행하는 lidarsim 패키지를 작성 스캔영상에서 에러 계산 영상처리 결과를 동영상으로 저장 (mp4)
에러를 이용하여 속도명령을 전송 다이내믹셀 구동
참고유튜브 https :://www youtube com/watch?v=HvWfm 4 Xtzbs
패키지의 소스코드와 실행결과 동영상 
(mp4 을 깃허브에 업로드 완료 후 강사에게 확인 받을 것)

![image](https://github.com/user-attachments/assets/5c7b5495-4093-40bd-92ce-33e0e2d06440)

***

설정 및 빌드

***

colcon build --packages-select lidarsim

source install/setup.bash

***

젝슨보드

***

창1

ros2 run lidarsim video_publisher

창2

ros2 run lidarsim sub

***

윈도우

***

창1

ros2 run lidarsim obstacle_avoidance

***

코드설명

8방향으로 찾습니다. 갔던 곳인지도 판단합니다.


void find_connected_points(const Mat &mask, Point close, vector<Point> &connected_points) {

    int rows = mask.rows, cols = mask.cols;
    
    Mat visited = Mat::zeros(rows, cols, CV_8U); // (방문 여부 저장)
    
    queue<Point> q;
    
    q.push(close);
    
    visited.at<uchar>(close) = 1;
    
    connected_points.push_back(close);

    int dx[] = {-1, -1, -1, 0, 0, 1, 1, 1}; // (8방향 x좌표 변화)
    
    int dy[] = {-1, 0, 1, -1, 1, -1, 0, 1}; // (8방향 y좌표 변화)

    while (!q.empty()) {
    
        Point p = q.front(); q.pop();
        
        for (int i = 0; i < 8; ++i) {
        
            int nx = p.x + dx[i], ny = p.y + dy[i];
            
            if (nx >= 0 && nx < cols && ny >= 0 && ny < rows) {
                
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



![4a3f0574_gxpvqq](https://github.com/user-attachments/assets/a2fd8ece-814a-478f-9504-f6a0f09f9dba)


***

    // 영역 마스크 생성: 사각형 내부만 255
    
    Mat region_mask = Mat::zeros(mask.size(), CV_8U);
    
    vector<vector<Point>> contours = {region_contour};
    
    fillPoly(region_mask, contours, Scalar(255));
    
    내부만 사각형인부분을 만들어내고

    Mat red_region;
    
    bitwise_and(mask, region_mask, red_region);

    and연산으로 빨강점 부분을 추출합니다

    Point get_line_endpoint(Point start, Point target, int len) {
    
    double dx = target.x - start.x;
    
    double dy = target.y - start.y;
    
    double norm_val = sqrt(dx*dx + dy*dy);
    
    if (norm_val < 1e-6) norm_val = 1; // 0 나눗셈 방지 0에가까운값
    
    double nx = dx / norm_val, ny = dy / norm_val;
    
    return Point(start.x + int(nx * len), start.y + int(ny * len));
    
}

***

    vector<Point> left_region = {Point(250,125), Point(125,125), Point(125,250), Point(250,250)};
    vector<Point> right_region = {Point(250,125), Point(375,125), Point(375,250), Point(250,250)};

    밑사진과 같이 빨간점이 없으면 오른쪽 영역일경우 (0,250) 방향으로 긋게합니다 
    
    if (points.empty()) {
    
        line(frame, center, default_point, Scalar(255, 0, 0), 2);
        
        closest_pt = default_point;
        
        dist_center = norm(center - default_point);
        
        return;
        
    }
    
![image](https://github.com/user-attachments/assets/1fd2ae84-53cc-44dc-ad9c-a89fbdd022a7)

    
![image](https://github.com/user-attachments/assets/c42fa673-a7ff-4da1-b585-1b49fa86a007)

***

실제출력영상입니다

[https://youtu.be/24GPV--2Mh8](https://youtu.be/24GPV--2Mh8)

***

