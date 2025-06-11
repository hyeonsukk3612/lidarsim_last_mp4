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



![4a3f0574_gxpvqq](https://github.com/user-attachments/assets/a2fd8ece-814a-478f-9504-f6a0f09f9dba)



카메라 실제 구동 영상입니다
