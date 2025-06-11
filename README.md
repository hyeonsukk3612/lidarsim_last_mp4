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

카메라 실제 구동 영상입니다
