cmake_minimum_required(VERSION 3.5)
project(lidarsim)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(geometry_msgs REQUIRED)  # 추가!

ament_export_dependencies(rclcpp std_msgs sensor_msgs cv_bridge OpenCV geometry_msgs) # 추가!

add_executable(obstacle_avoidance src/obstacle_avoidance.cpp)
ament_target_dependencies(obstacle_avoidance rclcpp std_msgs sensor_msgs cv_bridge OpenCV)

add_executable(video_publisher src/video_publisher.cpp)
ament_target_dependencies(video_publisher rclcpp std_msgs sensor_msgs cv_bridge OpenCV)

add_executable(motor_cmd_publisher src/motor_cmd_publisher.cpp)
ament_target_dependencies(motor_cmd_publisher rclcpp geometry_msgs) # std_msgs → geometry_msgs로 변경

install(TARGETS
  obstacle_avoidance
  video_publisher
  motor_cmd_publisher
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
