cmake_minimum_required(VERSION 3.0.2)
project(morai_camera_subscriber)

# ROS 패키지들에 대한 의존성을 찾아 설정합니다.
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  image_transport
  compressed_image_transport
)

# catkin 패키지 설정
catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs image_transport compressed_image_transport
)

# include 디렉토리를 설정합니다.
include_directories(
  ${catkin_INCLUDE_DIRS}
)

# src 디렉토리 내의 모든 Python 파일을 설치 대상으로 추가
file(GLOB PYTHON_SCRIPTS src/*.py)

# Python 스크립트를 catkin 설치 대상에 추가합니다.
catkin_install_python(PROGRAMS ${PYTHON_SCRIPTS}
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
