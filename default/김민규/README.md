# D211 김민규

# 8월 26일

ROS - Android 연결 시도 실패
rosjava, rosAndroid를 버전을 미세하게 바꿔가면서 작성해보기도 하고, 유튜브 영상, 해외 레딧들을 다 찾아봤는데 결국 websocket이 답인 것 같다.

MLkit에 있는 FaceMeshDetector를 사용해서 눈꺼풀 감지 단위기능 구현

https://github.com/kimmandoo/android-drill/tree/main/FaceMesh



# 8월 27, 28일

# ROS

### 메시지와 토픽 → 노드 간 데이터 주고받을 때 사용

마스터를 통해서 각 노드 Publisher ↔ Subscriber 정보 공유해서 메시지 주고받음

⇒ 토픽(메시지 이름), 메시지 타입 맞춰야 됨

메시지 타입은 std_msg/String 이런식으로 나온다.

메시지 발행은 한 노드가 하는데, 수신은 여러 노드에서 받을 수 있다.

### rosbridge

JSON형식의 통신을 주고받으며 ROS 메시지로 변환해주는 역할을 한다.

→ ROS 외부와의 통신을 담당한다.

ROS사용할 때 항상 sourcing이 되어있는지 확인해야된다.

그냥 /opt/ros/dirtro/setup.bash 에서 해서 가능한것과 devel 밑에 있는 setup.bash를 sourcing 하는 게 큰 차이가 있다.

## 센서

카메라, 라이다, 레이더, GPS, IMU

관성 측정 장비 →IMU(Inertia Measurement Unit)

가속도계 + 회전 속도계 + 자력계 조합

Ground Truth

→ 참값 3종류 RGB, Semantic(같은 객체 끼리 구분해주는 기능)

→ Instance는 식별기능인데 사용ㄹ ㅣ소스가 무한일 때 Instance로 모두 대체 가능

Semantic이 영역만 보면, Instance는 식별은 물론 바운딩 박스, 객체 위치와 속도, 정보까지 알 수 있음

라이다, 레이더 → 신호 강도가 추가 고려 정보..

반사되어 돌아오는 신호강도가 다 다름. 통과 매질, 표면 거칠기, 각도 같은 것에 영향을 받는 반사율

## Req 1

가상환경 구축완료 → catkin_make를 하려면 src 디렉토리가 있어야된다.

![image.png](https://prod-files-secure.s3.us-west-2.amazonaws.com/85916e9f-ea41-498b-818b-7390937a2a7b/620e4452-5e0a-4b47-a1ff-50b07ef5fcca/image.png)

ROS 설치 및 확인완료

## Req 2

```python
mingyu@vm-ros-noetic:~/catkin_sub$ rosrun ssafy_1 my_name_listener.py
[rosrun] Couldn't find executable named my_name_listener.py below /home/mingyu/catkin_sub/src/ssafy_1
[rosrun] Found the following, but they're either not files,
[rosrun] or not executable:
[rosrun]   /home/mingyu/catkin_sub/src/ssafy_1/scripts/my_name_listener.py
```

→ my_name_talker에 chmod +x 로 실행 권한을 설정한다.

+) 지금 문제는 20.04로 올리면서 python3를 쓰고있기 때문에 기존코드에 적혀 있는 환경변수가 안맞는다.

최상단의 코드를 아래처럼 바꿔준다.

*#!/usr/bin/env python3*

이러고 나면 std_msg오류가 발생한다 CMakeLists파일에서 꼬인 것이므로 직접 찾아가서 주석을 해제하면 된다.(활성화 시키기)

![Screenshot from 2024-08-28 21-47-55.png](https://prod-files-secure.s3.us-west-2.amazonaws.com/85916e9f-ea41-498b-818b-7390937a2a7b/ab535fda-1a27-48b9-9087-31b0ffbb9851/Screenshot_from_2024-08-28_21-47-55.png)

각 과정에서 발생하는 오류는 또 만나면 그때 기록하기

![Screenshot from 2024-08-28 21-48-45.png](https://prod-files-secure.s3.us-west-2.amazonaws.com/85916e9f-ea41-498b-818b-7390937a2a7b/86af2821-fac6-4292-ba29-420e44ea25ae/Screenshot_from_2024-08-28_21-48-45.png)

혈투의 흔적

```python
#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from ssafy_1.msg import student

# my_name_talker 는 Custom Msgs 를 이용한 Topic Publisher(메세지 송신) 예제입니다.
# /my_name 라는 메세지를 Publish 합니다.

# 노드 실행 순서 
# 1. publisher 생성
# 2. ROS 노드 이름 선언
# 3. 코드 반복 시간 설정 및 반복 실행
# 4. 송신 될 메세지 변수 생성 및 터미널 창 출력
# 5. /my_name 메세지 Publish

def talker():
    #TODO: (1) publisher 생성
    '''
    # student 라는 직접 만든 Custom ROS 메세지 형식을 사용하여 Topic Publisher 를 완성한다.
    # Topic 이름은 'my_name' 으로 설정한다.
    publisher = rospy.Publisher( 변수 1 , 변수 2 , queue_size=10)
    '''

    publisher = rospy.Publisher('my_name', student, queue_size=10)

    #TODO: (2) ROS 노드 이름 선언
    rospy.init_node('my_name_talker', anonymous=True)

    count = 0

    #TODO: (3) 코드 반복 시간 설정 및 반복 실행    
    rate = rospy.Rate(1) # 1 hz
    while not rospy.is_shutdown():
        #TODO: (4) 송신 될 메세지 변수 생성 및 터미널 창 출력 

        # 송신 될 메세지 변수를 만든뒤 출력 결과를 확인한다.        
        my_name = student()
        my_name.first_name =  "민규"
        my_name.last_name = "김"
        my_name.age = 25
        my_name.score = 100
        rospy.loginfo('\\n my name : %s %s \\n my age : %i \\n SSAFY score : %i', my_name.first_name,my_name.last_name,my_name.age,my_name.score)

        #TODO: (5) /my_name 메세지 Publish 
        publisher.publish(my_name)

        count += 1
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
```

rospy.loginfo는 print와 같은 기능 → 콘솔창에 출력하는 역할을 한다.

```python
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String
from ssafy_1.msg import student

# my_name_listener 는 Custom Msgs 를 이용한 Topic Subscriber(메세지 수신) 예제입니다.
# /my_name 라는 메세지를 Subscribe 합니다.

# 노드 실행 순서 
# 1. ROS 노드 이름 선언
# 2. Subscriber 생성
# 3. Callback 함수 생성 및 데이터 출력

#TODO: (3) Callback 함수 생성 및 데이터 출력
def callback(data):
    rospy.loginfo('\\n my name : %s %s \\n my age : %i \\n SSAFY score : %i', data.first_name,data.last_name,data.age,data.score)

def listener():
    #TODO: (1) ROS 노드 이름 선언
    rospy.init_node('my_name_listener', anonymous=True)

    #TODO: (2) Subscriber 생성
    '''
    # student 라는 직접 만든 Custom ROS 메세지 형식을 사용하여 Topic Subscriber 를 완성한다.
    # Topic 이름은 'my_name' 으로 설정한다.
    rospy.Subscriber( 변수 1 , 변수 2 , callback)

    '''
    rospy.Subscriber('my_name', student, callback=callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
```
