#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from morai_msgs.msg import MultiEgoSetting

def setup_multiple_egos():
    # Publisher 생성
    publisher = rospy.Publisher('/ego_setting', MultiEgoSetting, queue_size=10)
    rospy.init_node('Multi_Ego_Setup', anonymous=True)

    # MultiEgoSetting 메시지 생성 및 초기화
    ego_setting_msg = MultiEgoSetting()
    ego_setting_msg.number_of_ego_vehicle = 4  # 생성할 Ego 차량의 수
    ego_setting_msg.camera_index = 0  # 카메라 인덱스 설정

    # 각 차량의 인덱스 및 초기 위치 설정
    ego_setting_msg.ego_index = [0, 2, 3, 4]
    ego_setting_msg.global_position_x = [10.0, 20.0, 30.0, 40.0]
    ego_setting_msg.global_position_y = [0.0, 0.0, 0.0, 0.0]
    ego_setting_msg.global_position_z = [0.0, 0.0, 0.0, 0.0]

    # 각 차량의 초기 회전 각도 설정
    ego_setting_msg.global_roll = [0.0, 0.0, 0.0, 0.0]
    ego_setting_msg.global_pitch = [0.0, 0.0, 0.0, 0.0]
    ego_setting_msg.global_yaw = [0.0, 0.0, 0.0, 0.0]

    # 각 차량의 초기 속도 및 제어 모드 설정
    ego_setting_msg.velocity = [0.0, 0.0, 0.0, 0.0]
    ego_setting_msg.gear = [0, 0, 0, 0]
    ego_setting_msg.ctrl_mode = [4, 4, 4, 4]  # 제어 모드 설정

    rate = rospy.Rate(1)  # 1 Hz로 메시지 발행
    while not rospy.is_shutdown():
        rospy.loginfo("Publishing MultiEgoSetting: %s", ego_setting_msg)
        publisher.publish(ego_setting_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        setup_multiple_egos()
    except rospy.ROSInterruptException:
        pass