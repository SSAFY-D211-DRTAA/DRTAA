#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
import os, rospkg

from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridgeError

# image parser Node 는 시뮬레이터에서 송신하는 Camera 센서 정보를 받아 실시간으로 출력하는 예제입니다.
# Camera 센서 정보인 /image_jpeg/compressed 라는 메세지를 Subscribe 합니다.
# Subscribe 한 데이터를 OpenCV 를 이용하여 Image 로 출력합니다.

# 노드 실행 순서 
# 1. 문자열 Type 데이터를 정수형으로 변환 
# 2. 읽을 수 있는 bgr 이미지로 변환 
# 3. 이미지 출력

class IMGParser:
    def __init__(self):
        self.image_sub = rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.callback)

    def callback(self, msg):
        try:
            # (1) np.fromstring 함수를 이용하여 읽어오는 data 정보를 uint8 형태로 변환한다.
            np_arr = np.frombuffer(msg.data, np.uint8)

            # (2) cv2.imdecode 함수를 이용하여 np_arr 변수를 3차원 배열로 만듭니다.
            img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        except CvBridgeError as e:
            print(e)

        # (3) 이미지를 출력 합니다.
        cv2.imshow('Image Window', img_bgr)
        cv2.waitKey(1)  # 1ms 지연

if __name__ == '__main__':
    rospy.init_node('image_parser', anonymous=True)
    image_parser = IMGParser()
    rospy.spin()