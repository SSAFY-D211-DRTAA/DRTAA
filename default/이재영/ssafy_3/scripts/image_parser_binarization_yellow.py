#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
import os, rospkg

from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridgeError

class IMGParser:
    def __init__(self):
        self.image_sub = rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.callback)

    def callback(self, msg):
        try:
            # 압축된 이미지 데이터를 numpy 배열로 변환
            np_arr = np.frombuffer(msg.data, np.uint8)
            # numpy 배열을 디코딩하여 BGR 이미지로 변환
            img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except CvBridgeError as e:
            print(e)
            return

        # BGR 이미지를 HSV 이미지로 변환
        img_hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)

        # 특정 색상 영역을 검출하기 위해 범위를 지정합니다 (노란색 영역)
        lower_ylane = np.array([20, 100, 100])
        upper_ylane = np.array([30, 255, 255])

        # cv2.inRange 함수를 이용하여 HSV 이미지에서 색상 범위를 지정
        img_ylane = cv2.inRange(img_hsv, lower_ylane, upper_ylane)

        # 검출된 노란색 영역을 BGR 이미지로 변환
        img_ylane_bgr = cv2.cvtColor(img_ylane, cv2.COLOR_GRAY2BGR)

        # 원본 이미지와 검출된 노란색 영역 이미지를 수평으로 연결
        img_concat = np.concatenate((img_bgr, img_ylane_bgr), axis=1)

        # 이미지를 출력
        cv2.imshow("Image window", img_concat)
        cv2.waitKey(1)

if __name__ == '__main__':
    rospy.init_node('image_parser', anonymous=True)
    image_parser = IMGParser()
    rospy.spin()