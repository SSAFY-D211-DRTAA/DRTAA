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

        # BGR 이미지와 HSV 이미지를 수평으로 연결
        img_concat = np.concatenate((img_bgr, img_hsv), axis=1)

        # 이미지를 출력
        cv2.imshow("BGR and HSV Image", img_concat)
        cv2.waitKey(1)  # 1ms 대기하여 창을 업데이트

if __name__ == '__main__':
    rospy.init_node('image_parser', anonymous=True)
    image_parser = IMGParser()
    rospy.spin()