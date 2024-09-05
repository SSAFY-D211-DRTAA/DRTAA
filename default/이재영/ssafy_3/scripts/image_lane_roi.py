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
        
        # image_size
        x = 640
        y = 480

        # 관심있는 영역만 지정
        # 이미지의 좌표를 직접 지정하거나 이미지의 비율로 정의할 수 있습니다.
        # 여기서는 예시로 4개의 포인트를 지정합니다.
        self.crop_pts = np.array([[100, 480], [540, 480], [400, 245], [240, 245]], dtype=np.int32)

    def callback(self, msg):
        try:
            np_arr = np.fromstring(msg.data, np.uint8)
            img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except CvBridgeError as e:
            print(e)

        self.mask = self.mask_roi(img_bgr)
        if len(self.mask.shape) == 3:
            img_concat = np.concatenate([img_bgr, self.mask], axis=1)
        else:
            img_concat = np.concatenate([img_bgr, cv2.cvtColor(self.mask, cv2.COLOR_GRAY2BGR)], axis=1)

        cv2.imshow("Image window", img_concat)
        cv2.waitKey(1)

    def mask_roi(self, img):
        h = img.shape[0]
        w = img.shape[1]
        
        if len(img.shape) == 3:
            c = img.shape[2]
            mask = np.zeros((h, w, c), dtype=np.uint8)
            mask_value = (255, 255, 255)
        else:
            mask = np.zeros((h, w), dtype=np.uint8)
            mask_value = 255
        
        # 다각형을 그려 관심 영역을 만듭니다.
        cv2.fillPoly(mask, [self.crop_pts], mask_value)

        # 비트 연산을 통해 이미지와 마스크를 결합합니다.
        masked_img = cv2.bitwise_and(img, mask)
        
        return masked_img


if __name__ == '__main__':
    rospy.init_node('image_parser', anonymous=True)
    image_parser = IMGParser()
    rospy.spin()