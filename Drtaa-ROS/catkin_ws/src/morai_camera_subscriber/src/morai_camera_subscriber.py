#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np

def callback(data):
    rospy.loginfo("Received an image!")
    
    # 이미지를 디코딩합니다.
    np_arr = np.frombuffer(data.data, np.uint8)
    image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    
    # 이미지를 화면에 표시합니다.
    cv2.imshow("Morai Image Viewer", image_np)
    cv2.waitKey(1)

def listener():
    rospy.init_node('image_subscriber', anonymous=True)
    rospy.loginfo("Morai Image Subscriber Node has started")
    rospy.Subscriber('/image_jpeg/compressed', CompressedImage, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
