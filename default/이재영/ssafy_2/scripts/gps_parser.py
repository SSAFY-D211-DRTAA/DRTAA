#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import tf
import os
from pyproj import Proj
from std_msgs.msg import Float32MultiArray
from morai_msgs.msg import GPSMessage, EgoVehicleStatus

class LL2UTMConverter:
    def __init__(self, zone=52):
        self.gps_sub = rospy.Subscriber("/gps", GPSMessage, self.navsat_callback)
        self.x, self.y = None, None

        # 변환 하고자 하는 좌표계를 선언
        self.proj_UTM = Proj(proj='utm', zone=zone, ellps='WGS84', preserve_units=False)

    def navsat_callback(self, gps_msg):
        # GPS 센서에서 수신되는 위도 경도 데이터를 확인한다.
        self.lat = gps_msg.latitude
        self.lon = gps_msg.longitude

        self.convertLL2UTM()

        utm_msg = Float32MultiArray()
        utm_msg.data = [self.x, self.y]

        # 위도 경도 데이터와 변환한 UTM 좌표를 터미널 창에 출력 하여 확인
        os.system('clear')
        print('lat:', self.lat)
        print('lon:', self.lon)
        print('utm X:', self.x)
        print('utm Y:', self.y)

    def convertLL2UTM(self):
        # pyproj 라이브러리를 이용해 정의한 좌표 변환 변수를 이용하여 위 경도 데이터를 변환한다.
        xy_zone = self.proj_UTM(self.lon, self.lat)  # Note: Order is (longitude, latitude)

        self.x = xy_zone[0]
        self.y = xy_zone[1]

if __name__ == '__main__':
    rospy.init_node('gps_parser', anonymous=True)
    gps_parser = LL2UTMConverter()
    rospy.spin()