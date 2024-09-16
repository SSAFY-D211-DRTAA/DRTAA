#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import rospkg
from math import sqrt
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Bool

import numpy as np
from scipy.interpolate import interp1d

class local_path_pub:
    def __init__(self):
        rospy.init_node('local_path_pub', anonymous=True)

        # Subscribers
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/global_path', Path, self.global_path_callback)

        # Publishers
        self.local_path_pub = rospy.Publisher('/local_path', Path, queue_size=1)
        self.left_turn_pub = rospy.Publisher('/is_left_turn', Bool, queue_size=1)
        
        # Initialization
        self.is_odom = False
        self.is_path = False
        self.x = 0
        self.y = 0
        self.local_path_size = 30
        self.prev_current_waypoint = 0
        self.smoothing_factor = 0.8  # 경로 평활화 계수

        rate = rospy.Rate(50)  # 20hz
        while not rospy.is_shutdown():
            self.update_local_path()
            rate.sleep()

    def odom_callback(self, msg):
        self.is_odom = True
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

    def global_path_callback(self, msg):
        self.is_path = True
        self.global_path_msg = msg

    def update_local_path(self):
        if self.is_odom and self.is_path:
            local_path_msg = Path()
            local_path_msg.header.frame_id = 'map'
            
            current_waypoint = self.find_closest_waypoint(self.x, self.y, start_index=self.prev_current_waypoint)

            # 급격한 변화 방지를 위한 평활화
            smoothed_waypoint = int(self.smoothing_factor * self.prev_current_waypoint + 
                                    (1 - self.smoothing_factor) * current_waypoint)
            
            if smoothed_waypoint + self.local_path_size < len(self.global_path_msg.poses):
                local_path_msg.poses = self.global_path_msg.poses[smoothed_waypoint:smoothed_waypoint + self.local_path_size]
            else:
                local_path_msg.poses = self.global_path_msg.poses[smoothed_waypoint:]

            self.prev_current_waypoint = smoothed_waypoint

            is_turning_left = self.predict_left_turn(local_path_msg)
            self.left_turn_pub.publish(Bool(is_turning_left))
            self.local_path_pub.publish(local_path_msg)

    def find_closest_waypoint(self, x, y, start_index=0):
        min_dis = float('inf')
        current_waypoint = start_index
        for i in range(start_index, len(self.global_path_msg.poses)):
            pose = self.global_path_msg.poses[i]
            dis = sqrt((x - pose.pose.position.x)**2 + (y - pose.pose.position.y)**2)
            if dis < min_dis:
                min_dis = dis
                current_waypoint = i
            elif dis > min_dis + 1.0:  # 거리가 다시 증가하기 시작하면 검색 중단
                break
        return current_waypoint

    def predict_left_turn(self, local_path, look_ahead_distance=40):
        if len(local_path.poses) < 10:
            return False

        x = [pose.pose.position.x for pose in local_path.poses]
        y = [pose.pose.position.y for pose in local_path.poses]

        path_length = np.cumsum(np.sqrt(np.diff(x)**2 + np.diff(y)**2))
        path_length = np.insert(path_length, 0, 0)

        mask = path_length <= look_ahead_distance
        x = np.array(x)[mask]
        y = np.array(y)[mask]
        path_length = path_length[mask]

        if len(x) < 5:
            return False

        f = interp1d(path_length, np.column_stack((x, y)), kind='cubic', axis=0)
        smooth_path_length = np.linspace(0, path_length[-1], num=100)
        smooth_path = f(smooth_path_length)

        dx = np.gradient(smooth_path[:, 0])
        dy = np.gradient(smooth_path[:, 1])
        ddx = np.gradient(dx)
        ddy = np.gradient(dy)
        curvature = (dx * ddy - dy * ddx) / (dx**2 + dy**2)**1.5

        avg_curvature = np.mean(curvature)

        is_left_turn = avg_curvature > 0.01  # 임계값 조정 가능

        rospy.loginfo(f"Average curvature: {avg_curvature}, Is left turn: {is_left_turn}")

        return is_left_turn

if __name__ == '__main__':
    try:
        local_path_publisher = local_path_pub()
    except rospy.ROSInterruptException:
        pass