#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import rospkg
from math import sqrt, atan2
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Bool
from tf.transformations import euler_from_quaternion
import numpy as np
from scipy.interpolate import interp1d
from scipy.spatial import KDTree

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
        self.local_path_size = 70
        self.prev_current_waypoint = 0
        self.smoothing_factor = 0.5  # 경로 평활화 계수

        rate = rospy.Rate(20)  # 20hz
        while not rospy.is_shutdown():
            self.update_local_path()
            rate.sleep()

    def odom_callback(self, msg):
        self.is_odom = True
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        # Extract orientation as quaternion and convert to Euler angles
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        _, _, yaw = euler_from_quaternion(orientation_list)
        
        # Update heading (yaw)
        self.heading = yaw

    def global_path_callback(self, msg):
        self.is_path = True
        self.global_path_msg = msg
        if self.is_odom:  # 현재 위치 정보가 있는 경우에만 초기화
            self.prev_current_waypoint = self.find_closest_waypoint(self.x, self.y)
            rospy.loginfo(f"Global path updated and prev_current_waypoint set to {self.prev_current_waypoint}")

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

            rospy.loginfo("Local path generated with {} poses".format(len(local_path_msg.poses)))
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
            elif dis > min_dis + 5.0:  # 거리가 다시 증가하기 시작하면 검색 중단
                break

        # 방향 기준 추가: 차량의 진행 방향과 경로점의 방향 비교
        angle_to_waypoint = atan2(pose.pose.position.y - y, pose.pose.position.x - x)
        angle_difference = (angle_to_waypoint - self.heading + np.pi) % (2 * np.pi) - np.pi
        
        # 특정 임계값을 기준으로 진행 방향과 맞지 않는 경우 무시
        if angle_difference < -0.1 or angle_difference > 0.1:
            return self.prev_current_waypoint

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