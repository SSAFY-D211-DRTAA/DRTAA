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
        self.heading = 0
        self.local_path_size = 70
        self.prev_current_waypoint = 0
        self.smoothing_factor = 0.4
        self.global_path_msg = None
        self.path_tree = None
        
        rate = rospy.Rate(20) # 20hz
        while not rospy.is_shutdown():
            self.update_local_path()
            rate.sleep()

    def odom_callback(self, msg):
        self.is_odom = True
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        _, _, yaw = euler_from_quaternion(orientation_list)
        self.heading = yaw

    def global_path_callback(self, msg):
        self.is_path = True
        self.global_path_msg = msg
        self.build_kd_tree()
        if self.is_odom:
            self.prev_current_waypoint = self.find_closest_waypoint(self.x, self.y, start_index=0)
        rospy.loginfo(f"Global path updated and prev_current_waypoint set to {self.prev_current_waypoint}")

    def build_kd_tree(self):
        if self.global_path_msg is not None:
            points = [[p.pose.position.x, p.pose.position.y] for p in self.global_path_msg.poses]
            self.path_tree = KDTree(points)

    def update_local_path(self):
        if self.global_path_msg is None or len(self.global_path_msg.poses) == 0:
            rospy.logwarn("Global path is empty or not received yet.")
            return

        local_path_msg = Path()
        local_path_msg.header.frame_id = 'map'

        # 현재 위치에서 가장 가까운 웨이포인트 찾기
        current_waypoint = self.find_closest_waypoint_in_direction(self.x, self.y, self.heading, self.prev_current_waypoint)

        # 경로 이탈 감지
        if current_waypoint == -1:
            rospy.logwarn("Vehicle has deviated significantly from the path. Recalculating...")
            current_waypoint = self.find_closest_waypoint(self.x, self.y, start_index=0)

        # 로컬 패스 생성
        if current_waypoint != -1:
            end_index = min(current_waypoint + self.local_path_size, len(self.global_path_msg.poses))
            local_path_msg.poses = self.global_path_msg.poses[current_waypoint:end_index]

        self.prev_current_waypoint = current_waypoint

        is_turning_left = self.predict_left_turn(local_path_msg)
        self.left_turn_pub.publish(Bool(is_turning_left))
        self.local_path_pub.publish(local_path_msg)

    def find_closest_waypoint(self, x, y, start_index):
        if self.path_tree is None:
            return start_index
        _, idx = self.path_tree.query([x, y])
        return idx

    def find_closest_waypoint_in_direction(self, x, y, heading, start_index):
        if self.global_path_msg is None or len(self.global_path_msg.poses) == 0:
            return -1

        min_dist = float('inf')
        closest_waypoint = -1
        max_search_distance = 50  # 최대 탐색 거리 설정

        for i in range(start_index, len(self.global_path_msg.poses)):
            pose = self.global_path_msg.poses[i].pose
            dx = pose.position.x - x
            dy = pose.position.y - y
            dist = sqrt(dx*dx + dy*dy)

            if dist > max_search_distance:
                break

            # 진행 방향 고려
            angle_to_point = atan2(dy, dx)
            angle_diff = abs((angle_to_point - heading + np.pi) % (2*np.pi) - np.pi)

            if dist < min_dist and angle_diff < np.pi/2:  # 90도 이내의 각도만 고려
                min_dist = dist
                closest_waypoint = i

        return closest_waypoint

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

        is_left_turn = avg_curvature > 0.01
        rospy.loginfo(f"Average curvature: {avg_curvature}, Is left turn: {is_left_turn}")

        return is_left_turn

if __name__ == '__main__':
    try:
        local_path_publisher = local_path_pub()
    except rospy.ROSInterruptException:
        pass