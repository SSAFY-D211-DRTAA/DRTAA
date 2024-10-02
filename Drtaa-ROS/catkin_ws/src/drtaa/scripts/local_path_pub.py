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
        rospy.Subscriber('/global_path_per', Path, self.global_path_callback)

        # Publishers
        self.local_path_pub = rospy.Publisher('/local_path', Path, queue_size=1)
        self.left_turn_pub = rospy.Publisher('/is_left_turn', Bool, queue_size=1)
        
        # Initialization
        self.is_odom = False
        self.is_path = False
        self.x = 0
        self.y = 0
        self.heading = 0  # Added initialization for heading
        self.local_path_size = 70
        self.prev_current_waypoint = 0
        self.smoothing_factor = 0.5
        self.global_path_msg = None  # Added initialization for global_path_msg

        rate = rospy.Rate(15)  # 20hz
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
        if self.is_odom:
            self.prev_current_waypoint = self.find_closest_waypoint(self.x, self.y, start_index=0)
            rospy.loginfo(f"Global path updated and prev_current_waypoint set to {self.prev_current_waypoint}")

    def update_local_path(self):
        if self.global_path_msg is None or len(self.global_path_msg.poses) == 0:
            rospy.logwarn("Global path is empty or not received yet.")
            return

        local_path_msg = Path()
        local_path_msg.header.frame_id = 'map'
        
        current_waypoint = self.find_closest_waypoint(self.x, self.y, start_index=self.prev_current_waypoint)

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

    # def find_closest_waypoint(self, x, y, start_index):
    #     global pose
        
    #     if self.global_path_msg is None or len(self.global_path_msg.poses) == 0:
    #         rospy.logwarn("Global path is empty or not received yet.")
    #         return start_index

    #     min_dis = float('inf')
    #     current_waypoint = start_index
    #     for i in range(start_index, len(self.global_path_msg.poses)):
    #         pose = self.global_path_msg.poses[i]
    #         dis = sqrt((x - pose.pose.position.x)**2 + (y - pose.pose.position.y)**2)
    #         if dis < min_dis:
    #             min_dis = dis
    #             current_waypoint = i
    #         elif dis > min_dis + 5.0:
    #             break

    #     angle_to_waypoint = atan2(pose.pose.position.y - y, pose.pose.position.x - x)
    #     angle_difference = (angle_to_waypoint - self.heading + np.pi) % (2 * np.pi) - np.pi
        
    #     if angle_difference < -0.1 or angle_difference > 0.1:
    #         return self.prev_current_waypoint

    #     return current_waypoint
    
    def find_closest_waypoint(self, x, y, start_index):
        
        current_waypoint = start_index

        if self.global_path_msg is not None and len(self.global_path_msg.poses) != 0:
            min_dis = float('inf')
            current_waypoint = start_index
            for i in range(start_index, len(self.global_path_msg.poses)):
                pose = self.global_path_msg.poses[i]
                dis = sqrt((x - pose.pose.position.x)**2 + (y - pose.pose.position.y)**2)
                if dis < min_dis:
                    min_dis = dis
                    current_waypoint = i
                elif dis > min_dis + 5.0:
                    break

            angle_to_waypoint = atan2(pose.pose.position.y - y, pose.pose.position.x - x)
            angle_difference = (angle_to_waypoint - self.heading + np.pi) % (2 * np.pi) - np.pi
            
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

        is_left_turn = avg_curvature > 0.01

        rospy.loginfo(f"Average curvature: {avg_curvature}, Is left turn: {is_left_turn}")

        return is_left_turn

if __name__ == '__main__':
    try:
        local_path_publisher = local_path_pub()
    except rospy.ROSInterruptException:
        pass