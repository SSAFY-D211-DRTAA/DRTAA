#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import rospkg
from math import sqrt, atan2
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Bool, String
from tf.transformations import euler_from_quaternion
import numpy as np
from scipy.interpolate import interp1d
from scipy.spatial import KDTree

class LocalPathPublisher:
    def __init__(self):
        rospy.init_node('local_path_pub', anonymous=True)

        # Subscribers
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/global_path', Path, self.global_path_callback)
        rospy.Subscriber('/command_status', String, self.command_status_callback)
        rospy.Subscriber("/is_changing_lane", Bool, self.change_lane_callback)

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
        self.is_changing_lane = False
        self.lane_change_progress = 0
        self.lane_change_distance = 5.0
        self.vehicle_status = None
        self.kdtree = None
        self.global_path_points = None
        self.global_path_update_threshold = 10
        self.max_distance_from_path = 10.0

        rate = rospy.Rate(20)  # 20hz
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
        if not msg.poses:
            rospy.logwarn("Received empty global path")
            return

        self.is_path = True
        new_global_path = msg

        if self.global_path_msg is not None:
            old_closest = self.find_closest_waypoint(self.x, self.y, self.global_path_msg.poses)
            new_closest = self.find_closest_waypoint(self.x, self.y, new_global_path.poses)

            old_point = self.global_path_msg.poses[old_closest].pose.position
            new_point = new_global_path.poses[new_closest].pose.position

            distance = sqrt((old_point.x - new_point.x)**2 + (old_point.y - new_point.y)**2)

            if distance > self.global_path_update_threshold:
                self.prev_current_waypoint = new_closest
                rospy.loginfo(f"Global path significantly changed. Distance: {distance:.2f}m. Resetting prev_current_waypoint to {self.prev_current_waypoint}")
            else:
                rospy.loginfo(f"Global path updated but not significantly changed. Distance: {distance:.2f}m. Keeping current waypoint.")
        else:
            self.prev_current_waypoint = self.find_closest_waypoint(self.x, self.y, new_global_path.poses)
            rospy.loginfo(f"Initial global path received. Setting prev_current_waypoint to {self.prev_current_waypoint}")

        # Update global path and KD-Tree
        self.global_path_msg = new_global_path
        self.global_path_points = np.array([[pose.pose.position.x, pose.pose.position.y] for pose in self.global_path_msg.poses])
        
        try:
            self.kdtree = KDTree(self.global_path_points)
            rospy.loginfo(f"KDTree initialized with {len(self.global_path_points)} points")
        except Exception as e:
            rospy.logerr(f"Failed to initialize KDTree: {e}")
            self.kdtree = None

        # Immediately update the local path
        self.update_local_path()

    def command_status_callback(self, msg):
        self.vehicle_status = msg.data

    def change_lane_callback(self, msg):
        self.is_changing_lane = msg.data
        if self.is_changing_lane:
            self.lane_change_progress = 0

    def update_local_path(self):
        if not self.is_odom or not self.is_path or self.global_path_msg is None or len(self.global_path_msg.poses) == 0:
            rospy.logwarn("Odometry or global path not ready.")
            return

        try:
            local_path_msg = Path()
            local_path_msg.header.frame_id = 'map'

            current_waypoint = self.find_closest_waypoint(self.x, self.y, self.global_path_msg.poses, start_index=self.prev_current_waypoint)

            waypoint_pose = self.global_path_msg.poses[current_waypoint].pose.position
            distance_to_waypoint = sqrt((self.x - waypoint_pose.x)**2 + (self.y - waypoint_pose.y)**2)

            rospy.loginfo(f"Distance to waypoint: {distance_to_waypoint:.2f}m")

            if distance_to_waypoint > self.max_distance_from_path:
                rospy.logwarn(f"Selected waypoint is too far: {distance_to_waypoint:.2f}m. Finding closest point on global path.")
                current_waypoint = self.find_closest_waypoint_by_global(self.x, self.y)

            smoothed_waypoint = int(self.smoothing_factor * self.prev_current_waypoint + 
                                    (1 - self.smoothing_factor) * current_waypoint)

            if smoothed_waypoint + self.local_path_size < len(self.global_path_msg.poses):
                local_path_msg.poses = self.global_path_msg.poses[smoothed_waypoint:smoothed_waypoint + self.local_path_size]
            else:
                local_path_msg.poses = self.global_path_msg.poses[smoothed_waypoint:]

            # Update previous waypoint for next iteration
            self.prev_current_waypoint = smoothed_waypoint

            # Publish local path
            is_turning_left = self.predict_left_turn(local_path_msg)
            self.left_turn_pub.publish(Bool(is_turning_left))
            self.local_path_pub.publish(local_path_msg)

            rospy.loginfo(f"Local path published with {len(local_path_msg.poses)} poses")

        except Exception as e:
            rospy.logerr(f"Failed to update local path: {e}")

    def generate_lane_change_path(self, start_index):
        lane_change_path = []
        path_length = 30  # 차선 변경 경로의 길이 (미터)

        for i in range(path_length):
            original_pose = self.global_path_msg.poses[start_index + i].pose
            new_pose = PoseStamped()
            new_pose.header = self.global_path_msg.poses[start_index + i].header

            t = i / path_length
            lateral_offset = self.lane_change_distance * (10 * t**3 - 15 * t**4 + 6 * t**5)

            new_pose.pose.position.x = original_pose.position.x - lateral_offset * np.sin(self.heading)
            new_pose.pose.position.y = original_pose.position.y + lateral_offset * np.cos(self.heading)
            new_pose.pose.position.z = original_pose.position.z
            new_pose.pose.orientation = original_pose.orientation

            lane_change_path.append(new_pose)

        for i in range(path_length, path_length + 20):
            idx = min(start_index + i, len(self.global_path_msg.poses) - 1)
            lane_change_path.append(self.global_path_msg.poses[idx])

        return lane_change_path

    def find_closest_waypoint(self, x, y, poses, start_index=0):
        if self.kdtree is None:
            rospy.logwarn("KDTree is not initialized. Using linear search.")
            return self._linear_search_closest_waypoint(x, y, poses, start_index)

        distances, indices = self.kdtree.query([x, y], k=10)
        index = indices[0] if isinstance(indices, np.ndarray) else indices

        if index >= len(poses):
            rospy.logwarn(f"KDTree returned invalid index {index}. Falling back to linear search.")
            return self._linear_search_closest_waypoint(x, y, poses, start_index)
        
        max_correction_attempts = min(20, len(poses))
        for _ in range(max_correction_attempts):
            pose = poses[index]
            angle_to_waypoint = atan2(pose.pose.position.y - y, pose.pose.position.x - x)
            angle_difference = (angle_to_waypoint - self.heading + np.pi) % (2 * np.pi) - np.pi
            
            # if -np.pi/2 <= angle_difference <= np.pi/2 and abs(index - self.prev_current_waypoint) < 50:
            #     return index
            if np.any(-np.pi/2 <= angle_difference) and np.any(angle_difference <= np.pi/2):
                return index
            
            index = (index + 1) % len(poses)
        
        rospy.logwarn("Failed to find suitable waypoint. Falling back to linear search.")
        return self._linear_search_closest_waypoint(x, y, poses, start_index)

    def _linear_search_closest_waypoint(self, x, y, poses, start_index):
        min_dist = float('inf')
        closest_index = start_index
        search_range = 100

        for i in range(start_index, min(len(poses), start_index + search_range)):
            pose = poses[i]
            dist = sqrt((x - pose.pose.position.x)**2 + (y - pose.pose.position.y)**2)
            if dist < min_dist:
                min_dist = dist
                closest_index = i

        return closest_index

    def find_closest_waypoint_by_global(self, x, y):
        if self.kdtree is None:
            rospy.logwarn("KDTree is not initialized. Using linear search.")
            return self._linear_search_closest_waypoint(x, y, self.global_path_msg.poses)

        # KDTree를 사용하여 가장 가까운 점 찾기
        distances, indices = self.kdtree.query([x, y], k=10)
        
        # 찾은 점이 차량의 진행 방향과 일치하는지 확인
        max_correction_attempts = 20
        original_index = indices[0] if isinstance(indices, np.ndarray) else indices

        for _ in range(max_correction_attempts):
            pose = self.global_path_msg.poses[original_index]
            angle_to_waypoint = atan2(pose.pose.position.y - y, pose.pose.position.x - x)
            angle_difference = (angle_to_waypoint - self.heading + np.pi) % (2 * np.pi) - np.pi

            # 각도 차이가 허용 범위 내에 있으면 해당 인덱스 반환
            if np.any(-np.pi/2 <= angle_difference) and np.any(angle_difference <= np.pi/2):
                return original_index

            # 조건을 만족하지 않으면 다음 점 확인
            original_index = (original_index + 1) % len(self.global_path_msg.poses)

        # 적절한 점을 찾지 못했을 경우, 가장 가까운 점 반환
        rospy.logwarn("Failed to find suitable waypoint. Using closest point.")
        return indices[0] if isinstance(indices, np.ndarray) else indices

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
        local_path_publisher = LocalPathPublisher()
    except rospy.ROSInterruptException:
        pass