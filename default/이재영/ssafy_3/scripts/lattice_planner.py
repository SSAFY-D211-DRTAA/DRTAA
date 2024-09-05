#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os, sys
import rospy
from math import cos, sin, pi, sqrt, pow, atan2
from morai_msgs.msg import EgoVehicleStatus, ObjectStatusList
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import numpy as np

class latticePlanner:
    def __init__(self):
        rospy.init_node('lattice_planner', anonymous=True)
        arg = rospy.myargv(argv=sys.argv)
        object_topic_name = arg[1]

        # Subscriber and Publisher declarations
        rospy.Subscriber(object_topic_name, ObjectStatusList, self.object_callback)
        rospy.Subscriber("/local_path", Path, self.path_callback)
        rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self.status_callback)

        self.lattice_path_pub = rospy.Publisher('/lattice_path', Path, queue_size=1)

        self.is_path = False
        self.is_status = False
        self.is_obj = False

        self.local_path = None
        self.status_msg = None
        self.object_data = None

        rate = rospy.Rate(50)  # 50hz

        while not rospy.is_shutdown():
            if self.is_path and self.is_status and self.is_obj:
                rospy.loginfo("All data received, processing lattice path")
                if self.checkObject(self.local_path, self.object_data):
                    lattice_path = self.latticePlanner(self.local_path, self.status_msg)
                    lattice_path_index = self.collision_check(self.object_data, lattice_path)
                    # Publish the selected lattice path message
                    rospy.loginfo("Publishing selected lattice path")
                    self.lattice_path_pub.publish(lattice_path[lattice_path_index])
                else:
                    rospy.loginfo("No collision detected, publishing local path")
                    self.lattice_path_pub.publish(self.local_path)
            rate.sleep()

    def checkObject(self, ref_path, object_data):
        is_crash = False
        safety_distance = 3.0  # 기존 2.35에서 3.0으로 조정
        for obstacle in object_data.obstacle_list:
            for path in ref_path.poses:
                dis = sqrt(pow(obstacle.position.x - path.pose.position.x, 2) +
                           pow(obstacle.position.y - path.pose.position.y, 2))
                if dis < safety_distance:
                    is_crash = True
                    break
        return is_crash

    def collision_check(self, object_data, out_path):
        lane_weight = [3, 2, 1, 1, 2, 3]
        for obstacle in object_data.obstacle_list:
            for path_num in range(len(out_path)):
                for path_pos in out_path[path_num].poses:
                    dis = sqrt(pow(obstacle.position.x - path_pos.pose.position.x, 2) +
                               pow(obstacle.position.y - path_pos.pose.position.y, 2))
                    if dis < 2.0:  # 기존 1.5에서 2.0으로 조정
                        lane_weight[path_num] += 100
        selected_lane = lane_weight.index(min(lane_weight))
        return selected_lane

    def path_callback(self, msg):
        self.is_path = True
        self.local_path = msg
        rospy.loginfo("Local path received")

    def status_callback(self, msg):
        self.is_status = True
        self.status_msg = msg
        rospy.loginfo("Ego vehicle status received")

    def object_callback(self, msg):
        self.is_obj = True
        self.object_data = msg
        rospy.loginfo("Object data received")

    def latticePlanner(self, ref_path, vehicle_status):
        out_path = [Path() for _ in range(6)]
        vehicle_pose_x = vehicle_status.position.x
        vehicle_pose_y = vehicle_status.position.y
        vehicle_velocity = vehicle_status.velocity.x * 3.6
        look_distance = int(vehicle_velocity * 0.2 * 2)
        look_distance = max(look_distance, 20)

        if len(ref_path.poses) > look_distance:
            global_ref_start_point = (ref_path.poses[0].pose.position.x, ref_path.poses[0].pose.position.y)
            global_ref_start_next_point = (ref_path.poses[1].pose.position.x, ref_path.poses[1].pose.position.y)
            global_ref_end_point = (ref_path.poses[look_distance * 2].pose.position.x, ref_path.poses[look_distance * 2].pose.position.y)

            theta = atan2(global_ref_start_next_point[1] - global_ref_start_point[1],
                          global_ref_start_next_point[0] - global_ref_start_point[0])

            translation = [global_ref_start_point[0], global_ref_start_point[1]]
            trans_matrix = np.array([[cos(theta), -sin(theta), translation[0]],
                                     [sin(theta), cos(theta), translation[1]],
                                     [0, 0, 1]])

            det_trans_matrix = np.array([[trans_matrix[0][0], trans_matrix[1][0],
                                          -(trans_matrix[0][0] * translation[0] + trans_matrix[1][0] * translation[1])],
                                         [trans_matrix[0][1], trans_matrix[1][1],
                                          -(trans_matrix[0][1] * translation[0] + trans_matrix[1][1] * translation[1])],
                                         [0, 0, 1]])

            world_end_point = np.array([[global_ref_end_point[0]], [global_ref_end_point[1]], [1]])
            local_end_point = det_trans_matrix.dot(world_end_point)

            world_ego_vehicle_position = np.array([[vehicle_pose_x], [vehicle_pose_y], [1]])
            local_ego_vehicle_position = det_trans_matrix.dot(world_ego_vehicle_position)

            lane_off_set = [-3.0, -1.75, -1, 1, 1.75, 3.0]
            local_lattice_points = []

            for i in range(len(lane_off_set)):
                local_lattice_points.append([local_end_point[0][0], local_end_point[1][0] + lane_off_set[i], 1])

            for lane_num, end_point in enumerate(local_lattice_points):
                x_start = local_ego_vehicle_position[0][0]
                y_start = local_ego_vehicle_position[1][0]
                x_end = end_point[0]
                y_end = end_point[1]

                if x_start != x_end:  # Avoid division by zero
                    a = 2 * (y_start - y_end) / (x_start - x_end)**3
                    b = -3 * (y_start - y_end) / (x_start - x_end)**2
                else:
                    a = b = 0
                c = 0
                d = y_start

                num_points = 50
                for i in range(num_points):
                    t = float(i) / (num_points - 1)
                    x = x_start * (1 - t) + x_end * t
                    y = a * x**3 + b * x**2 + c * x + d

                    global_result = trans_matrix.dot(np.array([[x], [y], [1]]))
                    read_pose = PoseStamped()
                    read_pose.pose.position.x = global_result[0][0]
                    read_pose.pose.position.y = global_result[1][0]
                    read_pose.pose.position.z = 0
                    read_pose.pose.orientation.x = 0
                    read_pose.pose.orientation.y = 0
                    read_pose.pose.orientation.z = 0
                    read_pose.pose.orientation.w = 1
                    out_path[lane_num].poses.append(read_pose)

            add_point_size = min(int(vehicle_velocity * 2), len(ref_path.poses))
            for i in range(look_distance*2, add_point_size):
                if i+1 < len(ref_path.poses):
                    tmp_theta = atan2(ref_path.poses[i + 1].pose.position.y - ref_path.poses[i].pose.position.y,
                                      ref_path.poses[i + 1].pose.position.x - ref_path.poses[i].pose.position.x)
                    tmp_translation = [ref_path.poses[i].pose.position.x, ref_path.poses[i].pose.position.y]
                    tmp_t = np.array([[cos(tmp_theta), -sin(tmp_theta), tmp_translation[0]],
                                      [sin(tmp_theta), cos(tmp_theta), tmp_translation[1]],
                                      [0, 0, 1]])

                    for lane_num in range(len(lane_off_set)):
                        local_result = np.array([[0], [lane_off_set[lane_num]], [1]])
                        global_result = tmp_t.dot(local_result)

                        read_pose = PoseStamped()
                        read_pose.pose.position.x = global_result[0][0]
                        read_pose.pose.position.y = global_result[1][0]
                        read_pose.pose.position.z = 0
                        read_pose.pose.orientation.x = 0
                        read_pose.pose.orientation.y = 0
                        read_pose.pose.orientation.z = 0
                        read_pose.pose.orientation.w = 1
                        out_path[lane_num].poses.append(read_pose)

        return out_path

if __name__ == '__main__':
    try:
        latticePlanner()
    except rospy.ROSInterruptException:
        pass