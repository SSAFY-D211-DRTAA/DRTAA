#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import rospkg
from math import cos, sin, pi, sqrt, pow, atan2
from geometry_msgs.msg import Point, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, Path
from morai_msgs.msg import CtrlCmd, EgoVehicleStatus
import numpy as np
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class pure_pursuit:
    def __init__(self):
        rospy.init_node('pure_pursuit', anonymous=True)

        # Subscriber, Publisher 선언
        rospy.Subscriber("/global_path", Path, self.global_path_callback)
        rospy.Subscriber("local_path", Path, self.path_callback)
        rospy.Subscriber("odom", Odometry, self.odom_callback)
        rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self.status_callback)
        self.ctrl_cmd_pub = rospy.Publisher('/ctrl_cmd', CtrlCmd, queue_size=1)

        self.ctrl_cmd_msg = CtrlCmd()
        self.ctrl_cmd_msg.longlCmdType = 1

        self.is_path = False
        self.is_odom = False
        self.is_status = False
        self.is_global_path = False
        self.is_look_forward_point = False

        self.forward_point = Point()
        self.current_postion = Point()

        self.vehicle_length = 2.6
        self.lfd = 8
        self.min_lfd = 5
        self.max_lfd = 30
        self.lfd_gain = 0.78
        self.target_velocity = 60

        self.pid = pidControl()
        self.vel_planning = velocityPlanning(self.target_velocity / 3.6, 0.15)

        while True:
            if self.is_global_path:
                self.velocity_list = self.vel_planning.curvedBaseVelocity(self.global_path, 50)
                break
            else:
                rospy.loginfo('Waiting global path data')

        rate = rospy.Rate(30)  # 30hz
        while not rospy.is_shutdown():
            if self.is_path and self.is_odom and self.is_status:
                self.current_waypoint = self.get_current_waypoint(self.status_msg, self.global_path)
                self.target_velocity = self.velocity_list[self.current_waypoint] * 3.6

                steering = self.calc_pure_pursuit()
                if self.is_look_forward_point:
                    self.ctrl_cmd_msg.steering = steering
                else:
                    rospy.loginfo("no found forward point")
                    self.ctrl_cmd_msg.steering = 0.0

                output = self.pid.pid(self.target_velocity, self.status_msg.velocity.x * 3.6)

                if output > 0.0:
                    self.ctrl_cmd_msg.accel = output
                    self.ctrl_cmd_msg.brake = 0.0
                else:
                    self.ctrl_cmd_msg.accel = 0.0
                    self.ctrl_cmd_msg.brake = -output

                # 제어입력 메세지 Publish
                self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)

            rate.sleep()

    def path_callback(self, msg):
        self.is_path = True
        self.path = msg

    def odom_callback(self, msg):
        self.is_odom = True
        odom_quaternion = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        _, _, self.vehicle_yaw = euler_from_quaternion(odom_quaternion)
        self.current_postion.x = msg.pose.pose.position.x
        self.current_postion.y = msg.pose.pose.position.y

    def status_callback(self, msg):
        self.is_status = True
        self.status_msg = msg

    def global_path_callback(self, msg):
        self.global_path = msg
        self.is_global_path = True

    def get_current_waypoint(self, ego_status, global_path):
        min_dist = float('inf')
        current_waypoint = -1
        for i, pose in enumerate(global_path.poses):
            dx = ego_status.position.x - pose.pose.position.x
            dy = ego_status.position.y - pose.pose.position.y
            dist = sqrt(pow(dx, 2) + pow(dy, 2))
            if min_dist > dist:
                min_dist = dist
                current_waypoint = i
        return current_waypoint

    def calc_pure_pursuit(self):
        # 속도 비례 Look Ahead Distance 값 설정
        self.lfd = max(self.min_lfd, min(self.max_lfd, self.lfd_gain * self.status_msg.velocity.x))

        vehicle_position = self.current_postion
        self.is_look_forward_point = False

        translation = [vehicle_position.x, vehicle_position.y]

        # 좌표 변환 행렬 생성
        trans_matrix = np.array([[cos(self.vehicle_yaw), -sin(self.vehicle_yaw), translation[0]],
                                 [sin(self.vehicle_yaw), cos(self.vehicle_yaw), translation[1]],
                                 [0, 0, 1]])

        det_trans_matrix = np.linalg.inv(trans_matrix)

        for num, i in enumerate(self.path.poses):
            path_point = i.pose.position
            global_path_point = [path_point.x, path_point.y, 1]
            local_path_point = det_trans_matrix.dot(global_path_point)

            if local_path_point[0] > 0:
                dis = sqrt(local_path_point[0]**2 + local_path_point[1]**2)
                if dis >= self.lfd:
                    self.forward_point = path_point
                    self.is_look_forward_point = True
                    break

        # Steering 각도 계산
        theta = atan2(local_path_point[1], local_path_point[0])
        steering = atan2(2 * self.vehicle_length * sin(theta), dis)

        return steering

class pidControl:
    def __init__(self):
        self.p_gain = 0.3
        self.i_gain = 0.00
        self.d_gain = 0.03
        self.prev_error = 0
        self.i_control = 0
        self.controlTime = 0.02

    def pid(self, target_vel, current_vel):
        error = target_vel - current_vel

        # PID 제어 생성
        p_control = self.p_gain * error
        self.i_control += self.i_gain * error * self.controlTime
        d_control = self.d_gain * (error - self.prev_error) / self.controlTime

        output = p_control + self.i_control + d_control
        self.prev_error = error

        return output

class velocityPlanning:
    def __init__(self, car_max_speed, road_friction):
        self.car_max_speed = car_max_speed
        self.road_friction = road_friction

    def curvedBaseVelocity(self, global_path, point_num):
        out_vel_plan = []

        for i in range(0, point_num):
            out_vel_plan.append(self.car_max_speed)

        for i in range(point_num, len(global_path.poses) - point_num):
            x_list = []
            y_list = []

            for box in range(-point_num, point_num):
                x = global_path.poses[i + box].pose.position.x
                y = global_path.poses[i + box].pose.position.y
                x_list.append([-2 * x, -2 * y, 1])
                y_list.append((-x * x) - (y * y))

            # 도로의 곡률 계산
            A = np.array(x_list)
            B = np.array(y_list)
            X = np.linalg.lstsq(A, B, rcond=None)[0]
            r = sqrt(X[0]**2 + X[1]**2 + X[2])

            # 곡률 기반 속도 계획
            v_max = sqrt(r * 9.81 * self.road_friction)
            if v_max > self.car_max_speed:
                v_max = self.car_max_speed

            out_vel_plan.append(v_max)

        for i in range(len(global_path.poses) - point_num, len(global_path.poses) - 10):
            out_vel_plan.append(30)

        for i in range(len(global_path.poses) - 10, len(global_path.poses)):
            out_vel_plan.append(0)

        return out_vel_plan

if __name__ == '__main__':
    try:
        test_track = pure_pursuit()
    except rospy.ROSInterruptException:
        pass