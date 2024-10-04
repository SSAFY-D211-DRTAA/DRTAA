#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import json
import numpy as np
import tf
import rospy
import rospkg
from math import cos, sin, pi, sqrt, pow, atan2
from geometry_msgs.msg import Point, PoseWithCovarianceStamped, PoseStamped
from nav_msgs.msg import Odometry, Path
from morai_msgs.msg import CtrlCmd, EgoVehicleStatus, ObjectStatusList, GetTrafficLightStatus
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import Bool, String

class TrafficLightManager:
    def __init__(self):
        self.traffic_light_status = -1
        self.traffic_light_type = -1
        self.last_update_time = rospy.Time.now()
        self.timeout_duration = rospy.Duration(1.0)  # 1초 타임아웃
        rospy.Subscriber("/GetTrafficLightStatus", GetTrafficLightStatus, self.traffic_light_callback)

    def traffic_light_callback(self, data):
        self.traffic_light_status = data.trafficLightStatus
        self.traffic_light_type = data.trafficLightType
        self.last_update_time = rospy.Time.now()
    
    def is_data_available(self):
        return (rospy.Time.now() - self.last_update_time) < self.timeout_duration

    def can_turn_left(self):
        # Check if a left turn is allowed
        return (((self.traffic_light_status & 32) != 0 and (self.traffic_light_type in [1, 2])) or ((self.traffic_light_status & 16) !=0 and (self.traffic_light_type == 0)))

    def can_go_straight(self):
        # Check if going straight is allowed
        return (((self.traffic_light_status & 16) != 0 and (self.traffic_light_type in [0, 2])) or ((self.traffic_light_status & 32) != 0 and (self.traffic_light_type == 1)))

class pure_pursuit:
    def __init__(self):
        rospy.init_node('pure_pursuit', anonymous=True)

        rospy.Subscriber("/global_path", Path, self.global_path_callback)
        # rospy.Subscriber("/lattice_path", Path, self.path_callback)
        rospy.Subscriber("/local_path", Path, self.path_callback)
        # rospy.Subscriber("/lane_change_path", Path, self.path_callback)

        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self.status_callback)
        rospy.Subscriber("/Object_topic", ObjectStatusList, self.object_info_callback)
        # rospy.Subscriber("/Object_topic_to_lidar", ObjectStatusList, self.object_info_callback) => 추후에 npc, ped, obs 구별하는 로직 및 기둥, 중앙분리대 같은 것은 해당 리스트에 포함 안되도록 해야함!!!
        
        # 차량 상태 확인
        rospy.Subscriber('/command_status', String, self.command_callback)

        # 좌회전 여부 확인
        rospy.Subscriber("/is_left_turn", Bool, self.turning_left_callback)

        # 승객 하차 시에 우선 적으로 True로 설정 되어야함!!! -> 일단은 주차장으로 이동해야하는 상황
        rospy.Subscriber("/is_parking", Bool, self.parking_status_callback)  

        rospy.Subscriber("/complete_drive", PoseStamped, self.complete_drive_callback)
        
        self.ctrl_cmd_pub = rospy.Publisher('/ctrl_cmd', CtrlCmd, queue_size=1) # 제어 메시지 발행
       
        rospy.Subscriber("is_roaming", Bool, self.roaming_callback)


        self.ctrl_cmd_msg = CtrlCmd()
        self.ctrl_cmd_msg.longlCmdType = 1

        self.is_path = False
        self.is_odom = False
        self.is_status = False
        self.is_global_path = False
        self.is_look_forward_point = False
        self.is_parking = False
        self.is_complete_drive = False

        self.forward_point = Point()
        # self.current_position = Point()

        self.velocity_list = [] 
        self.previous_global_path = None  # 이전 경로 저장 변수 추가
        self.local_path_point = None
 
        self.vehicle_length = 2.7
        self.lfd = 8
        self.min_lfd = 5
        self.max_lfd = 15
        self.lfd_gain = 0.78
        self.target_velocity = 35 # 70
        self.stop_line_threshold = 15  ## 정지선 감지 거리 

        self.nodes = self.load_nodes()

        self.pid = pidControl()
        self.adaptive_cruise_control = AdaptiveCruiseControl(velocity_gain=0.5, distance_gain=1, time_gap=0.8, vehicle_length=2.7)
        self.vel_planning = velocityPlanning(self.target_velocity / 3.6, 0.3) # 0.15 0.5
        self.traffic_light_manager = TrafficLightManager()

        rate = rospy.Rate(20)  ## 30hz

        while not rospy.is_shutdown():

            if self.is_complete_drive:
                self.stop_vehicle()
                rospy.loginfo("Drive completed. Stopping vehicle.")
                rate.sleep()
                continue
            
            if self.is_path and self.is_odom and self.is_status and len(self.velocity_list) > 0: 

                result = self.calc_vaild_obj([self.current_position.x, self.current_position.y, self.vehicle_yaw], self.object_data) # 주변 객체 정보 계산
                global_npc_info, local_npc_info, global_ped_info, local_ped_info, global_obs_info, local_obs_info = result

                self.current_waypoint = self.get_current_waypoint([self.current_position.x, self.current_position.y], self.global_path) # 현재 위치에서 가장 가까운 경로 상의 waypoint 인덱스
                #self.target_velocity = self.velocity_list[self.current_waypoint] * 3.6 # 현재 waypoint에 대한 목표 속도
                # current_waypoint가 velocity_list의 범위 내에 있는지 확인
                if 0 <= self.current_waypoint < len(self.velocity_list):
                    self.target_velocity = self.velocity_list[self.current_waypoint] * 3.6
                else:
                    rospy.logwarn("Current waypoint is out of velocity list range. Stopping vehicle.")
                    self.ctrl_cmd_msg.accel = 0.0
                    self.ctrl_cmd_msg.brake = 1.0
                    self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)
                    continue

                steering = self.calc_pure_pursuit() # pure_pursuit 알고리즘을 통해 조향각 계산
                self.trafficlight_logic() # 신호등 및 좌회전 여부에 따른 제어 로직

                if self.is_look_forward_point: # 전방 경로 상에 waypoint가 존재하는 경우
                    self.ctrl_cmd_msg.steering = steering

                else: # 전방에 waypoint가 없는 경우 (경로 끝에 도달한 경우)
                    self.stop_vehicle()

                self.adaptive_cruise_control.check_object(self.path ,global_npc_info, local_npc_info
                                                                    ,global_ped_info, local_ped_info
                                                                    ,global_obs_info, local_obs_info) # 주변 객체 정보를 통해 ACC를 위한 정보 업데이트
                
                self.acc_velocity = self.adaptive_cruise_control.get_target_velocity(local_npc_info, local_ped_info, local_obs_info,
                                                                                    self.status_msg.velocity.x, self.target_velocity/3.6) # ACC를 통한 속도 제어

                if(self.acc_velocity < 0):  # 비상 정지
                    self.ctrl_cmd_msg.accel = 0.0
                    self.ctrl_cmd_msg.brake = 1.0
                    self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)
                    rospy.logwarn("Emergency stop initiated!")
                    continue

                # ACC와 조향각 기반 속도 제한 중 더 낮은 값 선택
                self.target_velocity = min(self.target_velocity, self.acc_velocity) 
                # rospy.loginfo(f"Target Velocity: {self.target_velocity}")
                output = self.pid.pid(self.target_velocity, self.status_msg.velocity.x * 3.6) # PID 제어를 통한 가속도 계산

                if output > 0.0: # 가속도
                    self.ctrl_cmd_msg.accel = output
                    self.ctrl_cmd_msg.brake = 0.0
                else: # 감속도
                    self.ctrl_cmd_msg.accel = 0.0
                    self.ctrl_cmd_msg.brake = -output
        
                self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)

            rate.sleep()

    def path_callback(self, msg):
        self.is_path = True
        self.path = msg

    def odom_callback(self, msg):
        self.is_odom = True
        odom_quaternion = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        _, _, self.vehicle_yaw = euler_from_quaternion(odom_quaternion)
        self.current_position = msg.pose.pose.position
        # self.current_position.x = msg.pose.pose.position.x
        # self.current_position.y = msg.pose.pose.position.y

    def status_callback(self, msg):
        self.is_status = True
        self.status_msg = msg

    def global_path_callback(self, msg):
        # 새로운 경로와 이전 경로 비교
        if self.previous_global_path is None or not self.is_same_path(self.previous_global_path, msg):
            self.global_path = msg
            self.is_global_path = True
            self.velocity_list = self.vel_planning.curvedBaseVelocity(self.global_path, 50)
            self.is_complete_drive = False
            rospy.loginfo("Global path updated and velocity list recalculated")
            self.previous_global_path = msg  # 이전 경로 업데이트
    
    def parking_status_callback(self, msg):
        self.is_parking = msg.data
        rospy.loginfo(f"Parking status updated: {self.is_parking}")

    def complete_drive_callback(self, msg):
        self.is_complete_drive = True

    def roaming_callback(self, msg):
        if msg.data:
            self.is_complete_drive = False
            
    def stop_vehicle(self):
        self.ctrl_cmd_msg.accel = 0.0
        self.ctrl_cmd_msg.brake = 1.0
        self.ctrl_cmd_msg.steering = 0.0
        self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)

    def is_same_path(self, path1, path2):
        # 두 경로가 동일한지 비교하는 함수
        if len(path1.poses) != len(path2.poses):
            return False
        for pose1, pose2 in zip(path1.poses, path2.poses):
            if pose1.pose.position.x != pose2.pose.position.x or pose1.pose.position.y != pose2.pose.position.y:
                return False
        return True

    def object_info_callback(self, data):
        self.is_object_info = True
        self.object_data = data
        # rospy.loginfo(f"Object info updated: {len(self.object_data.npc_list)} NPCs, {len(self.object_data.pedestrian_list)} pedestrians, {len(self.object_data.obstacle_list)} obstacles")

    def turning_left_callback(self, msg):
        self.is_turning_left = msg.data # Bool은 data로 값을 가져옴!!
        # rospy.loginfo(f"Left turn signal: {self.is_turning_left}")
    
    def command_callback(self, msg):
        rospy.loginfo(f'차량 상태: {msg.data}')
        
    def load_nodes(self): # 노드 정보 로드
        current_path = os.path.dirname(os.path.abspath(__file__))
        node_set_path = os.path.join(current_path, 'lib', 'mgeo_data', 'R_KR_PR_Sangam_NoBuildings', 'node_set.json')
        with open(node_set_path, 'r') as file:
            return json.load(file)
    
    # def extract_stop_lines(self): # 정지선 정보 추출
    #     self.stop_lines = [node for node in self.nodes if node['on_stop_line']]

    def calculate_distance(self, point1, point2):
        return sqrt((point1.x - point2[0])**2 + (point1.y - point2[1])**2)

    # def update_nearest_stop_line(self): # 가장 가까운 정지선 정보 업데이트
    #     self.nearest_stop_line = min(self.stop_lines, key=lambda node: 
    #         self.calculate_distance(self.current_position, node['point']))

    def detect_stop_line(self): # 정지선 감지
        for node in self.nodes:
            if node.get('on_stop_line'):
                distance = self.calculate_distance(self.current_position, node['point'])
                if distance < self.stop_line_threshold:
                    return True, distance
        return False, float('inf')
    
    def get_current_waypoint(self, ego_status, global_path):
        min_dist = float('inf')
        current_waypoint = -1
        for i, pose in enumerate(global_path.poses):
            dx = ego_status[0] - pose.pose.position.x
            dy = ego_status[1] - pose.pose.position.y
            dist = sqrt(pow(dx, 2) + pow(dy, 2))
            if min_dist > dist:
                min_dist = dist
                current_waypoint = i
        return current_waypoint
        
    def trafficlight_logic(self):

        try:
            if not self.traffic_light_manager.is_data_available():
                # rospy.loginfo("No traffic light data available, proceeding with caution")
                return  # 신호등 정보가 없으면 함수를 종료하고 기본 주행 로직을 따름
            
            is_near_stop_line, distance_to_stop_line = self.detect_stop_line()
            
            if is_near_stop_line: # 정지선 근처인 경우
                # rospy.loginfo(f"Near stop line, distance: {distance_to_stop_line:.2f}")
                
                if self.is_turning_left: # 좌회전을 해야할 경우
                    if self.traffic_light_manager.can_turn_left(): # 좌회전 가능
                        #rospy.loginfo("Left turn signal on, proceeding with left turn")
                        return
                    else: # 좌회전 금지
                        #rospy.loginfo("Left turn signal on, but left turn not allowed")
                        self.target_velocity = self.calculate_approach_velocity(distance_to_stop_line)
                    #self.target_velocity = self.normal_speed  # Set speed for left turn
                else: # 직진
                    if self.traffic_light_manager.traffic_light_status & 1:  # Red light
                        #rospy.loginfo("Red light detected, stopping")
                        self.target_velocity = self.calculate_approach_velocity(distance_to_stop_line)
                        
                    elif self.traffic_light_manager.traffic_light_status & 4:  # Yellow light
                        #rospy.loginfo("Yellow light detected, slowing down")
                        approach_velocity = self.calculate_approach_velocity(distance_to_stop_line)
                        self.target_velocity = min(approach_velocity, self.target_velocity)
                        
                    elif self.traffic_light_manager.can_go_straight():  # Green light
                        #rospy.loginfo("Green light detected, proceeding")
                        return
        except Exception as e:
            rospy.logerr(f"Error in trafficlight_logic: {e}")
            self.target_velocity = 0

    def calculate_approach_velocity(self, distance):
        max_approach_speed = 35  ## km/h # 20
        min_approach_speed = 0   # km/h
        
        if distance > self.stop_line_threshold:
            return max_approach_speed
        elif distance < 5:
            return min_approach_speed
        else:
            return max(min_approach_speed, (distance / self.stop_line_threshold) * max_approach_speed)
        
    def calc_vaild_obj(self, status_msg, object_data):

        self.all_object = object_data        
        ego_pose_x, ego_pose_y, ego_heading = status_msg
        # rospy.loginfo(f"heading: {ego_heading}")
        
        global_npc_info = []
        local_npc_info  = []
        global_ped_info = []
        local_ped_info  = []
        global_obs_info = []
        local_obs_info  = []

        num_of_object = self.all_object.num_of_npcs + self.all_object.num_of_obstacle + self.all_object.num_of_pedestrian    

        if num_of_object > 0:
            
            # 변환 행렬 계산 (한 번만 수행)
            cos_theta = np.cos(ego_heading)
            sin_theta = np.sin(ego_heading)
            tmp_det_t = np.array([
                [cos_theta, sin_theta, -(cos_theta * ego_pose_x + sin_theta * ego_pose_y)],
                [-sin_theta, cos_theta, (sin_theta * ego_pose_x - cos_theta * ego_pose_y)],
                [0, 0, 1]
            ])

            # NPC 차량 변환
            if self.all_object.npc_list:
                npc_positions = np.array([[npc.position.x, npc.position.y, 1] for npc in self.all_object.npc_list]).T
                local_results = tmp_det_t.dot(npc_positions)
                for i, npc in enumerate(self.all_object.npc_list):
                    if local_results[0, i] > 0:
                        global_npc_info.append([npc.type, npc.position.x, npc.position.y, npc.velocity.x])
                        local_npc_info.append([npc.type, local_results[0, i], local_results[1, i], npc.velocity.x])

            # 보행자 변환
            if self.all_object.pedestrian_list:
                ped_positions = np.array([[ped.position.x, ped.position.y, 1] for ped in self.all_object.pedestrian_list]).T
                local_results = tmp_det_t.dot(ped_positions)
                for i, ped in enumerate(self.all_object.pedestrian_list):
                    if 0 < local_results[0, i] < 50:
                        global_ped_info.append([ped.type, ped.position.x, ped.position.y, ped.velocity.x])
                        local_ped_info.append([ped.type, local_results[0, i], local_results[1, i], ped.velocity.x])

            # 장애물 변환
            if self.all_object.obstacle_list:
                obs_positions = np.array([[obs.position.x, obs.position.y, 1] for obs in self.all_object.obstacle_list]).T
                local_results = tmp_det_t.dot(obs_positions)
                for i, obs in enumerate(self.all_object.obstacle_list):
                    if 0 < local_results[0, i] < 50:
                        global_obs_info.append([obs.type, obs.position.x, obs.position.y, obs.velocity.x])
                        local_obs_info.append([obs.type, local_results[0, i], local_results[1, i], obs.velocity.x])

        return global_npc_info, local_npc_info, global_ped_info, local_ped_info, global_obs_info, local_obs_info
    
    def calc_pure_pursuit(self):
        try:
            is_near_stop_line, distance_to_stop_line = self.detect_stop_line()
            if is_near_stop_line:
                self.lfd = max(self.min_lfd, min(distance_to_stop_line, self.lfd))
            else:
                self.lfd = self.status_msg.velocity.x * self.lfd_gain
            # rospy.loginfo(f"Look-ahead Distance: {self.lfd}")

            vehicle_position = self.current_position
            self.is_look_forward_point = False

            translation = [vehicle_position.x, vehicle_position.y]
            
            trans_matrix = np.array([
                [cos(self.vehicle_yaw), -sin(self.vehicle_yaw), translation[0]],
                [sin(self.vehicle_yaw), cos(self.vehicle_yaw), translation[1]],
                [0, 0, 1]
            ])
            
            det_trans_matrix = np.linalg.inv(trans_matrix)

            for num, i in enumerate(self.path.poses):
                path_point = np.array([i.pose.position.x, i.pose.position.y, 1])
                self.local_path_point = det_trans_matrix.dot(path_point)
                
                if self.local_path_point[0] > 0:
                    dis = sqrt(pow(self.local_path_point[0], 2) + pow(self.local_path_point[1], 2))
                    if dis >= self.lfd:
                        self.forward_point = i.pose.position
                        self.is_look_forward_point = True
                        break

            theta = atan2(self.local_path_point[1], self.local_path_point[0])
            steering = atan2(2 * self.vehicle_length * sin(theta), self.lfd)

            # 조향각에 따른 속도 제한
            max_steering_angle = pi/4  # 45도
            steering_ratio = abs(steering) / max_steering_angle
            if steering_ratio > 0.5:  # 조향각이 최대의 50% 이상일 때
                self.target_velocity = min(self.target_velocity, 10)  # 10km/h로 제한
                
        except Exception as e:
            rospy.logerr(f"Error in pure pursuit: {e}")
            return 0.0
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
        out_vel_plan = [self.car_max_speed] * len(global_path.poses)
        for i in range(point_num, len(global_path.poses) - point_num):
            x_list = [[-2*global_path.poses[i+box].pose.position.x,
                       -2*global_path.poses[i+box].pose.position.y,
                       1] for box in range(-point_num, point_num)]
            y_list = [-(global_path.poses[i+box].pose.position.x**2 +
                        global_path.poses[i+box].pose.position.y**2)
                      for box in range(-point_num, point_num)]
            
            x_matrix = np.array(x_list)
            y_matrix = np.array(y_list)
            x_inverse = np.linalg.pinv(x_matrix)
            result = x_inverse.dot(y_matrix)
            
            r = sqrt(result[0]**2 + result[1]**2 - result[2])

            v_max = min(sqrt(r * 9.8 * self.road_friction), self.car_max_speed)
            out_vel_plan[i] = v_max

        return out_vel_plan

class AdaptiveCruiseControl:
    def __init__(self, velocity_gain, distance_gain, time_gap, vehicle_length):
        self.npc_vehicle = [False, 0]
        self.object = [False, 0]
        self.Person = [False, 0]
        self.velocity_gain = velocity_gain
        self.distance_gain = distance_gain
        self.time_gap = time_gap
        self.vehicle_length = vehicle_length
        self.object_type = None
        self.object_distance = 0
        self.object_velocity = 0
        self.emergency_distance = 3  # 비상 정지 거리 (미터)
        
        # self.check_object_pub = rospy.Publisher('/check_object', Bool, queue_size=1)

    def check_object(self, ref_path, global_npc_info, local_npc_info, global_ped_info, local_ped_info, global_obs_info, local_obs_info):
        # 주행 경로 상의 장애물 유무 확인
        min_rel_distance = float('inf')
        self.npc_vehicle = [False, 0]
        self.Person = [False, 0]
        self.object = [False, 0]
        
        # 보행자 확인
        if len(global_ped_info) > 0:
            for i in range(len(global_ped_info)):
                for path in ref_path.poses:
                    if global_ped_info[i][0] == 0:  # type=0 [pedestrian]
                        dis = sqrt(pow(path.pose.position.x - global_ped_info[i][1], 2) + pow(path.pose.position.y - global_ped_info[i][2], 2))
                        if dis < 2.35:
                            rel_distance = sqrt(pow(global_ped_info[i][1], 2) + pow(global_ped_info[i][2], 2))
                            if rel_distance < min_rel_distance:
                                min_rel_distance = rel_distance
                                self.Person = [True, i]

        # NPC 차량 확인
        if len(global_npc_info) > 0:
            for i in range(len(global_npc_info)):
                for path in ref_path.poses:
                    if global_npc_info[i][0] == 1:  # type=1 [npc_vehicle]
                        dis = sqrt(pow(path.pose.position.x - global_npc_info[i][1], 2) + pow(path.pose.position.y - global_npc_info[i][2], 2))
                        if dis < 2.35:
                            rel_distance = sqrt(pow(global_npc_info[i][1], 2) + pow(global_npc_info[i][2], 2))
                            if rel_distance < min_rel_distance:
                                min_rel_distance = rel_distance
                                self.npc_vehicle = [True, i]
                                

        # 장애물 확인
        if len(global_obs_info) > 0:
            for i in range(len(global_obs_info)):
                for path in ref_path.poses :      
                    if global_obs_info[i][0] == 2 : # type=1 [obstacle] 
                        dis = sqrt(pow(path.pose.position.x - global_obs_info[i][1], 2) + pow(path.pose.position.y - global_obs_info[i][2], 2))
                        if dis < 2.35:
                            rel_distance = sqrt(pow(global_obs_info[i][1], 2) + pow(global_obs_info[i][2], 2))
                            if rel_distance < min_rel_distance:
                                min_rel_distance = rel_distance
                                self.object=[True,i] 

        # rospy.loginfo(f"ACC: NPC detected={self.npc_vehicle[0]}, Pedestrian detected={self.Person[0]}, Obstacle detected={self.object[0]}")

    
    def get_target_velocity(self, local_npc_info, local_ped_info, local_obs_info, ego_vel, target_vel): 
        #TODO: (9) 장애물과의 속도와 거리 차이를 이용하여 ACC 를 진행 목표 속도를 설정
        out_vel =  target_vel
        default_space = 8
        time_gap = self.time_gap
        v_gain = self.velocity_gain
        x_errgain = self.distance_gain
        dis_rel = float('inf')  # 초기값 설정

        try:
            # self.check_object_pub.publish(False)
            if self.npc_vehicle[0] and len(local_npc_info) != 0: #ACC ON_vehicle   
                front_vehicle = [local_npc_info[self.npc_vehicle[1]][1], local_npc_info[self.npc_vehicle[1]][2], local_npc_info[self.npc_vehicle[1]][3]]
                
                dis_safe = ego_vel * time_gap + default_space
                dis_rel = sqrt(pow(front_vehicle[0],2) + pow(front_vehicle[1],2))            
                vel_rel=((front_vehicle[2] / 3.6) - ego_vel)                        
                acceleration = vel_rel * v_gain - x_errgain * (dis_safe - dis_rel)

                # self.check_object_pub.publish(True)

                rospy.loginfo(f"NPC_Vehicle: {front_vehicle}, dis_rel: {dis_rel}, vel_rel: {vel_rel}, acceleration: {acceleration}")
                out_vel = ego_vel + acceleration      

            if self.Person[0] and len(local_ped_info) != 0: #ACC ON_Pedestrian
                Pedestrian = [local_ped_info[self.Person[1]][1], local_ped_info[self.Person[1]][2], local_ped_info[self.Person[1]][3]]
                
                dis_safe = ego_vel* time_gap + default_space
                dis_rel = sqrt(pow(Pedestrian[0],2) + pow(Pedestrian[1],2))            
                vel_rel = (Pedestrian[2] - ego_vel)              
                acceleration = vel_rel * v_gain - x_errgain * (dis_safe - dis_rel)    

                rospy.loginfo(f"Pedestrian: {Pedestrian}, dis_rel: {dis_rel}, vel_rel: {vel_rel}, acceleration: {acceleration}")
                out_vel = ego_vel + acceleration
    
            if self.object[0] and len(local_obs_info) != 0: #ACC ON_obstacle     
                Obstacle = [local_obs_info[self.object[1]][1], local_obs_info[self.object[1]][2], local_obs_info[self.object[1]][3]]
                
                dis_safe = ego_vel* time_gap + default_space
                dis_rel = sqrt(pow(Obstacle[0],2) + pow(Obstacle[1],2))            
                vel_rel = (Obstacle[2] - ego_vel)
                acceleration = vel_rel * v_gain - x_errgain * (dis_safe - dis_rel)    

                rospy.loginfo(f"Obstacle: {Obstacle}, dis_rel: {dis_rel}, vel_rel: {vel_rel}, acceleration: {acceleration}")
                out_vel = ego_vel + acceleration    

            # 비상 정지 로직
            if dis_rel <= self.emergency_distance:
              
                return -1

            return out_vel * 3.6    
        
        except Exception as e:
            rospy.logerr(f"Error in ACC: {e}")
            return 0
        
if __name__ == '__main__':
    try:
        test_track=pure_pursuit()
    except rospy.ROSInterruptException:
        pass