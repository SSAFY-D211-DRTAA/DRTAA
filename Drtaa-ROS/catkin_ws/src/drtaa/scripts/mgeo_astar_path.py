#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import heapq
import rospy
import os
import sys
import copy
import tf
import numpy as np
from math import sqrt, pow, atan2
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Point
from nav_msgs.msg import Path, Odometry
from scipy.spatial import KDTree
from std_msgs.msg import String
from morai_msgs.srv import MoraiEventCmdSrv
from morai_msgs.msg import EventInfo
from std_msgs.msg import Bool
from morai_msgs.msg import CtrlCmd


current_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(current_path)
from lib.mgeo.class_defs import *

class AStarPathPublisher:
    def __init__(self):
        # ROS 노드 초기화 및 구독자, 발행자 설정
        rospy.init_node('astar_path_pub', anonymous=True)
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/command_status', String, self.command_callback)

        # 차량의 기어 등 상태를 제어하기 위한 서비스 클라이언트
        self.event_cmd_client = rospy.ServiceProxy('/Service_MoraiEventCmd', MoraiEventCmdSrv)

        self.global_path_pub = rospy.Publisher('/global_path', Path, queue_size=1)
        self.global_path_pub_per = rospy.Publisher('/global_path_per', Path, queue_size=1)


        # 목적지 도착 여부 
        self.destination_reached_pub = rospy.Publisher('/complete_drive', PoseStamped, queue_size=1)

        # 목적지 도착 후 승객 하차 여부
        rospy.Subscriber("/passenger_dropoff", Bool, self.dropoff_callback)  
        rospy.Subscriber("/is_roaming", Bool, self.roaming_callback)

        # 초기 상태 변수 설정
        self.is_goal_pose = False
        self.is_init_pose = False
        self.start_node = None
        self.end_node = None
        self.global_path_msg = None
        self.kd_tree = None
        self.vehicle_state = None
        
        # Flag to check if destination reached message is published
        self.current_position = None
        self.goal_pos = None
        self.is_global_path_once_pub = False

        self.last_dropoff_position = None
        self.is_prepared_roaming = False
        self.is_dropoff = False
        self.roaming_path = None
        self.replan_distance = 30  # 재사용할 경로의 거리 (미터)


        # 주차장 좌표 리스트
        self.parking_positions = [Point(x=1530.417358449311, y=-835.3657244807109, z=0.0)]

        try:
            # MGeo 데이터 로드
            load_path = os.path.normpath(os.path.join(current_path, 'lib/mgeo_data/R_KR_PR_Sangam_NoBuildings'))
            mgeo_planner_map = MGeo.create_instance_from_json(load_path)
            self.nodes = mgeo_planner_map.node_set.nodes
            self.links = mgeo_planner_map.link_set.lines
            self.build_kd_tree()
            self.path_cache = {}  # 경로 캐싱을 위한 딕셔너리
        except Exception as e:
            rospy.logerr(f"Failed to load Mgeo data: {e}")

        # A* 경로 계획기 초기화
        self.global_planner = AStar(self.nodes, self.links)

        # 메인 루프
        rate = rospy.Rate(10) # 1Hz
        while not rospy.is_shutdown():
            if self.global_path_msg is not None:
                # self.global_path_pub.publish(self.global_path_msg)
                # self.global_path_pub_per.publish(self.global_path_msg)

                if self.is_global_path_once_pub is not True:
                    self.global_path_pub.publish(self.global_path_msg)
                    self.is_global_path_once_pub = True

                    # Change ctrl_mode to automode
                    event_info = EventInfo()
                    event_info.option = 1  # ctrl_mode option
                    event_info.ctrl_mode = 3  # automode
                    self.event_cmd_client(event_info)

            rate.sleep()

    def goal_callback(self, msg):
        
        self.end_node = self.find_closest_node(msg.pose.position)
        self.is_goal_pose = True
        # self.is_global_path_once_pub = False
        rospy.loginfo(f"목적지 노드: {self.end_node}")
        # [INFO] [1727033826.562245]: 목적지 노드: A119AS319285

        if self.end_node in self.nodes:
            # 점 표기법을 사용하여 Node 객체의 point 속성에 접근
            node_point = self.nodes[self.end_node].point
            self.goal_pos = [node_point[0], node_point[1]]
            # rospy.loginfo(f"Goal position set to: {self.goal_pos}")
            # [INFO] [1727033826.564504]: Goal position set to: [1575.1212905439897, -891.3160112658516]
        else:
            rospy.logerr(f"End node {self.end_node} not found in nodes.")

    def odom_callback(self, msg):
        
        self.current_position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        _, _, yaw = tf.transformations.euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        self.current_heading = yaw

        if hasattr(self, 'goal_pos') and self.goal_pos is not None:
        
            if self.is_goal_pose: # 목표 위치가 설정되어 있을 때만 시작 위치 설정
                self.start_node = self.find_closest_node_in_direction(self.current_position, self.current_heading)
                # self.start_node = self.find_closest_node_in_direction(self.current_position, yaw)
                self.is_init_pose = True
                # rospy.loginfo(f"현재 위치 근접 노드: {self.start_node}")
                self.update_path()

            if self.destination_reached(self.current_position): # 목적지 도착 여부 확인
                if self.vehicle_state == 'roaming': # 배회 모드인 경우 
                    rospy.loginfo("배회 목적지 도착!, 새로운 배회 경로 갱신")
                    self.reset_global_path()
                    self.start_node = self.find_closest_node_in_direction(self.current_position, self.current_heading)
                    # self.start_node = self.find_closest_node_in_direction(msg.pose.pose.position, yaw)
                    self.is_init_pose = True
                    self.generate_roaming_path()
                else:
                    if self.goal_pos is not None:
                        rospy.loginfo("목적지 도착!")
                        goal_pose_msg = PoseStamped()
                        goal_pose_msg.header.stamp = rospy.Time.now()
                        goal_pose_msg.header.frame_id = "map"  # Set appropriate frame ID   
                        goal_pose_msg.pose.position.x = self.goal_pos[0]
                        goal_pose_msg.pose.position.y = self.goal_pos[1]
                        goal_pose_msg.pose.position.z = 0  # Assuming z=0 for 2D navigation
                        self.destination_reached_pub.publish(goal_pose_msg) # 목적지 좌표 발행
                        # Reset state variables and global path
                        self.reset_global_path()

    def dropoff_callback(self, msg):
        if msg.data and self.is_dropoff is not True:
            rospy.loginfo("Passenger dropped off. Resetting global path.")
            self.last_dropoff_position = self.current_position # 마지막 하차 위치 저장 
            self.is_dropoff = True
            self.vehicle_state = 'parking' # 승객 하차하면 주차 중 상태로 변경 
            self.move_to_parking() # 주차장으로 이동    

            
    def roaming_callback(self, msg):
        if msg.data and self.is_prepared_roaming is not True:
            rospy.loginfo("주차장 만석!, 배회 모드 시작")
            self.is_prepared_roaming = True
            self.vehicle_state = 'roaming'
            self.move_to_last_dropoff()

    def start_roaming(self):
        if self.roaming_path is None:
            rospy.loginfo("배회 목적지 도착!, 새로운 배회 경로 갱신")
            self.generate_roaming_path()
        else:
            rospy.loginfo("계속 배회 중")

    def move_to_last_dropoff(self): # 승객이 하차한 위치로 이동 
        rospy.loginfo("Moving to last drop-off position")
        self.end_node = self.find_closest_node(self.last_dropoff_position)
        self.is_goal_pose = True
        self.goal_pos = [self.last_dropoff_position.x, self.last_dropoff_position.y]
        self.update_path()
    
    def move_to_parking(self): # 주차장으로 이동
        # 주차장 좌표 리스트에서 현재 위치와 가장 가까운 노드 찾고 목적지로 설정 
        # self.start_node = self.find_closest_node(self.current_position)
        self.end_node = self.find_closest_node(self.parking_positions[0])
        self.goal_pos = [self.parking_positions[0].x, self.parking_positions[0].y]
        self.is_goal_pose = True
        self.update_path() # 이부분이 필요한가? is_goal_pose되면 odom_callback에서 update_path() 호출됨

    def generate_roaming_path(self): # 배회 경로 생성
        rospy.loginfo("Generating roaming path")
       
        self.end_node = self.find_backward_waypoint(self.current_position) #  self.last_dropoff_position
        self.goal_pos = [self.nodes[self.end_node].point[0], self.nodes[self.end_node].point[1]]
        self.is_goal_pose = True
        self.is_global_path_once_pub = False
        self.update_path()

    def command_callback(self, msg):
        self.is_status = True
        self.vehicle_state = msg.data

    def destination_reached(self, current_position):
 
        threshold_distance = 8.5  # 5 meters 
        current_pos_array = np.array([current_position.x, current_position.y])
        distance_to_goal = np.linalg.norm(np.array(self.goal_pos) - current_pos_array)
        # rospy.loginfo(f"Distance to goal: {distance_to_goal}")
        return distance_to_goal < threshold_distance
        
    def reset_global_path(self):
        # Reset the global path and related state variables
        self.global_path_msg = None
        self.is_goal_pose = False
        self.is_init_pose = False
        self.goal_pos = None # 추가: 목적지 위치 초기화
        self.is_global_path_once_pub = False

    def build_kd_tree(self):
        if not self.kd_tree:  # KD-Tree가 아직 생성되지 않은 경우에만 생성
            node_positions = np.array([[node.point[0], node.point[1]] for node in self.nodes.values()])
            self.kd_tree = KDTree(node_positions)
        
    def find_closest_node(self, position):
        # KD-Tree를 사용하여 노드 검색 최적화
        if not hasattr(self, 'kd_tree'):
            node_positions = np.array([[node.point[0], node.point[1]] for node in self.nodes.values()])
            self.kd_tree = KDTree(node_positions)

        _, idx = self.kd_tree.query([position.x, position.y])
        return list(self.nodes.keys())[idx]
    
    def find_closest_node_in_direction(self, position, heading):
        if not self.kd_tree:
            self.build_kd_tree()
        
        # 현재 위치에서 가장 가까운 10개의 노드 찾기
        distances, indices = self.kd_tree.query([position.x, position.y], k=10)
        
        best_node = None
        best_score = float('inf')
        max_angle_diff = np.pi / 12  # 최대 15도 각도 차이 허용 :핵심 (좌우 7.5도씩)
        
        for i, idx in enumerate(indices):
            node = list(self.nodes.values())[idx]
            node_vector = np.array([node.point[0] - position.x, node.point[1] - position.y])
            angle_diff = abs(atan2(node_vector[1], node_vector[0]) - heading)
            
            if angle_diff <= max_angle_diff:
                score = distances[i] + angle_diff * 20  # 각도 차이에 더 큰 가중치 부여
                if score < best_score:
                    best_score = score
                    best_node = node
        
        return best_node.idx if best_node else None
    

    def find_multiple_start_nodes(self, position, num_candidates=5): # 시작 노드 후보 검색 
        if not self.kd_tree:
            self.build_kd_tree()
        
        distances, indices = self.kd_tree.query([position.x, position.y], k=num_candidates)
        return [list(self.nodes.keys())[idx] for idx in indices]

    # def update_path(self):
    #     if self.is_goal_pose and self.is_init_pose:
    #         if self.global_path_msg is not None:
    #             # 기존 경로가 있는 경우, 일부를 재사용
    #             new_path = self.replan_path_with_existing(self.current_position, self.end_node)
    #         else:
    #             # 기존 경로가 없는 경우, 새로 계산
    #             new_path = self.calc_astar_path_node(self.start_node, self.end_node)

    #         if new_path is not None:
    #             self.global_path_msg = new_path
    #             rospy.loginfo("New global path calculated")
    #             self.is_goal_pose = False
    #             self.is_global_path_once_pub = False
    #         else:
    #             rospy.logerr("Failed to calculate new path")

    def update_path(self):
        if self.is_goal_pose and self.is_init_pose:
            new_path = None
            if self.global_path_msg is not None:
                # 기존 경로가 있는 경우, 일부를 재사용 시도
                new_path = self.replan_path_with_existing(self.current_position, self.end_node)
            
            if new_path is None:
                # 기존 경로 재사용 실패 또는 기존 경로가 없는 경우
                start_candidates = self.find_multiple_start_nodes(self.current_position, num_candidates=5)
                for start_node in start_candidates:
                    new_path = self.calc_astar_path_node(start_node, self.end_node)
                    if new_path is not None:
                        break  # 유효한 경로를 찾으면 루프 종료

            if new_path is not None:
                self.global_path_msg = new_path
                rospy.loginfo("New global path calculated")
                self.is_goal_pose = False
                self.is_global_path_once_pub = False
            else:
                rospy.logerr("Failed to calculate new path with all start node candidates")

    def find_closest_point_on_path(self, current_position):
        if self.global_path_msg is None:
            return None, None
        
        points = np.array([[p.pose.position.x, p.pose.position.y] for p in self.global_path_msg.poses])
        distances = np.linalg.norm(points - np.array([current_position.x, current_position.y]), axis=1)
        closest_index = np.argmin(distances)
        return self.global_path_msg.poses[closest_index], closest_index

    def replan_path_with_existing(self, current_position, end_node):
        closest_pose, closest_index = self.find_closest_point_on_path(current_position)
        if closest_pose is None:
            return self.calc_astar_path_node(self.start_node, end_node)

        # 현재 위치부터 일정 거리까지의 기존 경로 유지
        keep_index = closest_index
        keep_distance = 0
        while keep_index < len(self.global_path_msg.poses) - 1 and keep_distance < self.replan_distance:
            p1 = self.global_path_msg.poses[keep_index].pose.position
            p2 = self.global_path_msg.poses[keep_index + 1].pose.position
            keep_distance += np.linalg.norm([p2.x - p1.x, p2.y - p1.y])
            keep_index += 1

        # 유지할 경로의 마지막 점을 새로운 시작점으로 설정
        new_start_node = self.find_closest_node(self.global_path_msg.poses[keep_index].pose.position)

        # 새로운 경로 계산
        new_path = self.calc_astar_path_node(new_start_node, end_node)
        
        if new_path is not None:
            # 기존 경로의 일부와 새로 계산된 경로를 합침
            combined_path = Path()
            combined_path.header = self.global_path_msg.header
            combined_path.poses = self.global_path_msg.poses[closest_index:keep_index] + new_path.poses
            return combined_path
        
        return None
    def find_backward_waypoint(self, position):
        if not self.kd_tree:
            self.build_kd_tree()
        
        distances, indices = self.kd_tree.query([position.x, position.y], k=50)  # 더 많은 후보 노드 검색
        
        min_distance = 20.0  # 최소 거리 설정 (미터 단위)
        max_distance = 50.0  # 최대 거리 설정
        
        for idx in indices:
            node = list(self.nodes.values())[idx]
            node_vector = np.array([node.point[0] - position.x, node.point[1] - position.y])
            distance = np.linalg.norm(node_vector)
            angle_to_node = atan2(node_vector[1], node_vector[0])
            
            # 차량의 후방 방향에 있고, 지정된 거리 범위 내에 있는 노드 선택
            if abs(angle_to_node - self.current_heading) > np.pi/2 and min_distance < distance < max_distance:
                return node.idx
        
        return None   
      
    def calc_astar_path_node(self, start_node, end_node):
        # A* 알고리즘을 사용하여 경로 계산
        rospy.loginfo(f"Attempting to find path from {start_node} to {end_node}")
        # 1. 기본 A* 알고리즘 시도
        success, path = self.global_planner.find_shortest_path(start_node, end_node)
        if success:
            return self.create_path_msg(path['point_path'])

        # 2. 탐색 범위 확장
        expanded_path = self.global_planner.find_path_with_expanded_search(start_node, end_node)
        if expanded_path:
            return self.create_path_msg(expanded_path['point_path'])

        rospy.logerr(f"Failed to find path. Start: {start_node}, End: {end_node}")
        return None
    
    def create_path_msg(self, point_path):
        smoothed_point_path = self.global_planner.smooth_path(point_path)
        global_path_msg = Path()
        global_path_msg.header.frame_id = 'map'
        for point in smoothed_point_path:
            pose = PoseStamped()
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            pose.pose.position.z = 0
            global_path_msg.poses.append(pose)
        return global_path_msg
    
class AStar:
    def __init__(self, nodes, links):
        self.nodes = nodes
        self.links = links

        # 가중치 행렬 초기화 개선: 실제 연결된 노드만 초기화
        self.weight = {from_node_id: {} for from_node_id in nodes.keys()}
        for from_node_id, from_node in nodes.items():
            for to_node in from_node.get_to_nodes():
                shortest_link, min_cost = from_node.find_shortest_link_leading_to_node(to_node)
                if shortest_link is not None:
                    self.weight[from_node_id][to_node.idx] = min_cost
        
        rospy.Subscriber('/odom', Odometry, self.odom_callback)

    # def heuristic(self, a, b):
    #     # 휴리스틱 함수: 두 노드 간의 유클리드 거리
    #     return sqrt(pow(a.point[0] - b.point[0], 2) + pow(a.point[1] - b.point[1], 2))
    
    def heuristic(self, a, b, current_heading):
        distance = sqrt(pow(a.point[0] - b.point[0], 2) + pow(a.point[1] - b.point[1], 2))
        direction_vector = np.array([b.point[0] - a.point[0], b.point[1] - a.point[1]])
        angle_diff = abs(atan2(direction_vector[1], direction_vector[0]) - current_heading)
        return distance + angle_diff * 10

    def smooth_path(self, path, weight_data=0.2, weight_smooth=0.3, tolerance=0.001): # 0.5 0.1 0.000001
        # 경로 평활화
        newpath = copy.deepcopy(path)
        change = tolerance
        while change >= tolerance:
            change = 0.0
            for i in range(1, len(path) - 1):
                for j in range(len(path[0])):
                    aux = newpath[i][j]
                    newpath[i][j] += weight_data * (path[i][j] - newpath[i][j])
                    newpath[i][j] += weight_smooth * (newpath[i-1][j] + newpath[i+1][j] - (2.0 * newpath[i][j]))
                    change += abs(aux - newpath[i][j])
        return newpath

    def odom_callback(self, msg):
        
        self.current_position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        _, _, yaw = tf.transformations.euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        self.current_heading = yaw

    def find_shortest_path(self, start_node_idx, end_node_idx):
        start_node = self.nodes[start_node_idx]
        end_node = self.nodes[end_node_idx]
        
        open_list = []
        heapq.heappush(open_list, (0, start_node_idx))
        
        came_from = {}
        
        g_score = {node_id: float('inf') for node_id in self.nodes.keys()}
        g_score[start_node_idx] = 0
        
        f_score = {node_id: float('inf') for node_id in self.nodes.keys()}
        f_score[start_node_idx] = self.heuristic(start_node, end_node, self.current_heading)

        while open_list:
            current_f_score, current_node_idx = heapq.heappop(open_list)

            if current_node_idx == end_node_idx:
                return True, self.reconstruct_path(came_from, end_node_idx)

            for neighbor_idx in self.weight[current_node_idx]:
                tentative_g_score = g_score[current_node_idx] + self.weight[current_node_idx][neighbor_idx]

                if tentative_g_score < g_score[neighbor_idx]:
                    came_from[neighbor_idx] = current_node_idx
                    g_score[neighbor_idx] = tentative_g_score
                    f_score[neighbor_idx] = g_score[neighbor_idx] + self.heuristic(self.nodes[neighbor_idx], end_node, self.current_heading)
                    heapq.heappush(open_list, (f_score[neighbor_idx], neighbor_idx))

        return False, None

    def reconstruct_path(self, came_from, current_node_idx):
        # 경로 재구성
        node_path = [current_node_idx]
        while current_node_idx in came_from:
            current_node_idx = came_from[current_node_idx]
            node_path.append(current_node_idx)
        node_path.reverse()

        link_path, point_path = self.construct_link_and_point_paths(node_path)
        return {'node_path': node_path, 'link_path': link_path, 'point_path': point_path}

    def construct_link_and_point_paths(self, node_path):
        # 노드 경로를 링크와 포인트 경로로 변환
        link_path = []
        point_path = []
        for i in range(len(node_path) - 1):
            from_node_idx = node_path[i]
            to_node_idx = node_path[i + 1]
            from_node = self.nodes[from_node_idx]
            to_node = self.nodes[to_node_idx]
            shortest_link, _ = from_node.find_shortest_link_leading_to_node(to_node)
            link_path.append(shortest_link.idx)
            for point in shortest_link.points:
                point_path.append([point[0], point[1], 0])
        return link_path, point_path
    
    def find_path_with_expanded_search(self, start_node_idx, end_node_idx):
        max_iterations = 5  # 최대 반복 횟수
        expansion_factor = 1.5  # 탐색 범위 확장 계수

        for iteration in range(max_iterations):
            expanded_radius = self.heuristic(self.nodes[start_node_idx], self.nodes[end_node_idx], self.current_heading) * expansion_factor * (iteration + 1)

            open_list = []
            heapq.heappush(open_list, (0, start_node_idx))
            came_from = {}
            g_score = {node_id: float('inf') for node_id in self.nodes.keys()}
            g_score[start_node_idx] = 0
            f_score = {node_id: float('inf') for node_id in self.nodes.keys()}
            f_score[start_node_idx] = self.heuristic(self.nodes[start_node_idx], self.nodes[end_node_idx], self.current_heading)

            while open_list:
                current_f_score, current_node_idx = heapq.heappop(open_list)

                if current_node_idx == end_node_idx:
                    return self.reconstruct_path(came_from, end_node_idx)

                for neighbor_idx in self.weight[current_node_idx]:
                    # 시작 노드와 같은 노드는 무시
                    if neighbor_idx == start_node_idx:
                        continue

                    if self.heuristic(self.nodes[neighbor_idx], self.nodes[start_node_idx], self.current_heading) > expanded_radius:
                        continue  # 확장된 반경을 벗어난 노드는 무시

                    tentative_g_score = g_score[current_node_idx] + self.weight[current_node_idx][neighbor_idx]
                    if tentative_g_score < g_score[neighbor_idx]:
                        came_from[neighbor_idx] = current_node_idx
                        g_score[neighbor_idx] = tentative_g_score
                        f_score[neighbor_idx] = g_score[neighbor_idx] + self.heuristic(self.nodes[neighbor_idx], self.nodes[end_node_idx], self.current_heading)
                        heapq.heappush(open_list, (f_score[neighbor_idx], neighbor_idx))

        return None

if __name__ == '__main__':
    try:
        AStarPathPublisher()
    except rospy.ROSInterruptException:
        pass