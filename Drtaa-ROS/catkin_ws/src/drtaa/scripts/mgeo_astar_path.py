#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import os
import sys
import copy
import numpy as np
from math import sqrt, pow, atan2
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path
from queue import PriorityQueue

current_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(current_path)
from lib.mgeo.class_defs import *

class AStarPathPublisher:
    def __init__(self):
        # ROS 노드 초기화 및 구독자, 발행자 설정
        rospy.init_node('astar_path_pub', anonymous=True)
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)
        rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, self.init_callback)
        self.global_path_pub = rospy.Publisher('/global_path', Path, queue_size=1)

        # 초기 상태 변수 설정
        self.is_goal_pose = False
        self.is_init_pose = False
        self.start_node = None
        self.end_node = None
        self.global_path_msg = None
        self.path_calculated = False

        try:
            # MGeo 데이터 로드
            load_path = os.path.normpath(os.path.join(current_path, 'lib/mgeo_data/R_KR_PR_Sangam_NoBuildings'))
            mgeo_planner_map = MGeo.create_instance_from_json(load_path)
            self.nodes = mgeo_planner_map.node_set.nodes
            self.links = mgeo_planner_map.link_set.lines
        except Exception as e:
            rospy.logerr(f"Failed to load Mgeo data: {e}")

        # A* 경로 계획기 초기화
        self.global_planner = AStar(self.nodes, self.links)

        # 메인 루프
        rate = rospy.Rate(50) # 50Hz
        while not rospy.is_shutdown():
            if self.global_path_msg is not None:
                self.global_path_pub.publish(self.global_path_msg)
            else:
                rospy.loginfo('Waiting for goal and current position data')
            rate.sleep()

    def goal_callback(self, msg):
        # 목표 위치 설정
        if self.nodes is None:
            rospy.logerr("Nodes data is not available. Cannot process goal.")
            return
        self.end_node = self.find_closest_node(msg.pose.position)
        self.is_goal_pose = True
        self.update_path()

    def init_callback(self, msg):
        # 시작 위치 설정
        if self.nodes is None:
            rospy.logerr("Nodes data is not available. Cannot process init pose.")
            return
        self.start_node = self.find_closest_node(msg.pose.pose.position)
        self.is_init_pose = True
        self.update_path()

    def find_closest_node(self, position):
        # 주어진 위치에서 가장 가까운 노드 찾기
        closest_node = None
        min_distance = float('inf')
        for node_id, node in self.nodes.items():
            distance = sqrt(pow(node.point[0] - position.x, 2) + pow(node.point[1] - position.y, 2))
            if distance < min_distance:
                min_distance = distance
                closest_node = node_id
        return closest_node

    def update_path(self):
        # 시작점과 목표점이 설정되면 경로 계산
        if self.is_goal_pose and self.is_init_pose and not self.path_calculated:
            self.global_path_msg = self.calc_astar_path_node(self.start_node, self.end_node)
            if self.global_path_msg is not None:
                self.path_calculated = True
                rospy.loginfo("Global path calculated and fixed")

    def calc_astar_path_node(self, start_node, end_node):
        # A* 알고리즘을 사용하여 경로 계산
        rospy.loginfo(f"Attempting to find path from {start_node} to {end_node}")
        success, path = self.global_planner.find_shortest_path(start_node, end_node)
        if not success:
            rospy.logerr(f"Failed to find path. Start: {start_node}, End: {end_node}")
            rospy.logerr(f"Number of nodes: {len(self.nodes)}")
            rospy.logerr(f"Number of links: {len(self.links)}")
            return None

        # 경로 평활화 적용
        smoothed_point_path = self.global_planner.smooth_path(path['point_path'])

        # ROS Path 메시지 생성
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
        self.weight = self.get_weight_matrix()

    def get_weight_matrix(self):
        # 노드 간 가중치 행렬 생성
        weight = {from_node_id: {to_node_id: float('inf') for to_node_id in self.nodes.keys()} for from_node_id in self.nodes.keys()}
        for from_node_id, from_node in self.nodes.items():
            weight[from_node_id][from_node_id] = 0
            for to_node in from_node.get_to_nodes():
                shortest_link, min_cost = from_node.find_shortest_link_leading_to_node(to_node)
                weight[from_node_id][to_node.idx] = min_cost
        return weight
    
    def heuristic(self, a, b):
        # 휴리스틱 함수: 두 노드 간의 유클리드 거리
        return sqrt(pow(a.point[0] - b.point[0], 2) + pow(a.point[1] - b.point[1], 2))

    def smooth_path(self, path, weight_data=0.5, weight_smooth=0.1, tolerance=0.000001):
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
    
    def find_shortest_path(self, start_node_idx, end_node_idx):
        # A* 알고리즘을 사용한 최단 경로 탐색
        start_node = self.nodes[start_node_idx]
        end_node = self.nodes[end_node_idx]

        open_list = PriorityQueue()
        open_list.put((0, start_node_idx))

        came_from = {}
        g_score = {node_id: float('inf') for node_id in self.nodes.keys()}
        g_score[start_node_idx] = 0
        f_score = {node_id: float('inf') for node_id in self.nodes.keys()}
        f_score[start_node_idx] = self.heuristic(start_node, end_node)

        while not open_list.empty():
            current_node_idx = open_list.get()[1]

            if current_node_idx == end_node_idx:
                return True, self.reconstruct_path(came_from, end_node_idx)

            for neighbor_idx in self.weight[current_node_idx]:
                tentative_g_score = g_score[current_node_idx] + self.weight[current_node_idx][neighbor_idx]

                if tentative_g_score < g_score[neighbor_idx]:
                    came_from[neighbor_idx] = current_node_idx
                    g_score[neighbor_idx] = tentative_g_score
                    f_score[neighbor_idx] = g_score[neighbor_idx] + self.heuristic(self.nodes[neighbor_idx], end_node)
                    open_list.put((f_score[neighbor_idx], neighbor_idx))

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

if __name__ == '__main__':
    try:
        AStarPathPublisher()
    except rospy.ROSInterruptException:
        pass