#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import rospkg
import sys
import os
import copy
import numpy as np
import json
from math import cos, sin, sqrt, pow, atan2, pi
from geometry_msgs.msg import Point32, PoseStamped
from nav_msgs.msg import Odometry, Path

current_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(current_path)
from lib.mgeo.class_defs import *

class DijkstraPathPublisher:
    def __init__(self):
        rospy.init_node('dijkstra_path_pub', anonymous=True)
        self.global_path_pub = rospy.Publisher('/global_path', Path, queue_size=1)

        # Load Mgeo data and verify
        try:
            load_path = os.path.normpath(os.path.join(current_path, 'lib/mgeo_data/R_KR_PG_K-City'))
            mgeo_planner_map = MGeo.create_instance_from_json(load_path)
            self.nodes = mgeo_planner_map.node_set.nodes
            self.links = mgeo_planner_map.link_set.lines
        except Exception as e:
            rospy.logerr(f"Failed to load Mgeo data: {e}")
            sys.exit(1)

        self.global_planner = Dijkstra(self.nodes, self.links)

        # Define start and end nodes
        self.start_node = 'A119BS010266'
        self.end_node = 'A119BS010148'

        self.global_path_msg = self.calc_dijkstra_path_node(self.start_node, self.end_node)

        rate = rospy.Rate(10)  # 10hz
        while not rospy.is_shutdown():
            # Publish the Global Path message
            self.global_path_pub.publish(self.global_path_msg)
            rate.sleep()

    def calc_dijkstra_path_node(self, start_node, end_node):
        success, path = self.global_planner.find_shortest_path(start_node, end_node)
        if not success:
            rospy.logerr("Failed to find the shortest path.")
            return Path()

        # Define Dijkstra path data in ROS Path message format
        out_path = Path()
        out_path.header.frame_id = '/map'
        for point in path['point_path']:
            pose = PoseStamped()
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            pose.pose.position.z = point[2]
            out_path.poses.append(pose)

        return out_path

class Dijkstra:
    def __init__(self, nodes, links):
        self.nodes = nodes
        self.links = links
        self.weight = self.get_weight_matrix()

    def get_weight_matrix(self):
        weight = {from_node_id: {to_node_id: float('inf') for to_node_id in self.nodes.keys()} for from_node_id in self.nodes.keys()}

        for from_node_id, from_node in self.nodes.items():
            weight[from_node_id][from_node_id] = 0
            for to_node in from_node.get_to_nodes():
                shortest_link, min_cost = from_node.find_shortest_link_leading_to_node(to_node)
                weight[from_node_id][to_node.idx] = min_cost

        return weight

    def find_shortest_path(self, start_node_idx, end_node_idx):
        visited = {node_id: False for node_id in self.nodes.keys()}
        from_node = {node_id: start_node_idx for node_id in self.nodes.keys()}

        visited[start_node_idx] = True
        distance = copy.deepcopy(self.weight[start_node_idx])

        for _ in range(len(self.nodes.keys()) - 1):
            selected_node_idx = self.find_nearest_node_idx(distance, visited)
            if selected_node_idx is None:
                break
            visited[selected_node_idx] = True
            for to_node_idx in self.nodes.keys():
                if not visited[to_node_idx]:
                    if selected_node_idx in self.weight and to_node_idx in self.weight[selected_node_idx]:
                        distance_candidate = distance[selected_node_idx] + self.weight[selected_node_idx][to_node_idx]
                        if distance_candidate < distance[to_node_idx]:
                            distance[to_node_idx] = distance_candidate
                            from_node[to_node_idx] = selected_node_idx

        node_path = self.construct_path(from_node, start_node_idx, end_node_idx)
        link_path, point_path = self.construct_link_and_point_paths(node_path)

        return True, {'node_path': node_path, 'link_path': link_path, 'point_path': point_path}

    def find_nearest_node_idx(self, distance, visited):
        min_value = float('inf')
        min_idx = None
        for idx in self.nodes.keys():
            if distance[idx] < min_value and not visited[idx]:
                min_value = distance[idx]
                min_idx = idx
        return min_idx

    def construct_path(self, from_node, start_node_idx, end_node_idx):
        tracking_idx = end_node_idx
        node_path = [end_node_idx]
        while start_node_idx != tracking_idx:
            tracking_idx = from_node[tracking_idx]
            node_path.append(tracking_idx)
        node_path.reverse()
        return node_path

    def construct_link_and_point_paths(self, node_path):
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
        dijkstra_path_pub = DijkstraPathPublisher()
    except rospy.ROSInterruptException:
        pass