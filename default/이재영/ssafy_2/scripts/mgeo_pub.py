#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import sys
import rospy

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point32
from sensor_msgs.msg import PointCloud

current_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(current_path)

from lib.mgeo.class_defs import *

class get_mgeo:
    def __init__(self):
        rospy.init_node('test', anonymous=True)
        self.link_pub = rospy.Publisher('link', PointCloud, queue_size=1)
        self.node_pub = rospy.Publisher('node', PointCloud, queue_size=1)

        # (1) Mgeo data 읽어온 후 데이터 확인
        load_path = os.path.normpath(os.path.join(current_path, 'lib/mgeo_data/R_KR_PG_K-City'))
        
        try:
            mgeo_planner_map = MGeo.create_instance_from_json(load_path)
            rospy.loginfo("MGeo data successfully loaded.")
        except Exception as e:
            rospy.logerr(f"Failed to load MGeo data: {e}")
            sys.exit(1)

        node_set = mgeo_planner_map.node_set
        link_set = mgeo_planner_map.link_set

        self.nodes = node_set.nodes
        self.links = link_set.lines

        self.link_msg = self.getAllLinks()
        self.node_msg = self.getAllNode()

        rospy.loginfo(f'# of nodes: {len(node_set.nodes)}')
        rospy.loginfo(f'# of links: {len(link_set.lines)}')

        rate = rospy.Rate(1) 
        while not rospy.is_shutdown():
            # (4) 변환한 Link, Node 정보 Publish
            self.link_pub.publish(self.link_msg)
            self.node_pub.publish(self.node_msg)
            rospy.loginfo("Published link and node data.")
            rate.sleep()

    def getAllLinks(self):
        all_link = PointCloud()
        all_link.header.frame_id = 'map'

        # (2) Link 정보 Point Cloud 데이터로 변환
        for link_idx, link in self.links.items():
            for point in link.points:
                pt = Point32()
                pt.x = point[0]
                pt.y = point[1]
                pt.z = point[2] if len(point) > 2 else 0.0  # Assuming 2D if z is not provided
                all_link.points.append(pt)
        
        rospy.loginfo(f"Converted {len(self.links)} links to PointCloud.")
        return all_link
    
    def getAllNode(self):
        all_node = PointCloud()
        all_node.header.frame_id = 'map'

        # (3) Node 정보 Point Cloud 데이터로 변환
        for node_idx, node in self.nodes.items():
            pt = Point32()
            pt.x = node.point[0]
            pt.y = node.point[1]
            pt.z = node.point[2] if len(node.point) > 2 else 0.0  # Assuming 2D if z is not provided
            all_node.points.append(pt)

        rospy.loginfo(f"Converted {len(self.nodes)} nodes to PointCloud.")
        return all_node

if __name__ == '__main__':
    test_track = get_mgeo()