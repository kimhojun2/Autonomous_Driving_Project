#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
import math

class local_path_pub:
    def __init__(self):
        rospy.init_node('local_path_pub', anonymous=True)

        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self.global_path_sub = rospy.Subscriber("/global_path", Path, self.global_path_callback)

        self.local_path_pub = rospy.Publisher('/local_path', Path, queue_size=1)

        self.global_path_msg = Path()
        self.is_global_path_received = False

        self.current_waypoint_index = 0  # 현재 방문 중인 웨이포인트 인덱스
        self.visit_threshold = 3.0  # 웨이포인트를 방문했다고 간주하는 거리 임계값

    def odom_callback(self, msg):
        if not self.is_global_path_received:
            return

        # 현재 위치에서 가장 가까운 웨이포인트 찾기
        closest_waypoint_index = self.find_closest_waypoint_index(msg.pose.pose)
        
        # 가장 가까운 웨이포인트가 현재 인덱스보다 앞에 있지 않은지 확인
        if closest_waypoint_index >= self.current_waypoint_index:
            self.current_waypoint_index = closest_waypoint_index
        
        self.publish_local_path()

    def global_path_callback(self, msg):
        self.global_path_msg = msg
        self.is_global_path_received = True
        self.current_waypoint_index = 0  # 글로벌 패스를 받으면 웨이포인트 인덱스 초기화

    def publish_local_path(self):
        if not self.is_global_path_received:
            return

        local_path_msg = Path()
        local_path_msg.header.frame_id = 'map'

        # 현재 웨이포인트 인덱스부터 로컬 패스 생성
        start_index = self.current_waypoint_index
        end_index = min(start_index + 100, len(self.global_path_msg.poses))  # 최대 100개의 포인트를 포함

        for i in range(start_index, end_index):
            local_path_msg.poses.append(self.global_path_msg.poses[i])

        self.local_path_pub.publish(local_path_msg)

    def find_closest_waypoint_index(self, current_pose):
        min_distance = float('inf')
        closest_index = self.current_waypoint_index
        search_start_index = max(0, self.current_waypoint_index - 20)  # 20m 이전의 웨이포인트부터 검색 시작
        search_end_index = min(self.current_waypoint_index + 20, len(self.global_path_msg.poses))  # 20m 이후의 웨이포인트까지 검색

        for i in range(search_start_index, search_end_index):
            distance = self.calculate_distance(current_pose, self.global_path_msg.poses[i].pose)
            if distance < min_distance:
                min_distance = distance
                closest_index = i

        # 최근 방문한 웨이포인트에서 20m 이내에 있는 가장 가까운 웨이포인트를 찾되,
        # 현재 웨이포인트 인덱스보다 앞선 웨이포인트는 제외
        if closest_index > self.current_waypoint_index and min_distance <= 20.0:
            return closest_index
        else:
            return self.current_waypoint_index  # 가장 가까운 웨이포인트가 조건을 만족하지 않으면 현재 인덱스 유지


    def calculate_distance(self, pose1, pose2):
        dx = pose1.position.x - pose2.position.x
        dy = pose1.position.y - pose2.position.y
        return math.sqrt(dx ** 2 + dy ** 2)

if __name__ == '__main__':
    try:
        local_path_publisher = local_path_pub()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
