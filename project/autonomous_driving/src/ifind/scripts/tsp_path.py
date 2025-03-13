#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import rospkg
import sys
import os
import copy
import numpy as np
import json
from morai_msgs.msg import GPSMessage
from pyproj import Proj, Transformer

import firebase_admin
from firebase_admin import credentials
from firebase_admin import firestore, messaging

from math import cos,sin,sqrt,pow,atan2,pi
from geometry_msgs.msg import Point32,PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry,Path
from std_msgs.msg import String
from python_tsp.heuristics import solve_tsp_local_search
import pickle



current_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(current_path)

from lib.mgeo.class_defs import *

class tsp_path_pub :
    def __init__(self):
        rospy.init_node('tsp_path_pub', anonymous=True)

        self.global_path_pub = rospy.Publisher('/global_path',Path, queue_size = 1)
        self.visit_order_pub = rospy.Publisher('/visit_order_topic', String, queue_size=10)

        self.proj_UTM = Proj(proj='utm', zone=52, ellps='WGS84', preserve_units=False)
        self.proj_WGS84 = Proj(proj='latlong', datum='WGS84')


        #TODO: (1) Mgeo data 읽어온 후 데이터 확인
        # load_path = os.path.normpath(os.path.join(current_path, 'lib/mgeo_data/R_KR_PG_K-City'))
        load_path = os.path.normpath(os.path.join(current_path, 'lib/mgeo_data/R_KR_PR_Sangam_NoBuildings'))
        mgeo_planner_map = MGeo.create_instance_from_json(load_path)

        node_set = mgeo_planner_map.node_set
        link_set = mgeo_planner_map.link_set

        self.nodes=node_set.nodes
        self.links=link_set.lines

    
        self.global_planner=Astar(self.nodes,self.links)

        self.is_goal_pose = False
        self.is_init_pose = False

        self.is_store_start = False

        self.visit_order = []

        self.total_distance = 0

        # 이미 초기화된 앱이 있는지 확인
        if not firebase_admin._apps:
            # Firebase Admin SDK를 초기화
            cred = credentials.Certificate('/home/morai/catkin_ws/src/ifind/key/ifind-firebase-adminsdk.json')
            firebase_admin.initialize_app(cred, {
                'databaseURL': 'https://ifind.firebaseio.com'
            })

        # Firestore 데이터베이스 가져오기
        self.db = firestore.client()

        self.gps_sub = rospy.Subscriber("/gps", GPSMessage, self.init_callback)
        self.goal_callback()

        print(self.is_goal_pose, self.is_init_pose)

        while True:
            if self.is_goal_pose == True and self.is_init_pose == True:
                break
            else:
                # rospy.loginfo('Waiting goal pose data')
                # rospy.loginfo('Waiting init pose data')
                pass


        self.global_path_msg = Path()
        self.global_path_msg.header.frame_id = '/map'

        self.global_path_msg = self.calc_path_through_nodes()
            
        # 만든 Global Path 메세지 를 전송         
        self.global_path_pub.publish(self.global_path_msg)

        # firebase에 global path 쓰기
        self.store_global_path()

        # firebase에 방문 순서 쓰기
        self.store_visit_order()
            
        # rate.sleep()

    def store_global_path(self):

        # 글로벌 패스 메시지 줄이기
        reduced_path_data = self.global_path_msg.poses[::50]  # 100개 포인트마다 하나씩 추출

        # Firestore에 저장할 경로 데이터
        path_data_for_firestore = []

        # 추출된 경로 데이터 변환 및 Firestore용 데이터 준비
        for pose_stamped in reduced_path_data:
            x = pose_stamped.pose.position.x
            y = pose_stamped.pose.position.y

            # UTM 좌표를 위도와 경도로 변환
            lat, lon = self.convert_xy_to_latlon(x, y)
            path_data_for_firestore.append({'latitude': lat, 'longitude': lon})

        # Firestore에 글로벌 패스 데이터 쓰기
        self.db.collection('morai').document('path').set({'path' : path_data_for_firestore, 'total_distance' :self.total_distance})



    def store_visit_order(self):
        # Firestore에 저장할 경로 데이터
        order_data_for_firestore = []

        # 추출된 경로 데이터 변환 및 Firestore용 데이터 준비
        for node in self.visit_order:
            x = node['x']
            y = node['y']

            # UTM 좌표를 위도와 경도로 변환
            lat, lon = self.convert_xy_to_latlon(x, y)
            order_data_for_firestore.append({'latitude': lat, 'longitude': lon})

        # Firestore에 글로벌 패스 데이터 쓰기
        order_document_data = {'stop': order_data_for_firestore[:-1]}
        self.db.collection('morai').document('order').set(order_document_data)

        # 방문 노드 publish
        visit_order_json = json.dumps(self.visit_order)
        self.visit_order_pub.publish(visit_order_json)


    def solve_tsp(self, distance_matrix):

        distance_matrix = np.array(distance_matrix)

        permutation, _ = solve_tsp_local_search(distance_matrix)

        return permutation
    

    def generate_final_path(self, optimal_order):
        total_path = []
        self.total_distance = 0
        self.visit_order = []

        start_node_index = None
        for i, node_info in enumerate(self.selected_nodes):
            if node_info['idx'] == self.start_node:
                start_node_index = i
                break

        if start_node_index is None:
            print("Start node is not in selected_nodes.")
            return []
        
        rospy.loginfo(self.selected_nodes)
        rospy.loginfo(optimal_order)

        # optimal_order을 회전하여 시작 노드 첫 번째로 조정
        start_idx_in_order = optimal_order.index(0)
        optimal_order_rotated = np.roll(optimal_order, -start_idx_in_order)

        rospy.loginfo(optimal_order)
        rospy.loginfo(optimal_order_rotated)

        for i in range(len(optimal_order_rotated)-1):
            from_idx = optimal_order_rotated[i]
            to_idx = optimal_order_rotated[i+1]

            from_node = self.selected_nodes[from_idx]
            to_node = self.selected_nodes[to_idx]

            rospy.loginfo(from_node)
            rospy.loginfo(to_node)
            rospy.loginfo("............")


            result, path = self.global_planner.find_shortest_path(from_node['idx'], to_node['idx'])
            if result:
                total_path.extend(path['point_path'])
                self.total_distance += path['total_distance']
                self.visit_order.append(to_node)

        # 마지막 노드에서 시작 노드로 돌아가는 경로 추가
        last_to_first = self.global_planner.find_shortest_path(self.selected_nodes[optimal_order_rotated[-1]]['idx'], self.start_node)
        if last_to_first[0]:
            total_path.extend(last_to_first[1]['point_path'])
            self.total_distance += last_to_first[1]['total_distance']
            self.visit_order.append(self.start)

        return total_path
        
    def create_path_message(self, total_path):
        global_path_msg = Path()
        global_path_msg.header.frame_id = '/map'
        for point in total_path:
            pose = PoseStamped()
            pose.header.frame_id = '/map'
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            pose.pose.position.z = 0 
            global_path_msg.poses.append(pose)

        return global_path_msg

    def calculate_distance_matrix(self):
        num_nodes = len(self.selected_nodes)
        distance_matrix = np.full((num_nodes, num_nodes), np.inf)

        for i, from_node in enumerate(self.selected_nodes):
            for j, to_node in enumerate(self.selected_nodes):
                if i == j:
                    distance_matrix[i, j] = 0.0
                else:
                    # Astar 알고리즘을 사용하여 각 노드 간의 거리 계산
                    result, path = self.global_planner.find_shortest_path(from_node['idx'], to_node['idx'])
                    if result:
                        distance_matrix[i, j] = path['total_distance']

        return distance_matrix

    
    def find_shortest_link_leading_to_node(self, from_node,to_node):
        # 현재 노드에서 to_node로 연결되어 있는 링크를 찾고, 그 중에서 가장 빠른 링크를 찾아준다

        to_links = []
        for link in from_node.get_to_links():
            if link.to_node is to_node:
                to_links.append(link)
        
        shortest_link = None
        min_cost = float('inf')

        for link in to_links:
            if link.cost < min_cost:
                min_cost = link.cost
                shortest_link = link       

        return shortest_link, min_cost

    def calc_path_through_nodes(self):
        # 거리 매트릭스 계산
        distance_matrix = self.calculate_distance_matrix()

        # TSP 문제 해결
        optimal_order = self.solve_tsp(distance_matrix)

        # 최종 경로 생성
        total_path = self.generate_final_path(optimal_order)

        # Global Path 메시지 생성 및 발행
        global_path_msg = self.create_path_message(total_path)

        return global_path_msg
    
    def convert_latlon_to_xy(self, lon, lat):
        # 위도와 경도 UTM 좌표계로 변환
        xy_zone = self.proj_UTM(lon, lat)

        x = 0.0
        y = 0.0
        if lon == 0 and lat == 0:
            x = 0.0
            y = 0.0
        else:
            x = xy_zone[0] - self.e_o
            y = xy_zone[1] - self.n_o    

        return x, y
    
    def convert_xy_to_latlon(self, x, y):
        # UTM 좌표에 offset 적용하기 전의 위치 복원

        if x == 0 and y == 0:
            original_x  = 0.0
            original_y  = 0.0
        else:
            original_x  = x + self.e_o
            original_y  = y + self.n_o

        # 변환 객체 생성
        transformer = Transformer.from_proj(self.proj_UTM, self.proj_WGS84, always_xy=True)

        # 좌표 변환
        lon, lat = transformer.transform(original_x, original_y)
        return lat, lon

    def init_callback(self,msg):
        self.lat = msg.latitude
        self.lon = msg.longitude
        self.e_o = msg.eastOffset
        self.n_o = msg.northOffset

        self.is_gps=True
        
        x, y = self.convert_latlon_to_xy(self.lon, self.lat)

        nearest_node_id = self.find_nearest_node_idx(x, y)

        self.start_node = nearest_node_id
        self.start = {'x' : x, 'y' : y}
        self.is_init_pose = True

        # 출발 지점 파이어 베이스에 쓰기
        if self.is_store_start == False:
            self.is_store_start = True
            order_document_data = {'latitude': msg.latitude, 'longitude': msg.longitude}
            self.db.collection('morai').document('start').set(order_document_data)        

    def goal_callback(self):

        doc = self.db.collection('morai').document('node').get()

        self.selected_nodes = []

        if doc.exists:
            # 문서의 데이터를 딕셔너리 형태로 변환
            data = doc.to_dict()

            # 변환된 데이터를 리스트로 재구성
            data_list = []
            for key, value in data.items():
                node_data = {'number': key}
                node_data.update(value)
                data_list.append(node_data)

            # 승하차 있는 node만 가져오기
            for node in data_list:
                if node['select']:
                    lat = node['latitude']
                    lon = node['longitude']

                    # 위도와 경도를 UTM 좌표계로 변환
                    x, y = self.convert_latlon_to_xy(lon, lat)

                    idx = self.find_nearest_node_idx(x, y)
                    self.selected_nodes.append({'number' : node['number'], 'x' : x, 'y' : y, 'idx' : idx, 'select' : node['select'], 'stop' : node['stop']})            
                    
                    self.is_goal_pose = True
       
            # 시작 지점을 selected_nodes에 추가
            self.selected_nodes.insert(0, {'x': self.start['x'], 'y': self.start['y'], 'idx': self.start_node})

        else:
            print("저장된 하차 지점이 없습니다.")


    def find_nearest_node_idx(self, x, y):
        nearest_node_id = None
        min_distance = float('inf')
        for node_id, node in self.nodes.items():
            node_x = node.point[0]
            node_y = node.point[1]
            distance = sqrt((node_x - x) ** 2 + (node_y - y) ** 2)
            if distance < min_distance:
                nearest_node_id = node_id
                min_distance = distance

        return nearest_node_id

    def calc_tsp_path_node(self, start_node, end_node):

        result, path = self.global_planner.find_shortest_path(start_node, end_node)

        # tsp 경로 데이터를 ROS Path 메세지 형식에 맞춰 정의
        out_path = Path()
        out_path.header.frame_id = '/map'
        
        for point in path['point_path']:
            pose = PoseStamped()
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            pose.pose.position.z = 0 
            out_path.poses.append(pose)
        

        return out_path

class Astar:
    def __init__(self, nodes, links):
        self.nodes = nodes
        self.links = links

        current_path = os.path.dirname(os.path.realpath(__file__))
        pkl_dir = os.path.join(current_path, os.pardir, 'pkl')
        self.weight_matrix_file = os.path.join(pkl_dir, "weight_matrix.pkl")
        self.weight = self.get_weight_matrix()

    def heuristic(self, current_node, goal_node):
        # Euclidean distance 구하기
        current_point = np.array(self.nodes[current_node].point)
        goal_point = np.array(self.nodes[goal_node].point)
        return np.linalg.norm(current_point - goal_point)

    def save_weight_matrix(self, weight_matrix):
        try:
            with open(self.weight_matrix_file, "wb") as f:
                pickle.dump(weight_matrix, f)
        except IOError as e:
            print(f"Failed to save weight matrix: {e}")

    def load_weight_matrix(self):
        if os.path.exists(self.weight_matrix_file):
            try:
                with open(self.weight_matrix_file, "rb") as f:
                    return pickle.load(f)
            except IOError as e:
                print(f"Failed to load weight matrix: {e}")
        return None

    def get_weight_matrix(self):
        weight_matrix = self.load_weight_matrix()
        if weight_matrix is not None:
            return weight_matrix

        weight = {node_id: {to_node_id: float('inf') for to_node_id in self.nodes} for node_id in self.nodes}
        for from_node_id in self.nodes:
            weight[from_node_id][from_node_id] = 0
            for to_node in self.nodes[from_node_id].get_to_nodes():
                shortest_link, min_cost = self.find_shortest_link_leading_to_node(from_node_id, to_node.idx)
                if shortest_link:
                    weight[from_node_id][to_node.idx] = min_cost

        self.save_weight_matrix(weight)
        return weight

    def find_shortest_link_leading_to_node(self, from_node_id, to_node_id):
        from_node = self.nodes[from_node_id]
        to_links = from_node.get_to_links()
        shortest_link = None
        min_cost = float('inf')
        for link in to_links:
            if link.to_node.idx == to_node_id and link.cost < min_cost:
                shortest_link = link
                min_cost = link.cost
        if shortest_link is None:
            return None, float('inf') 
        return shortest_link, min_cost
    
    def find_shortest_path(self, start_node_idx, end_node_idx):
        open_set = set([start_node_idx])
        came_from = {}

        g_score = {node: float('inf') for node in self.nodes}
        g_score[start_node_idx] = 0

        f_score = {node: float('inf') for node in self.nodes}
        f_score[start_node_idx] = self.heuristic(start_node_idx, end_node_idx)

        while open_set:
            current = min(open_set, key=lambda o: f_score[o])

            # 목표 노드 도착
            if current == end_node_idx:
                return True, self.compile_path_data(came_from, current)

            open_set.remove(current)

            for neighbor in self.nodes[current].get_to_nodes():
                tentative_g_score = g_score[current] + self.heuristic(current, neighbor.idx)

                if tentative_g_score < g_score[neighbor.idx]:
                    came_from[neighbor.idx] = current
                    g_score[neighbor.idx] = tentative_g_score
                    f_score[neighbor.idx] = g_score[neighbor.idx] + self.heuristic(neighbor.idx, end_node_idx)
                    open_set.add(neighbor.idx)

        return False, {}
    
    def compile_path_data(self, came_from, current):
        # path 경로 만들기
        total_path = [current]
        while current in came_from:
            current = came_from[current]
            total_path.append(current)
        total_path.reverse() 

        node_path = total_path
        link_path = []
        point_path = []
        total_distance = 0

        for i in range(len(node_path) - 1):
            from_node_idx = node_path[i]
            to_node_idx = node_path[i+1]
            link, cost = self.find_shortest_link_leading_to_node(from_node_idx, to_node_idx)
            if link:
                link_path.append(link.idx)
                total_distance += cost
                for point in link.points:
                    point_path.append([point[0], point[1], 0])

        return {
            'node_path': node_path,
            'link_path': link_path,
            'point_path': point_path,
            'total_distance': total_distance
        }


if __name__ == '__main__':
    
    tsp_path_pub = tsp_path_pub()