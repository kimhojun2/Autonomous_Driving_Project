#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os, sys
import rospy
from math import cos,sin,pi,sqrt,pow,atan2
from morai_msgs.msg  import EgoVehicleStatus,ObjectStatusList
from geometry_msgs.msg import Point,PoseStamped, Point32
from nav_msgs.msg import Path
import numpy as np
from std_msgs.msg import Bool

# lattice_planner은 충돌 회피 경로 생성 및 선택 예제입니다.
# 차량 경로상의 장애물을 탐색하여 충돌 여부의 판단은 지역경로(/local_path) 와 장애물 정보(/Object_topic)를 받아 판단합니다.
# 충돌이 예견될 경우 회피경로를 생성 및 선택 하고 새로운 지역경로(/lattice_path)를 Pulish합니다.

# 노드 실행 순서
# 0. 필수 학습 지식
# 1. subscriber, publisher 선언
# 2. 경로상의 장애물 탐색
# 3. 좌표 변환 행렬 생성
# 4. 충돌회피 경로 생성
# 5. 생성된 모든 Lattice 충돌 회피 경로 메시지 Publish
# 6. 생성된 충돌회피 경로 중 낮은 비용의 경로 선택
# 7. 선택 된 새로운 지역경로 (/lattice_path) 메세지 Publish

#TODO: (0) 필수 학습 지식
'''
# Lattice Planner 는 격자(Lattice) 점을 이용해 충돌 회피 경로를 만드는 예제 입니다.
# Lattice 경로를 만들기 위해 차량 기준 좌표계를 기준으로 경로의 폭과 완만함 정도를 결정합니다.
# Lattice Path 가 만들어질 경로를 3차 방정식을 이용하여 3 차 곡선을 생성 합니다.
# 위 방식을 이용해 차량이 주행 방향으로 회피가 가능한 여러 Lattice 경로 곡선을 만듭니다.
# 만들어진 경로 중 어떤 경로를 주행해야지 안전하기 주행 할 수 있을 지 판단합니다.
# 아래 예제는 경로 상 장애물의 유무를 판단하고 장애물이 있다면 Lattice Path 를 생성하여 회피 할 수 있는 경로를 탐색합니다.

'''
class latticePlanner:
    def __init__(self):
        rospy.init_node('lattice_planner', anonymous=True)

        '''
        #TODO: ros Launch File <arg> Tag 
        # ros launch 파일 에는 여러 태그 를 사용 할 수 있지만 
        # 그중 <arg> 태그를 사용하여 변수를 정의 할 수 있습니다.
        # 3 장 에서는 사용하는 Path 정보와 Object 각 예제 별로 다르기 때문에
        # launch 파일의 <arg> 태그를 사용하여 예제에 맞게 변수를 설정합니다.
        
        '''
        arg = rospy.myargv(argv=sys.argv)
        object_topic_name = arg[1]

        rospy.Subscriber(object_topic_name,ObjectStatusList, self.object_callback)

        #TODO: (1) subscriber, publisher 선언
        '''
        # Local/Gloabl Path 와 Odometry Ego Status 데이터를 수신 할 Subscriber 를 만들고 
        # CtrlCmd 를 시뮬레이터로 전송 할 publisher 변수를 만든다.
        # CtrlCmd 은 1장을 참고 한다.
        # Ego topic 데이터는 차량의 현재 속도를 알기 위해 사용한다.
        # Gloabl Path 데이터는 경로의 곡률을 이용한 속도 계획을 위해 사용한다.

        rospy.Subscriber( "/local_path" )
        rospy.Subscriber( "/Ego_topic" )

        self.lattice_path_pub = 


        '''
        self.local_path_sub = rospy.Subscriber("/local_path", Path, self.path_callback)
        self.ego_status_sub = rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self.status_callback)
        self.lattice_path_pub = rospy.Publisher("/lattice_path", Path, queue_size=1)
        # rospy.Subscriber("/Object_topic", ObjectStatusList, self.object_callback)
        self.collision_risk_pub = rospy.Publisher("/collision_risk", Bool, queue_size=1)
        self.is_path = False
        self.is_status = False
        self.is_obj = False
        self.is_obj_GT = False
        collision_risk = Bool()
        self.collision_message_printed = False

        rate = rospy.Rate(30) # 50hz
        while not rospy.is_shutdown():
            
            if self.is_path and self.is_status and self.is_obj:
                if self.checkObject(self.local_path, self.object_data):
                    lattice_path = self.latticePlanner(self.local_path, self.status_msg)
                    lattice_path_index = self.collision_check(self.object_data, lattice_path)
                  
                    #TODO: (7) lattice 경로 메세지 Publish
                    self.lattice_path_pub.publish(lattice_path[lattice_path_index])

                elif self.checkPedes(self.local_path, self.object_data):
                    collision_risk.data = True
                    self.collision_risk_pub.publish(collision_risk)

                else:
                    self.lattice_path_pub.publish(self.local_path)
                    collision_risk.data = False
                    self.collision_risk_pub.publish(collision_risk)
            rate.sleep()


    def checkPedes(self, ref_path, object_data):
        for pedestrian in object_data.pedestrian_list:
            # print(pedestrian)
            for path in ref_path.poses:
                distance = sqrt(pow(pedestrian.position.x - path.pose.position.x, 2) +
                                pow(pedestrian.position.y - path.pose.position.y, 2))
                if distance < 2.0: # 2미터 이내에 보행자가 있으면 충돌 위험으로 간주
                    return True
        return False


    def checkObject(self, ref_path, object_data):
        #TODO: (2) 경로상의 장애물 탐색
        '''
        # 경로 상에 존재하는 장애물을 탐색합니다.
        # 경로 상 기준이 되는 지역 경로(local path)에서 일정 거리 이상 가까이 있다면
        # in_crash 변수를 True 값을 할당합니다.

        is_crash = False
        for obstacle in object_data.obstacle_list:
            for path in ref_path.poses:  
                dis =                
                if dis < 2.35: # 장애물의 좌표값이 지역 경로 상의 좌표값과의 직선거리가 2.35 미만일때 충돌이라 판단.
                    is_crash = True
                    break

        '''
        is_crash = False
        # print(object_data)
        for obstacle in object_data.obstacle_list:
            for path in ref_path.poses:
                dis = sqrt(pow(obstacle.position.x - path.pose.position.x, 2) + pow(obstacle.position.y - path.pose.position.y, 2))
                if dis < 2.35: # 장애물과의 거리가 2.35m 미만이면 충돌 가능성 있음
                    is_crash = True
                    if not self.collision_message_printed:
                        # print('')
                        # print('')
                        # print('')
                        # print('')
                        # print('')
                        print('전방에 장애물이 있습니다.')
                        print('회피 경로로 주행합니다.')
                        self.collision_message_printed = True      
                    break

        '''
        # npc 차량도 회피경로로 주행하려면 이렇게
        for npc in object_data.npc_list:
           
            for path in ref_path.poses:
                dis = sqrt(pow(npc.position.x - path.pose.position.x, 2) + pow(npc.position.y - path.pose.position.y, 2))
                if dis < 2.35: # 차량과의 거리가 2.35m 미만이면 충돌 가능성 있음
                    is_crash = True
                    break
        '''            
        return is_crash

    def collision_check(self, object_data, out_path):
        #TODO: (6) 생성된 충돌회피 경로 중 낮은 비용의 경로 선택
        '''
        # 충돌 회피 경로를 생성 한 이후 가장 낮은 비용의 경로를 선택 합니다.
        # lane_weight 에는 기존 경로를 제외하고 좌 우로 3개씩 총 6개의 경로를 가지도록 합니다.
        # 이 중 장애물이 있는 차선에는 가중치 값을 추가합니다.
        # 모든 Path를 탐색 후 가장 비용이 낮은 Path를 선택하게 됩니다.
        # 장애물이 존제하는 차선은 가중치가 추가 되어 높은 비용을 가지게 되기 떄문에 
        # 최종적으로 가장 낮은 비용은 차선을 선택 하게 됩니다. 

        '''
        
        selected_lane = -1
        lane_weight = [3, 2, 1, 1, 2, 3] #reference path
        
        for obstacle in object_data.obstacle_list:                        
            for path_num in range(len(out_path)) :                    
                for path_pos in out_path[path_num].poses :                                
                    dis = sqrt(pow(obstacle.position.x - path_pos.pose.position.x, 2) + pow(obstacle.position.y - path_pos.pose.position.y, 2))
                    if dis < 2.5:
                        lane_weight[path_num] = lane_weight[path_num] + 100
            
        selected_lane = lane_weight.index(min(lane_weight))
        # print(lane_weight)
        # selected_lane = lane_weight.index(min(lane_weight))                    
        return selected_lane

    def path_callback(self,msg):
        self.is_path = True
        self.local_path = msg  
        
    def status_callback(self,msg): ## Vehicl Status Subscriber 
        self.is_status = True
        self.status_msg = msg

    def object_callback(self,msg):
        self.is_obj = True
        self.object_data = msg

    
    def latticePlanner(self,ref_path, vehicle_status):
        out_path = []
        vehicle_pose_x = vehicle_status.position.x
        vehicle_pose_y = vehicle_status.position.y
        vehicle_velocity = vehicle_status.velocity.x * 3.6

        # look_distance = int(vehicle_velocity * 0.2 * 2)
        look_distance = int(vehicle_velocity * 0.05 * 2)
        
        # if look_distance < 20 : #min 10m   
        #     look_distance = 20  
        if look_distance < 10 : # 최소 거리를 10m로 조정
            look_distance = 10                

        if len(ref_path.poses) > look_distance :
            #TODO: (3) 좌표 변환 행렬 생성
            """
            # 좌표 변환 행렬을 만듭니다.
            # Lattice 경로를 만들기 위해서 경로 생성을 시작하는 Point 좌표에서 
            # 경로 생성이 끝나는 Point 좌표의 상대 위치를 계산해야 합니다.
            
            """

            global_ref_start_point      = (ref_path.poses[0].pose.position.x, ref_path.poses[0].pose.position.y)
            global_ref_start_next_point = (ref_path.poses[1].pose.position.x, ref_path.poses[1].pose.position.y)

            global_ref_end_point = (ref_path.poses[look_distance * 2].pose.position.x, ref_path.poses[look_distance * 2].pose.position.y)
            
            theta = atan2(global_ref_start_next_point[1] - global_ref_start_point[1], global_ref_start_next_point[0] - global_ref_start_point[0])
            translation = [global_ref_start_point[0], global_ref_start_point[1]]

            trans_matrix    = np.array([    [cos(theta),                -sin(theta),                                                                      translation[0]], 
                                            [sin(theta),                 cos(theta),                                                                      translation[1]], 
                                            [         0,                          0,                                                                                  1 ]     ])

            det_trans_matrix = np.array([   [trans_matrix[0][0], trans_matrix[1][0],        -(trans_matrix[0][0] * translation[0] + trans_matrix[1][0] * translation[1])], 
                                            [trans_matrix[0][1], trans_matrix[1][1],        -(trans_matrix[0][1] * translation[0] + trans_matrix[1][1] * translation[1])],
                                            [                 0,                  0,                                                                                   1]     ])

            world_end_point = np.array([[global_ref_end_point[0]], [global_ref_end_point[1]], [1]])
            local_end_point = det_trans_matrix.dot(world_end_point)
            world_ego_vehicle_position = np.array([[vehicle_pose_x], [vehicle_pose_y], [1]])
            local_ego_vehicle_position = det_trans_matrix.dot(world_ego_vehicle_position)
            lane_off_set = [-3.0, -1.75, -1, 1, 1.75, 3.0]
            local_lattice_points = []
            
            for i in range(len(lane_off_set)):
                local_lattice_points.append([local_end_point[0][0], local_end_point[1][0] + lane_off_set[i], 1])
            
            #TODO: (4) Lattice 충돌 회피 경로 생성
            '''
            # Local 좌표계로 변경 후 3차곡선계획법에 의해 경로를 생성한 후 다시 Map 좌표계로 가져옵니다.
            # Path 생성 방식은 3차 방정식을 이용하며 lane_change_ 예제와 동일한 방식의 경로 생성을 하면 됩니다.
            # 생성된 Lattice 경로는 out_path 변수에 List 형식으로 넣습니다.
            # 충돌 회피 경로는 기존 경로를 제외하고 좌 우로 3개씩 총 6개의 경로를 가지도록 합니다.
                
            for end_point in local_lattice_points :

            '''
            for end_point in local_lattice_points :
                lattice_path = Path()
                lattice_path.header.frame_id = 'map'
                x = []
                y = []
                x_interval = 0.5
                xs = 0
                xf = end_point[0]
                ps = local_ego_vehicle_position[1][0]

                pf = end_point[1]
                x_num = xf / x_interval

                for i in range(xs,int(x_num)) : 
                    x.append(i*x_interval)
                
                a = [0.0, 0.0, 0.0, 0.0]
                a[0] = ps
                a[1] = 0
                a[2] = 3.0 * (pf - ps) / (xf * xf)
                a[3] = -2.0 * (pf - ps) / (xf * xf * xf)

                for i in x:
                    result = a[3] * i * i * i + a[2] * i * i + a[1] * i + a[0]
                    y.append(result)

                for i in range(0,len(y)) :
                    local_result = np.array([[x[i]], [y[i]], [1]])
                    global_result = trans_matrix.dot(local_result)

                    read_pose = PoseStamped()
                    read_pose.pose.position.x = global_result[0][0]
                    read_pose.pose.position.y = global_result[1][0]
                    read_pose.pose.position.z = 0
                    read_pose.pose.orientation.x = 0
                    read_pose.pose.orientation.y = 0
                    read_pose.pose.orientation.z = 0
                    read_pose.pose.orientation.w = 1
                    lattice_path.poses.append(read_pose)

                out_path.append(lattice_path)

            #Add_point            
            add_point_size = min(int(vehicle_velocity * 2), len(ref_path.poses) )           
            
            for i in range(look_distance*2,add_point_size):
                if i+1 < len(ref_path.poses):
                    tmp_theta = atan2(ref_path.poses[i + 1].pose.position.y - ref_path.poses[i].pose.position.y,ref_path.poses[i + 1].pose.position.x - ref_path.poses[i].pose.position.x)                    
                    tmp_translation = [ref_path.poses[i].pose.position.x,ref_path.poses[i].pose.position.y]
                    tmp_t = np.array([[cos(tmp_theta), -sin(tmp_theta), tmp_translation[0]], [sin(tmp_theta), cos(tmp_theta), tmp_translation[1]], [0, 0, 1]])

                    for lane_num in range(len(lane_off_set)) :
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
            
            #TODO: (5) 생성된 모든 Lattice 충돌 회피 경로 메시지 Publish
            '''
            # 생성된 모든 Lattice 충돌회피 경로는 ros 메세지로 송신하여
            # Rviz 창에서 시각화 하도록 합니다.

            for i in range(len(out_path)):          
                globals()['lattice_pub_{}'.format(i+1)] = rospy.Publisher('/lattice_path_{}'.format(i+1),Path,queue_size=1)
                globals()['lattice_pub_{}'.format(i+1)].publish(out_path[i])

            '''
            # Publisher 리스트 초기화
            lattice_path_pubs = []

            # 각 경로에 대한 Publisher 생성
            for i in range(len(out_path)):
                pub = rospy.Publisher('/lattice_path_{}'.format(i+1), Path, queue_size=1)
                lattice_path_pubs.append(pub)

            # 잠시 대기하여 Publisher가 준비될 수 있도록 함
            rospy.sleep(1)

            # 각 경로 Publish
            for i, path in enumerate(out_path):
                # 경로 메시지에 현재 시간을 헤더에 설정
                path.header.stamp = rospy.Time.now()
                path.header.frame_id = "map"  # 경로가 참조하는 좌표계 설정
                # 해당 경로 Publish
                lattice_path_pubs[i].publish(path)

        return out_path

if __name__ == '__main__':
    try:
        latticePlanner()
    except rospy.ROSInterruptException:
        pass
