#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import rospkg
from math import cos,sin,pi,sqrt,pow,atan2
from geometry_msgs.msg import Point,PoseWithCovarianceStamped
from nav_msgs.msg import Odometry,Path
from morai_msgs.msg import CtrlCmd,EgoVehicleStatus,GPSMessage, Lamps, ObjectStatusList
from std_msgs.msg import Bool
import numpy as np
import tf
from tf.transformations import euler_from_quaternion,quaternion_from_euler
import sys
from std_msgs.msg import String

import json

import firebase_admin
from firebase_admin import firestore, messaging, credentials
import time
from datetime import datetime
from pyproj import Proj, Transformer
import pytz

# advanced_purepursuit 은 차량의 차량의 종 횡 방향 제어 예제입니다.
# Purpusuit 알고리즘의 Look Ahead Distance 값을 속도에 비례하여 가변 값으로 만들어 횡 방향 주행 성능을 올립니다.
# 횡방향 제어 입력은 주행할 Local Path (지역경로) 와 차량의 상태 정보 Odometry 를 받아 차량을 제어 합니다.
# 종방향 제어 입력은 목표 속도를 지정 한뒤 목표 속도에 도달하기 위한 Throttle control 을 합니다.
# 종방향 제어 입력은 longlCmdType 1(Throttle control) 이용합니다.

# 노드 실행 순서 
# 0. 필수 학습 지식
# 1. subscriber, publisher 선언
# 2. 속도 비례 Look Ahead Distance 값 설정
# 3. 좌표 변환 행렬 생성
# 4. Steering 각도 계산
# 5. PID 제어 생성
# 6. 도로의 곡률 계산
# 7. 곡률 기반 속도 계획
# 8. 제어입력 메세지 Publish

#TODO: (0) 필수 학습 지식

# advanced_purepursuit 은 Pure Pursuit 알고리즘을 강화 한 예제입니다.
# 이전까지 사용한 Pure Pursuit 알고리즘은 고정된 전방주시거리(Look Forward Distance) 값을 사용하였습니다.
# 해당 예제에서는 전방주시거리(Look Forward Distance) 값을 주행 속도에 비례한 값으로 설정합니다.
# 이때 최소 최대 전방주시거리(Look Forward Distance) 를 설정합니다.
# 주행 속도에 비례한 값으로 변경 한 뒤 "self.lfd_gain" 을 변경 하여서 직접 제어기 성능을 튜닝 해보세요.
# 


class pure_pursuit :
    def __init__(self):
        rospy.init_node('pure_pursuit', anonymous=True)

        self.proj_UTM = Proj(proj='utm', zone=52, ellps='WGS84', preserve_units=False)
        self.proj_WGS84 = Proj(proj='latlong', datum='WGS84')

        arg = rospy.myargv(argv=sys.argv)
        object_topic_name = arg[1]
        
        collision_risk_topic = sys.argv[2]
        rospy.Subscriber(collision_risk_topic, Bool, self.collision_risk_callback)
        rospy.Subscriber(object_topic_name,Path, self.lattice_path_callback)

        # Firebase Admin SDK 초기화
        if not firebase_admin._apps:
            self.cred = firebase_admin.credentials.Certificate("/home/morai/catkin_ws/src/ifind/key/ifind-firebase-adminsdk.json")
            self.default_app = firebase_admin.initialize_app(self.cred)
        self.db = firestore.client()

        # 인식 전까지는 False 고정
        self.collision_risk = False

        self.forward_point = [0, 0]
        self.current_postion = Point()

        #TODO: (1) subscriber, publisher 선언
        
        # Local/Gloabl Path 와 Odometry Ego Status 데이터를 수신 할 Subscriber 를 만들고 
        # CtrlCmd 를 시뮬레이터로 전송 할 publisher 변수를 만든다.
        # CtrlCmd 은 1장을 참고 한다.
        # Ego topic 데이터는 차량의 현재 속도를 알기 위해 사용한다.
        # Gloabl Path 데이터는 경로의 곡률을 이용한 속도 계획을 위해 사용한다.
        rospy.Subscriber("/global_path", Path, self.global_path_callback)
        rospy.Subscriber("/local_path", Path, self.path_callback)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self.status_callback)
        # rospy.Subscriber("/lattice_path", Path, self.path_callback)
        rospy.Subscriber("/child_zone_detected", Bool, self.zone_callback)
        self.gps_sub = rospy.Subscriber("/gps", GPSMessage, self.init_callback)

        # ACC 를 위한 장애물 정보 구독 
        # rospy.Subscriber("/Object_topic", ObjectStatusList,  self.object_info_callback)


        self.current_node_index = 0
        self.visit_order = []
        self.visit_order_sub = rospy.Subscriber("/visit_order_topic", String, self.visit_order_callback)

        self.ctrl_cmd_pub = rospy.Publisher("/ctrl_cmd", CtrlCmd, queue_size=1)

        self.ctrl_cmd_msg = CtrlCmd()
        self.ctrl_cmd_msg.longlCmdType = 1

        self.is_path = False
        self.is_odom = False
        self.is_status = False
        self.is_global_path = False
        self.is_lattice_path = False
        self.is_look_forward_point = False
        self.zone_active = False

        self.lamp_cmd_pub = rospy.Publisher("/lamps", Lamps, queue_size=1)
        

        self.vehicle_length = 2.6
        self.lfd = 8
        self.min_lfd = 5
        self.max_lfd = 50
        self.lfd_gain = 0.78
        self.target_velocity = 40

        self.pid = pidControl()
        # self.adaptive_cruise_control = AdaptiveCruiseControl(velocity_gain = 0.5, distance_gain = 1, time_gap = 0.8, vehicle_length = 2.7)
        self.vel_planning = velocityPlanning(self.target_velocity/3.6, 0.15)
        while True:
            if self.is_global_path == True:
                self.velocity_list = self.vel_planning.curvedBaseVelocity(self.global_path, 50)
                break
            else:
                # rospy.loginfo('Waiting global path data')
                pass


        rate = rospy.Rate(30) # 50hz
        while not rospy.is_shutdown():

            if self.is_path == True and self.is_odom == True and self.is_status == True:

                # 차량이 노드에 도착했다면 일시 정지 및 알림
                self.check_arrival_and_alert()

                # if self.collision_risk:
                #     self.ctrl_cmd_msg.accel = 0
                #     self.ctrl_cmd_msg.brake = 0.5
                # else:
                # result = self.calc_vaild_obj([self.current_postion.x,self.current_postion.y,self.vehicle_yaw],self.object_data)

                # global_npc_info = result[0] 
                # local_npc_info = result[1] 
                # global_ped_info = result[2] 
                # local_ped_info = result[3] 
                # global_obs_info = result[4] 
                # local_obs_info = result[5] 

                self.current_waypoint = self.get_current_waypoint(self.status_msg,self.global_path)
                self.target_velocity = self.velocity_list[self.current_waypoint]*3.6

                if self.zone_active:
                    if self.target_velocity > 25:
                        self.target_velocity = 25  # 감지되면 속도를 30으로 설정


                # rospy.loginfo("Zone active. Speed set to 30.")
                # elif not self.zone_active:
                #      self.target_velocity = 60  # 감지가 없으면 속도를 50으로 복구

                steering = self.calc_pure_pursuit()
                if self.is_look_forward_point :
                    self.ctrl_cmd_msg.steering = steering
                else : 
                    # rospy.loginfo("no found forward point")
                    self.ctrl_cmd_msg.steering = 0.0


                # self.adaptive_cruise_control.check_object(self.path ,global_npc_info, local_npc_info
                                                                # ,global_ped_info, local_ped_info
                                                                # ,global_obs_info, local_obs_info)
                
                # self.target_velocity = self.adaptive_cruise_control.get_target_velocity(local_npc_info, local_ped_info, local_obs_info,
                                                                                                    # self.status_msg.velocity.x, self.target_velocity/3.6)
                

                output = self.pid.pid(self.target_velocity,self.status_msg.velocity.x*3.6)

                
                if output > 0.0:
                    self.ctrl_cmd_msg.accel = output
                    self.ctrl_cmd_msg.brake = 0.0
                else:
                    self.ctrl_cmd_msg.accel = 0.0
                    self.ctrl_cmd_msg.brake = -output

                #TODO: (8) 제어입력 메세지 Publish

                # 제어입력 메세지 를 전송하는 publisher 를 만든다.
                self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)
            
            rate.sleep()


    # def object_info_callback(self,data): ## Object information Subscriber
    #     self.is_object_info = True
    #     self.object_data = data 

    def init_callback(self,msg):
        self.lat = msg.latitude
        self.lon = msg.longitude
        self.e_o = msg.eastOffset
        self.n_o = msg.northOffset

        self.is_gps=True
        
        self.current_latlon = self.convert_latlon_to_xy(self.lon, self.lat)

    def convert_latlon_to_xy(self, lon, lat):
        # 위도와 경도를 UTM 좌표계로 변환
        xy_zone = self.proj_UTM(lon, lat)

        x = 0.0
        y = 0.0
        if lon == 0 and lat == 0:
            x = 0.0
            y = 0.0
        else:
            x = xy_zone[0] - self.e_o
            y = xy_zone[1] - self.n_o    

        return {'x' : x, 'y' : y}

    def check_arrival_and_alert(self):
    
        if self.current_node_index >= len(self.visit_order):
            return  # 모든 노드를 방문했다면 함수 종료
        
        current_node = self.visit_order[self.current_node_index]
        distance_to_node = self.calculate_distance(self.current_latlon, current_node)
        if distance_to_node < 10:
            self.ctrl_cmd_msg.accel = 0
            self.ctrl_cmd_msg.brake = 0.4
            self.ctrl_cmd_msg.steering = self.calc_pure_pursuit()
            self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)

            tz = pytz.timezone('Asia/Seoul')
            current_time = datetime.now(tz)

            # 비상등 켜기
            lamp_cmd = Lamps()
            lamp_cmd.emergencySignal = 1

            self.lamp_cmd_pub.publish(lamp_cmd)
                    
            # 방문 노드 알림
            alert_data = {
                'type': 1,
                'time': current_time
            }

            # 알림 db 쓰기
            if self.current_node_index == len(self.visit_order) - 1:
                alert_data['type'] = 2
                for visit_node in self.visit_order[:-1]:
                    for user_id in visit_node['select']:
                        user_ref = self.db.collection('users').document(user_id)
                        user_ref.update({'alert': firestore.ArrayUnion([alert_data])}) 

                        self.set_push_alert(user_ref, '멀티캠퍼스')
            else:
                for user_id in current_node['select']:
                    user_ref = self.db.collection('users').document(user_id)
                    user_ref.update({'alert': firestore.ArrayUnion([alert_data])})

                    self.set_push_alert(user_ref, current_node['stop'])


            rospy.sleep(6)

            # 비상등 끄기
            lamp_cmd.emergencySignal = 0
            self.lamp_cmd_pub.publish(lamp_cmd)

            current_node = self.visit_order[self.current_node_index]
            self.current_node_index += 1

    def set_push_alert(self, user_ref, stop):
        user_doc = user_ref.get()
        if user_doc.exists:
                user_data = user_doc.to_dict()
                fcm_token = user_data.get('fcmToken')
                if fcm_token:
                    message = messaging.Message(
                        notification=messaging.Notification(
                            title="아이파인",  
                            body=f"{stop}에 도착했습니다" 
                        ),
                        token=fcm_token,
                    )
                    response = messaging.send(message)


    def calculate_distance(self, position1, position2):
        # 두 위치 사이의 거리를 계산하는 함수
        dx = position1['x'] - position2['x']
        dy = position1['y'] - position2['y']
        return sqrt(dx**2 + dy**2)

    def visit_order_callback(self, msg):
        # 게시된 JSON 문자열을 파싱하여 Python 리스트로 변환
        self.visit_order = json.loads(msg.data)
        rospy.loginfo("----")


    def zone_callback(self, msg):

        # rospy.loginfo(f"Child zone detected: {msg.data}, Current zone active status: {self.zone_active}")
        if msg.data and not self.zone_active:
            self.zone_active = msg.data
            rospy.loginfo("스쿨존 진입. 스피드 30km로")

        elif not msg.data and self.zone_active:
            self.zone_active = msg.data
            rospy.loginfo("스쿨존 벗어남. 스피드 60km로")

    def path_callback(self,msg):
        self.is_path=True
        self.path=msg  

    def lattice_path_callback(self, msg):
        self.lattice_path = msg
        self.is_lattice_path = True 

    def odom_callback(self,msg):
        self.is_odom=True
        odom_quaternion=(msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w)
        _,_,self.vehicle_yaw=euler_from_quaternion(odom_quaternion)
        self.current_postion.x=msg.pose.pose.position.x
        self.current_postion.y=msg.pose.pose.position.y

    def status_callback(self,msg): ## Vehicl Status Subscriber
        self.is_status=True
        self.status_msg=msg
        
    def global_path_callback(self,msg):
        self.global_path = msg
        self.is_global_path = True


    def get_current_waypoint(self,ego_status,global_path):
        min_dist = float('inf')        
        currnet_waypoint = -1
        for i,pose in enumerate(global_path.poses):
            dx = ego_status.position.x - pose.pose.position.x
            dy = ego_status.position.y - pose.pose.position.y

            dist = sqrt(pow(dx,2)+pow(dy,2))
            if min_dist > dist :
                min_dist = dist
                currnet_waypoint = i
        return currnet_waypoint

    def collision_risk_callback(self, msg):
        self.collision_risk = msg.data
        
    def calc_vaild_obj(self,status_msg,object_data):
        
        self.all_object = object_data        
        ego_pose_x = status_msg[0]
        ego_pose_y = status_msg[1]
        ego_heading = status_msg[2]
        
        global_npc_info = []
        local_npc_info  = []
        global_ped_info = []
        local_ped_info  = []
        global_obs_info = []
        local_obs_info  = []

        num_of_object = self.all_object.num_of_npcs + self.all_object.num_of_obstacle + self.all_object.num_of_pedestrian        
        if num_of_object > 0:

            #translation
            tmp_theta=ego_heading
            tmp_translation=[ego_pose_x, ego_pose_y]
            tmp_t=np.array([[cos(tmp_theta), -sin(tmp_theta), tmp_translation[0]],
                            [sin(tmp_theta),  cos(tmp_theta), tmp_translation[1]],
                            [0             ,               0,                  1]])
            tmp_det_t=np.array([[tmp_t[0][0], tmp_t[1][0], -(tmp_t[0][0] * tmp_translation[0] + tmp_t[1][0]*tmp_translation[1])],
                                [tmp_t[0][1], tmp_t[1][1], -(tmp_t[0][1] * tmp_translation[0] + tmp_t[1][1]*tmp_translation[1])],
                                [0,0,1]])

            #npc vehicle ranslation        
            for npc_list in self.all_object.npc_list:
                global_result=np.array([[npc_list.position.x],[npc_list.position.y],[1]])
                local_result=tmp_det_t.dot(global_result)
                if local_result[0][0]> 0 :        
                    global_npc_info.append([npc_list.type,npc_list.position.x,npc_list.position.y,npc_list.velocity.x])
                    local_npc_info.append([npc_list.type,local_result[0][0],local_result[1][0],npc_list.velocity.x])

            #ped translation
            for ped_list in self.all_object.pedestrian_list:
                global_result=np.array([[ped_list.position.x],[ped_list.position.y],[1]])
                local_result=tmp_det_t.dot(global_result)
                if local_result[0][0]> 0 :
                    global_ped_info.append([ped_list.type,ped_list.position.x,ped_list.position.y,ped_list.velocity.x])
                    local_ped_info.append([ped_list.type,local_result[0][0],local_result[1][0],ped_list.velocity.x])

            #obs translation
            for obs_list in self.all_object.obstacle_list:
                global_result=np.array([[obs_list.position.x],[obs_list.position.y],[1]])
                local_result=tmp_det_t.dot(global_result)
                if local_result[0][0]> 0 :
                    global_obs_info.append([obs_list.type,obs_list.position.x,obs_list.position.y,obs_list.velocity.x])
                    local_obs_info.append([obs_list.type,local_result[0][0],local_result[1][0],obs_list.velocity.x])
                
        return global_npc_info, local_npc_info, global_ped_info, local_ped_info, global_obs_info, local_obs_info

    def calc_pure_pursuit(self,):
        

        #TODO: (2) 속도 비례 Look Ahead Distance 값 설정
        
        # 차량 속도에 비례하여 전방주시거리(Look Forward Distance) 가 변하는 수식을 구현 합니다.
        # 이때 'self.lfd' 값은 최소와 최대 값을 넘어서는 안됩니다.
        # "self.min_lfd","self.max_lfd", "self.lfd_gain" 을 미리 정의합니다.
        # 최소 최대 전방주시거리(Look Forward Distance) 값과 속도에 비례한 lfd_gain 값을 직접 변경해 볼 수 있습니다.
        # 초기 정의한 변수 들의 값을 변경하며 속도에 비례해서 전방주시거리 가 변하는 advanced_purepursuit 예제를 완성하세요.
        # 
        current_speed = self.status_msg.velocity.x 
        self.lfd = max(self.min_lfd, min(self.lfd_gain * current_speed, self.max_lfd))

        # rospy.loginfo(self.lfd)

        
        
        vehicle_position=self.current_postion
        self.is_look_forward_point= False

        translation = [vehicle_position.x, vehicle_position.y]

        #TODO: (3) 좌표 변환 행렬 생성
        
        # Pure Pursuit 알고리즘을 실행 하기 위해서 차량 기준의 좌표계가 필요합니다.
        # Path 데이터를 현재 차량 기준 좌표계로 좌표 변환이 필요합니다.
        # 좌표 변환을 위한 좌표 변환 행렬을 작성합니다.
        # Path 데이터를 차량 기준 좌표 계로 변환 후 Pure Pursuit 알고리즘 중 전방주시거리(Look Forward Distance) 와 가장 가까운 Path Point 를 찾습니다.
        # 전방주시거리(Look Forward Distance) 와 가장 가까운 Path Point 를 이용하여 조향 각도를 계산하게 됩니다.
        # 좌표 변환 행렬을 이용해 Path 데이터를 차량 기준 좌표 계로 바꾸는 반복 문을 작성 한 뒤
        # 전방주시거리(Look Forward Distance) 와 가장 가까운 Path Point 를 계산하는 로직을 작성 하세요.

        trans_matrix = np.array([[cos(self.vehicle_yaw), -sin(self.vehicle_yaw), translation[0]],
                                [sin(self.vehicle_yaw), cos(self.vehicle_yaw), translation[1]],
                                [0, 0, 1]])

        det_trans_matrix = np.linalg.inv(trans_matrix)
        if self.is_lattice_path == True:
            for num,i in enumerate(self.lattice_path.poses) :
                path_point = i.pose.position

                global_path_point = [path_point.x, path_point.y, 1]
                local_path_point = det_trans_matrix.dot(global_path_point)    

                if local_path_point[0]>0 :
                    dis = sqrt(local_path_point[0]**2 + local_path_point[1]**2)
                    if dis >= self.lfd :
                        self.forward_point = local_path_point
                        self.is_look_forward_point = True
                        break
        else:            
            for num,i in enumerate(self.path.poses) :
                path_point = i.pose.position

                global_path_point = [path_point.x, path_point.y, 1]
                local_path_point = det_trans_matrix.dot(global_path_point)    

                if local_path_point[0]>0 :
                    dis = sqrt(local_path_point[0]**2 + local_path_point[1]**2)
                    if dis >= self.lfd :
                        self.forward_point = local_path_point
                        self.is_look_forward_point = True
                        break

        
        #TODO: (4) Steering 각도 계산
        
        # 제어 입력을 위한 Steering 각도를 계산 합니다.
        # theta 는 전방주시거리(Look Forward Distance) 와 가장 가까운 Path Point 좌표의 각도를 계산 합니다.
        # Steering 각도는 Pure Pursuit 알고리즘의 각도 계산 수식을 적용하여 조향 각도를 계산합니다.
        theta = atan2(self.forward_point[1], self.forward_point[0])
        steering = atan2(2.0 * self.vehicle_length * sin(theta), self.lfd)

        
        return steering

class pidControl:
    def __init__(self):
        self.p_gain = 0.3
        self.i_gain = 0.00
        self.d_gain = 0.03
        self.prev_error = 0 
        self.i_control = 0
        self.controlTime = 0.02

    def pid(self,target_vel, current_vel):
        error = target_vel - current_vel

        #TODO: (5) PID 제어 생성
        
        # 종방향 제어를 위한 PID 제어기는 현재 속도와 목표 속도 간 차이를 측정하여 Accel/Brake 값을 결정 합니다.
        # 각 PID 제어를 위한 Gain 값은 "class pidContorl" 에 정의 되어 있습니다.
        # 각 PID Gain 값을 직접 튜닝하고 아래 수식을 채워 넣어 P I D 제어기를 완성하세요.

        p_control = self.p_gain * error
        self.i_control += self.i_gain * error * self.controlTime
        d_control = self.d_gain * (error - self.prev_error) / self.controlTime
        output = p_control + self.i_control + d_control
        self.prev_error = error

        return output

class velocityPlanning:
    def __init__ (self,car_max_speed, road_friciton):
        self.car_max_speed = car_max_speed
        self.road_friction = road_friciton

    def curvedBaseVelocity(self, gloabl_path, point_num):
        out_vel_plan = []

        for i in range(0,point_num):
            out_vel_plan.append(self.car_max_speed)

        for i in range(point_num, len(gloabl_path.poses) - point_num):
            x_list = []
            y_list = []
            for box in range(-point_num, point_num):
                x = gloabl_path.poses[i+box].pose.position.x
                y = gloabl_path.poses[i+box].pose.position.y
                # x_list.append([-2*x, -2*y ,1])
                # y_list.append((-x*x) - (y*y))
                x_list.append(x)
                y_list.append(y)

            #TODO: (6) 도로의 곡률 계산
            
            # 도로의 곡률 반경을 계산하기 위한 수식입니다.
            # Path 데이터의 좌표를 이용해서 곡선의 곡률을 구하기 위한 수식을 작성합니다.
            # 원의 좌표를 구하는 행렬 계산식, 최소 자승법을 이용하는 방식 등 곡률 반지름을 구하기 위한 식을 적용 합니다.
            # 적용한 수식을 통해 곡률 반지름 "r" 을 계산합니다.

            x_start  = x_list[0]
            x_end    = x_list[-1]
            x_mid    = x_list[int(len(x_list)/2)]

            y_start  = y_list[0]
            y_end    = y_list[-1]
            y_mid    = y_list[int(len(y_list)/2)]

            dSt = np.array([x_start - x_mid, y_start - y_mid])
            dEd = np.array([x_end - x_mid, y_end - y_mid])

            Dcom = 2 * (dSt[0]*dEd[1] - dSt[1]*dEd[0])

            dSt2 = np.dot(dSt,dSt)
            dEd2 = np.dot(dEd,dEd)

            U1 = (dEd[1] * dSt2 - dSt[1] * dEd2)/Dcom
            U2 = (dSt[0] * dEd2 - dEd[0] * dSt2)/Dcom

            r = sqrt(pow(U1, 2)+ pow(U2, 2))

        
            #TODO: (7) 곡률 기반 속도 계획
            
            # 계산 한 곡률 반경을 이용하여 최고 속도를 계산합니다.
            # 평평한 도로인 경우 최대 속도를 계산합니다. 
            # 곡률 반경 x 중력가속도 x 도로의 마찰 계수 계산 값의 제곱근이 됩니다.
            v_max = sqrt(r * 9.81 * self.road_friction)

            if v_max > self.car_max_speed:
                v_max = self.car_max_speed
            out_vel_plan.append(v_max)

        for i in range(len(gloabl_path.poses) - point_num, len(gloabl_path.poses)-10):
            out_vel_plan.append(30)

        for i in range(len(gloabl_path.poses) - 10, len(gloabl_path.poses)):
            out_vel_plan.append(0)

        return out_vel_plan


# class AdaptiveCruiseControl:
#     def __init__(self, velocity_gain, distance_gain, time_gap, vehicle_length):
#         self.npc_vehicle=[False,0]
#         self.object=[False,0]
#         self.Person=[False,0]
#         self.velocity_gain = velocity_gain
#         self.distance_gain = distance_gain
#         self.time_gap = time_gap
#         self.vehicle_length = vehicle_length

#         self.object_type = None
#         self.object_distance = 0
#         self.object_velocity = 0

#     def check_object(self,ref_path, global_npc_info, local_npc_info, 
#                                     global_ped_info, local_ped_info, 
#                                     global_obs_info, local_obs_info):
#         #TODO: (8) 경로상의 장애물 유무 확인 (차량, 사람, 정지선 신호)
        
#         # 주행 경로 상의 장애물의 유무를 파악합니다.
#         # 장애물이 한개 이상 있다면 self.object 변수의 첫번째 값을 True 로 둡니다.
#         # 장애물의 대한 정보는 List 형식으로 self.object 변수의 두번째 값으로 둡니다.
#         # 장애물의 유무 판단은 주행 할 경로에서 얼마나 떨어져 있는지를 보고 판단 합니다.
#         # 아래 예제는 주행 경로에서 Object 까지의 거리를 파악하여 
#         # 경로를 기준으로 2.5 m 안쪽에 있다면 주행 경로 내 장애물이 있다고 판단 합니다.
#         # 주행 경로 상 장애물이 여러게 있는 경우 가장 가까이 있는 장애물 정보를 가지도록 합니다.

        

    
#         # # 주행 경로 상 보행자 유무 파악
#         min_rel_distance=float('inf')
#         # if len(global_ped_info) > 0 :        
#         #     for i in range(len(global_ped_info)):
#         #         for path in ref_path.poses :      
#         #             if global_ped_info[i][0] == 0 : # type=0 [pedestrian]                    
#         #                 dis = sqrt((path.pose.position.x - global_ped_info[i][1])**2 + (path.pose.position.y - global_ped_info[i][2])**2)
#         #                 if dis<2.35:                            
#         #                     rel_distance = dis
#         #                     if rel_distance < min_rel_distance:
#         #                         min_rel_distance = rel_distance
#         #                         self.Person=[True,i]

        

        
#         # 주행 경로 상 NPC 차량 유무 파악
#         if len(global_npc_info) > 0 :            
#             for i in range(len(global_npc_info)):
#                 for path in ref_path.poses :      
#                     if global_npc_info[i][0] == 1 : # type=1 [npc_vehicle] 
#                         dis = sqrt((path.pose.position.x - global_npc_info[0][1])**2 + (path.pose.position.y - global_npc_info[0][2])**2)
#                         if dis<2.35:
#                             rel_distance = dis
#                             if rel_distance < min_rel_distance:
#                                 min_rel_distance = dis
#                                 self.npc_vehicle=[True,i]
        
        

        
#         # 주행 경로 상 Obstacle 유무 파악
#         # acc 예제는 주행 중 전방에 차량에 속도에 맞춰 움직이도록 하는 Cruise Control
#         # 예제 이기 때문에 정적 장애물(Obstacle) 의 정보는 받지 않는게 좋습니다.
#         # 정적 장애물은 움직이지 않기 때문에 Cruise Control 알고리즘 상
#         # 정적 장애물을 만나게 되면 속도가 0인 정적 장애물 바로 뒤에 정지하게 됩니다.
#         # if len(global_obs_info) > 0 :            
#         #     for i in range(len(global_obs_info)):
#         #         for path in ref_path.poses :      
#         #             if global_obs_info[i][0] == 2 : # type=1 [obstacle] 
#         #                 dis = sqrt((path.pose.position.x - global_obs_info[i][1])**2 + (path.pose.position.y - global_obs_info[i][2])**2)
#         #                 if dis<2.35:
#         #                     rel_distance = dis
#         #                     if rel_distance < min_rel_distance:
#         #                         min_rel_distance = rel_distance
#         #                         # self.object=[True,i] 



#     def get_target_velocity(self, local_npc_info, local_ped_info, local_obs_info, ego_vel, target_vel): 
#         #TODO: (9) 장애물과의 속도와 거리 차이를 이용하여 ACC 를 진행 목표 속도를 설정
#         out_vel =  target_vel
#         default_space = 8
#         time_gap = self.time_gap
#         v_gain = self.velocity_gain
#         x_errgain = self.distance_gain

#         if self.npc_vehicle[0] and len(local_npc_info) != 0: #ACC ON_vehicle   
#             # print("ACC ON NPC_Vehicle")         
#             front_vehicle = [local_npc_info[self.npc_vehicle[1]][1], local_npc_info[self.npc_vehicle[1]][2], local_npc_info[self.npc_vehicle[1]][3]]
            
#             dis_safe = ego_vel * time_gap + default_space
#             dis_rel = sqrt(pow(front_vehicle[0],2) + pow(front_vehicle[1],2))            
#             vel_rel=((front_vehicle[2] / 3.6) - ego_vel)                        
#             acceleration = vel_rel * v_gain - x_errgain * (dis_safe - dis_rel)

#             out_vel = ego_vel + acceleration      

#         if self.Person[0] and len(local_ped_info) != 0: #ACC ON_Pedestrian
#             # print("ACC ON Pedestrian")
#             Pedestrian = [local_ped_info[self.Person[1]][1], local_ped_info[self.Person[1]][2], local_ped_info[self.Person[1]][3]]
            
#             dis_safe = ego_vel* time_gap + default_space
#             dis_rel = sqrt(pow(Pedestrian[0],2) + pow(Pedestrian[1],2))            
#             vel_rel = (Pedestrian[2] - ego_vel)              
#             acceleration = vel_rel * v_gain - x_errgain * (dis_safe - dis_rel)    

#             out_vel = ego_vel + acceleration
   
#         # if self.object[0] and len(local_obs_info) != 0: #ACC ON_obstacle     
#         #     print("ACC ON Obstacle")                    
#         #     Obstacle = [local_obs_info[self.object[1]][1], local_obs_info[self.object[1]][2], local_obs_info[self.object[1]][3]]
            
#         #     dis_safe = ego_vel* time_gap + default_space
#         #     dis_rel = sqrt(pow(Obstacle[0],2) + pow(Obstacle[1],2))            
#         #     vel_rel = (Obstacle[2] - ego_vel)
#         #     acceleration = vel_rel * v_gain - x_errgain * (dis_safe - dis_rel)    

#         #     out_vel = ego_vel + acceleration       

#         out_vel_km_h = min(max(out_vel * 3.6, 0), 50)
#         return out_vel_km_h
    

if __name__ == '__main__':
    try:
        test_track=pure_pursuit()
        
    except rospy.ROSInterruptException:
        pass
