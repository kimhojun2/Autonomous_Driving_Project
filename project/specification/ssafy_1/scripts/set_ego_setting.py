#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from morai_msgs.msg import MultiEgoSetting

# Ego_setting_Command 는 Simulator 에서 Ego 차량의 위치를 제어하는 메세지 송신의 예제입니다.
# /ego_setting 라는 메세지를 Publish 하여 Ego 차량을 제어 합니다.

# 노드 실행 순서 
# 1. publisher 생성
# 2. 송신 될 메세지 변수 생성
# 3. /ego_setting 메세지 Publish

def talker():
    #TODO: (1) publisher 생성
    
    # MultiEgoSetting 라는 Morai ROS 메세지 형식을 사용하여 Topic Publisher 를 완성한다.
    # Topic 이름은 시뮬레이터 Network 연결시 확인 가능하다.
    publisher = rospy.Publisher('ego_settring', MultiEgoSetting, queue_size=10)

    
    rospy.init_node('Ego_setting_Command', anonymous=True)

    #TODO: (2) 송신 될 메세지 변수 생성

    # 시뮬레이터로 송신 될 메세지 변수를 만든다.
    # MultiEgoSetting 메세지는 차량의 위치와 상태를 바꾸는 명령어이다.
    # 원하는 위치에 원하는 상태 로 차량을 배치할 수 있다. 
    ego_setting_msg = MultiEgoSetting()
    ego_setting_msg.number_of_ego_vehicle = 2  # Ego 차량의 수
    ego_setting_msg.camera_index = 0  # 카메라 인덱스, 시뮬레이션에 따라 조정 가능
    ego_setting_msg.ego_index = [0, 1]  # Ego 차량 인덱스 리스트
    ego_setting_msg.global_position_x = [100, 200]  # 각 Ego 차량의 X 좌표
    ego_setting_msg.global_position_y = [100, 200]  # 각 Ego 차량의 Y 좌표
    ego_setting_msg.global_position_z = [0, 0]  # 각 Ego 차량의 Z 좌표 (일반적으로 0)
    ego_setting_msg.global_roll = [0, 0]  # 각 Ego 차량의 Roll 각도
    ego_setting_msg.global_pitch = [0, 0]  # 각 Ego 차량의 Pitch 각도
    ego_setting_msg.global_yaw = [0, 0]  # 각 Ego 차량의 Yaw 각도
    ego_setting_msg.velocity = [0, 0]  # 각 Ego 차량의 속도
    ego_setting_msg.gear = [0, 0]  # 각 Ego 차량의 기어 상태 (0: 중립)
    ego_setting_msg.ctrl_mode = [0, 0]  # 각 Ego 차량의 제어 모드
    

    rate = rospy.Rate(1) # 1 hz
    while not rospy.is_shutdown():
        rospy.loginfo(ego_setting_msg)
        #TODO: (3) /ego_setting 메세지 Publish
        
        # ego_setting_msg 를 전송하는 publisher 를 만든다.
        publisher.publish(ego_setting_msg)
        
        
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
