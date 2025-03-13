### 2024.02.26 (월)

📌 주제 회의

주제

```
1. 학원버스
2. 투어버스
3. 통근버스
```

팀원 의견

```
1. 일단 구현을 목표로 하고 나중에 기능을 추가하자
2. 모든 주제가 기능적인 부분에서 큰 차이가 없는데 발표했을 때 사람들의 공감을 끌어낼 수 있는 부분을 생각해보자
3. 같은 지도에서 진행한다고 하면 장애물이 다 JSON 형태로 되어있어서 하드코딩도 가능하다.
4. 인지/판단/제어를 어떻게 분배할지, 웹을 어떻게 해야 할지 고민해야 한다.
5. 웹을 만들되, 자율주행에 큰 비중을 두고 웹은 뒷순위로 하자
6. 신호를 인식하는 것이 어려울 수 있다. RGB 센서가 얼마나 잘 인식하는지 확인해야 한다.
7. 사람이 하는 실수를 자율주행으로 줄일 수 있거나 사람의 편의를 증가시키는 방향으로 주제를 잡아야 할 것
```

### 2024.02.27 (화)

📌 컨설팅

요약

```
1. 정해진 루트를 가더라도 중간에 오브젝트를 배치해서 경로를 변경한다는 등의 내용이 보여야 한다
2. 자율주행을 배포한다고 하면 대두된 주제 중 학원버스가 제일 수요가 있을 것 같다
3. 학원버스를 주제로 하면 위험하고 조심해야 하는 것에 대한 해결책을 제시하는 방향으로 진행, 어린이보호구역 등 특별한 알고리즘 구현을 추가하기
4. 역할분배는 명세서의 스켈레톤 코드를 학습하고 나누는 것이 좋을 것 같다
5. 자율주행팀은 시뮬레이터만 보여주는 경우도 많다. 웹의 비중은 10% 이하로 줄여도 무방
6. 주제를 정하면 해당 차량이 현재 어떤 역할을 하는지를 알아보고 그것에 맞추어 알고리즘을 짜야 한다
```

컨설팅 후 팀 회의

```
1. 더미 데이터로 학원버스가 서는 곳을 찍어두고 하면 시간 단축할 수 있을 것
2. 명세서를 다 같이 학습하고 역할을 정하자
3. 금주 주말까지 명세서를 각자 학습하고 다음 주에 더 깊은 대화를 나누는 것이 좋을 것 같다
4. 인지(도로인지, 물체인지)/판단/제어 등 어떤 파트를 하고 싶은지 본인 스스로 적당히 생각해보기
```

### 2024.02.28 (수)

프로젝트 소개

- 기획 의도
    - 자율 주행 기술을 활용해 학원 차량의 안전 및 편리성 증대
- 메인 기능
    - 경로 최적화
    - 회피 주행
    - 안전 주행 모드
    - 웹 사이트 차량 실시간 위치 제공
- 세부 사항
    - 경로 최적화
        - 실시간 교통 데이터 정보를 반영한 경로 탐색
        - 탑승 학생에 따른 경로 탐색
    - 회피 주행
        - 돌발 장애물 및 고정 장애물 인식 및 회피
    - 안전 주행 모드
        - 시속에 따른 안전거리 유지
        - 저속 운행 및 어린이 보호구역 등 특정 위치별 운행 속도 제한
    - 웹 사이트 차량 실시간 위치 제공
        - 실시간 GPS 정보를 반영한 웹 사이트 차량 위치 확인 가능

---

📖 자율 주행 사전 학습

### 3강

```
<launch>
  <node pkg="beginner_tutorials" type="talker.py" name="talker" output="screen"/>

  <node pkg="beginner_tutorials" type="listener.py" name="listener" output="screen"/>
</launch>
```

### 4강

```
rostopic pub /ctrl_cmd morai_msgs/CtrlCmd "{longlCmdType: 1, accel: 0.6, brake: 0.0, steering: 0.1, velocity: 0.0, acceleration: 0.0}"
```

### 5강

```
#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from morai_msgs.msg import CtrlCmd, CollisionData, EgoVehicleStatus, EventInfo
from morai_msgs.srv import MoraiEventCmdSrv
from enum import Enum

class Gear(Enum):
    P = 1
    R = 2
    N = 3
    D = 4

class s_drive():
    def __init__(self):
        rospy.init_node('collision_avoid', anonymous=True)
        
        # publisher
        self.cmd_pub = rospy.Publisher('/ctrl_cmd', CtrlCmd, queue_size=1)
        
        # subscriber
        rospy.Subscriber('/CollisionData', CollisionData, self.collision_callback)
        rospy.Subscriber('/Ego_topic', EgoVehicleStatus, self.ego_callback)

        # service
        rospy.wait_for_service('/Service_MoraiEventCmd')
        self.event_cmd_srv = rospy.ServiceProxy('Service_MoraiEventCmd', MoraiEventCmdSrv)

        self.rate = rospy.Rate(10)

        self.is_collision = False
        self.ego_status = EgoVehicleStatus()

        # 처음에 auto_mode , drive gear로 세팅
        self.send_gear_cmd(Gear.D.value)

        while not rospy.is_shutdown():
            if self.is_collision:
                # 후진
                self.send_gear_cmd(Gear.R.value)
                self.send_ctrl_cmd(0, 10.0) 
                rospy.sleep(4)
                
                # 방향 꺾기		
                self.send_ctrl_cmd(1.0, 10.0)  
                rospy.sleep(1) 
                
                # 전진
                self.send_gear_cmd(Gear.D.value)
                self.send_ctrl_cmd(0, 10.0) 
                rospy.sleep(4.2)

                # 방향 원래대로
                self.send_ctrl_cmd(0.9, 10.0)
                rospy.sleep(1)

            else:
                # 충돌이 감지되지 않으면 차량 전진
                self.send_ctrl_cmd(0, 20.0)
                self.rate.sleep()

    # 충돌 메시지 콜백 함수
    def collision_callback(self, data):
        if(len(data.collision_object) > 0):
            self.is_collision = True
        else:
            self.is_collision = False

    # EGO 차량 상태 정보 콜백 함수
    def ego_callback(self, data):
        #print(self.ego_status.velocity.x)
        self.ego_status = data

    # 기어 변경 이벤트 메시지 세팅 함수
    def send_gear_cmd(self, gear_mode):
        # 기어 변경이 제대로 되기 위해서는 차량 속도가 약 0 이어야함
        while( abs(self.ego_status.velocity.x) > 0.1):
            self.send_ctrl_cmd(0,0)
            self.rate.sleep()
        
        gear_cmd = EventInfo()
        gear_cmd.option = 3
        gear_cmd.ctrl_mode = 3
        gear_cmd.gear = gear_mode
        gear_cmd_resp = self.event_cmd_srv(gear_cmd)
        rospy.loginfo(gear_cmd)

    # ctrl_cmd 메시지 세팅 함수
    def send_ctrl_cmd(self, steering ,velocity):
        cmd = CtrlCmd()
        if(velocity > 0):
            cmd.longlCmdType = 2
            cmd.velocity = velocity
            cmd.steering = steering
        else:
            cmd.longlCmdType = 1
            cmd.brake = 1
            cmd.steering = 0
        self.cmd_pub.publish(cmd)


if __name__ == '__main__':
    try:
        s_d = s_drive()
    except rospy.ROSInterruptException:
        pass
```
