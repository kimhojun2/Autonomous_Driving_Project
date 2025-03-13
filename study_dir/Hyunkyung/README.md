# 🚗금주 활동 내용

### 2024.02.26(월)
#### 📌주제 회의
##### 주제
```
1. 학원버스
2. 투어버스
3. 통근버스
```

##### 팀원 의견
```
1. 일단 구현을 목표로 하고 나중에 기능을 추가하자
2. 모든 주제가 기능적인 부분에서 큰 차이가 없는데 발표했을 때 사람들의 공감을 이끌어 낼 수 있는 부분을 생각해보자
3. 같은 맵에서 진행한다고 하면 장애물이 다 json형태로 되어있어서 하드코딩도 가능하다
4. 인지/판단/제어를 어떻게 분배할지, 웹을 어떻게 해야할지 고민해야한다
5. 웹을 만들되, 자율주행에 많은 비중을 두고 웹은 후순위로 하자
6. 신호를 인식하는 것이 어려울 수 있다. RGB센서가 얼마나 잘 인식하는지 확인해야 한다.
7. 사람이 하는 실수를 자율주행으로 줄일 수 있거나 사람의 편의를 증가시키는 방향으로 주제를 잡아야할 것
```

#### 📖개인 공부
##### 센서이론
1. Camara
- 사진을 찍는 기기
- 시각적으로 보이는 정보를 찍어 다양한 정보 인식 가능(딥러닝, 표지판, 신호등 등)
- 비교적 낮은 단가
- 단독으로는 정확한 거리 알기 힘듬, 밤이나 악천후 시 기능 저하
- Sensor Model Selection
    - Pinhole camara - 일반적인 카메라
    - Fisheye camara - 물고기처럼 아래에서 위로 올려다보는 카메라, 광각 휘는 현상 있음
- Ground Truth Selection(GT) - 학습시킬 대상 또는 원본 / 이미지 처리 또는 거리 기반 알고리즘
    - RGB - 현실과 동일
    - Semantic - 같은 객체끼리 묶어서 구분, 차량이 주행할 영역을 판단
    - Instance - 객체를 식별하고 바운딩박스, 객체의 위차와 속도의 정보를 가짐
- parameter Settings
    - 카메라 내/외부 정보
    - Communication Interface -UDP/ROS protocol

2. LiDAR
- 레이저 펄스를 쏘아서 반사되어 돌아오는 시간을 이용하여 거리를 인지
- 정밀도가 높음(채널 수에 따라 다름)
- 3D 이미지 제공
- 높은 에너지 소비/가격대/큰 외형/악천후에 약함
- GT
    - Intensity - 반사되어 돌아오는 신호의 강도에 따라 색을 다르게 설정
    - 가우시안?

3. Radar
- 전파를 매게체로 사물간 거리 파악
- 라이다보다 정밀도 떨어짐
- 악천후 이용가능/소형화가능
- 사이즈 = 성능/작은 물체 인지 어려움/정밀도 떨어짐

4. GPS
- 인공위성을 이용해 현재 위치를 알아내는 센서
- 31개의 인공위성으로 구성됨
- 최소 4개 이상 위성과 연결되어 있어야 위치 계산이 가능
- 경로계획을 위해서는 좌표값을 2D로 받아야 함(WGS84 - 타원형/위도,경도로 나옴 → 2D로 바꾸어서 경로 계획)
- 가성비 좋음
- 장애물 영향/노이즈 영향을 많이 받음, 오차가 큼
- 가우시안 노이즈로 구현

5. IMU(Inertia Measurement Unit)
- 관성 측정 장비
- 3축에 대한 각속도를 측정하는 자이로센서
- 3축에 대한 가속도를 측정하는 가속도센서
- 회전 이름 : x축 roll y축 pitch z축 yaw
- 적분에 의해 물체의 회전각과 기울기 측정 → 적분에 의한 오차 발생 → 시간이 지나면 오차 누적으로 정확도 떨어짐
- 노이즈 구현?

##### ROS(Robot Operating System)

- 로봇 소프트웨어를 개발하기 위한 소프트웨어 프레임워크
- 로봇 개발에 필수적인 라이브러리 제공
- Application만 개발하면 되기 때문에 시간 절약 가능
- 노드 간 통신을 기반으로 전체 시스템 구동
    - ROS Master(roscore) - 노드와 노드 사이의 연결과 통신을 위한 서버, 반드시 실행해야 노드간 통신이 가능
    - ROS Node - ROS에서 실행되는 최소 단위 프로세스, 하나의 목적에 하나의 노드
    - ROS Message - 노드와 노드 간의 데이터 주고받는 양식
    - ROS Package - 노드, 라이브러리, 환경 설정 파일 등을 통합하는 최소 빌드 단위
    - ROS Topic - 단방향의 연속적인 송수신 방식
    - ROS Service - 양방향의 일회성 송수신 방식
    - ROS Publish - Topic에 원하는 메세지를 담아 송신하는 것
    - ROS Subscribe - Topic의 내용에 해당하는 Message를 수신하는 것
- src - 실제 코드 작성 파일들(특히 Launch, scripts)
- build - make 파일
- devel - 라이브러리 저장소

##### ❓ 모르는 용어 정리

1. 가우시안 효과
- 흐림 효과를 말함
    - 가우시안 노이즈 : 어떤 주파수 대역 내의 모든 주파수 출력이 포함되어 있는 잡음
2. 노이즈
- 외부 환경에서 수집된 데이터에 영향을 주는 잡음이나 왜곡


### 2024.02.27(화)

#### 📌컨설팅
##### 요약
```
1. 정해진 루트를 가더라도 중간에 오브젝트를 배치해서 경로를 변경한다는 등의 내용이 보여야한다
2. 자율주행을 배포한다고 하면 대두된 주제 중 학원버스가 제일 수요가 있을 것 같다
3. 학원버스를 주제로하면 위험하고 조심해야하는 것에 대한 해결책을 제시하는 방향으로 진행, 어린이보호구역 등 특별한 알고리즘 구현을 추가하기
4. 역할분배는 면세서의 스켈레톤 코드를 학습하고 나누는 것이 좋을 것 같다
5. 자율주행팀은 시뮬레이터만 보여주는 경우도 많다. 웹의 비중은 10%이하로 줄여도 무방
6. 주제를 정하면 해당 챠랑이 현재 어떤 역할을 하는지를 알아보고 그것에 맞추어 알고리즘을 짜야한다
```
##### 컨설팅 후 팀원회의
```
1. 더미데이터로 학원버스가 서는 곳을 찍어두고 하면 시간 단축할 수 있을 것
2. 명세서를 다같이 학습하고 역할을 정하자
3. 금주 주말까지 명세서를 각자 학습하고 다음 주에 더 깊은 대화를 나누는 것이 좋을 것 같다
4. 인지(도로인지, 물체인지)/판단/제어 등 어떤 파트를 하고 싶은 지 본인 스스로 적당히 생각해보기
```



#### 📖개인 공부
##### 좌표계 변환

1. 3차원 좌표를 2차원 좌표로 변환하는 이유
- 자율주행 경로계획 시 2차원 맵 사용
- GPS 경위도 좌표계는 3차원 → UTM 2차원 좌표계로 변환 필요
- 필요한 정보는 어떤 지점이 기준점으로부터 얼마나 멀리 있느냐
- WGS84는 도, 분, 초로 나타내서 거리 계산이 어려움

2. gps.py 코드 분석
    
    ```python
    #!/usr/bin/env python
    # -*- coding: utf-8 -*-
    
    import rospy
    
    from pyproj import Proj
    from std_msgs.msg import Float32MultiArray
    from morai_msgs.msg import GPSMessage
    
    class GPS_to_UTM:
        def __init__(self, output_data_type):
            self.output_data_type = output_data_type
            rospy.init_node('GPS_to_UTM', anonymous=True)
    				#rosmaster가 실행시키기 위한 이름부여
            self.gps_sub = rospy.Subscriber("/gps", GPSMessage, self.gps_callback)
            #Subscriber를 정의, /gps 타입은 morai_msgs, 메세지를 받았을 때 콜백함수 호출
    				self.utm_pub = rospy.Publisher("/utm", Float32MultiArray, queue_size=1)
            #Publisher를 정의, utm topic사용, 메세지타입 
    				self.proj_UTM = Proj(proj='utm', zone=52, ellps = 'WGS84', preserve_units=False)
    				#변환을 위한 타입
    
            self.utm_msg = Float32MultiArray()
    				#메세지에 정보를 넣음
            self.is_gps_data = False
    				#데이터가 들어왔는지 판단하는 것
    
            rate = rospy.Rate(30)
            while not rospy.is_shutdown():
                if self.is_gps_data == True:
                    self.utm_pub.publish(self.utm_msg)
                    print("data published at '/utm' topic !")
                else:
                    print("waiting for data...")
                rate.sleep()
    
    		#subscriber에서 호출하는 콜백함수
        def gps_callback(self, gps_msg):
            self.is_gps_data = True
            longitude = gps_msg.longitude
            latitude = gps_msg.latitude
            utm_xy = self.proj_UTM(longitude, latitude)
            utm_x = utm_xy[0]
            utm_y = utm_xy[1]
    
            if self.output_data_type == "utm_xy":    
                self.utm_msg.data = [utm_x, utm_y]
    				
    				#현실과의 오차 줄이기
            elif self.output_data_type == "simulator_map_xy":
                map_x = utm_x - gps_msg.eastOffset
                map_y = utm_y - gps_msg.northOffset
                self.utm_msg.data = [map_x, map_y]
    
    if __name__ == '__main__':
        try:
            GPS_to_UTM(output_data_type = 'utm_xy')
            # GPS_to_UTM(output_data_type = 'simulator_map_xy')
    
        except rospy.ROSInterruptException:
            pass
    
    ```
    


3. Odometry


- 이동물체의 주행거리 측정
- GPS + IMU(가속도와 각속도)
    - 가속도를 적분하여 사용하므로 시간 경과에 따라 오차가 누적
- GPS의 정확도를 향상시키기 위해 IMU와 결합해서 사용
    - IMU의 3축에 대한 가속도를 통해 연속적인 위치 데이터 획득

- odometry.py 코드 분석
    
    ```python
    #!/usr/bin/env python
    # -*- coding: utf-8 -*-
     
    import rospy
    
    from std_msgs.msg import Float32MultiArray
    from sensor_msgs.msg import Imu
    #imu는 degree각이 아닌 Quaternion각 사용
    from nav_msgs.msg import Odometry
    
    class Odometry_publisher:
        def __init__(self):
            rospy.init_node('Odometry_publisher', anonymous=True)
            self.utm_sub = rospy.Subscriber("/utm", Float32MultiArray, self.utm_callback)
            self.imu_sub = rospy.Subscriber("/imu", Imu, self.imu_callback)
            self.odom_pub = rospy.Publisher('/odom',Odometry, queue_size=1)
            
            self.is_utm=False
            self.is_imu=False
            self.utm_x = None
            self.utm_y = None
            
            self.odom_msg=Odometry()
            self.odom_msg.header.frame_id='/odom'
            self.odom_msg.child_frame_id='/base_link'
    
            rate = rospy.Rate(30)
            while not rospy.is_shutdown():
                if self.is_utm == True and self.is_imu==True:
                    self.odom_pub.publish(self.odom_msg)
                    print("data published at '/odom' topic !")
                else:
                    print("waiting for data...")
                rate.sleep()
    
        def utm_callback(self, utm_msg):
            self.is_utm=True
            self.odom_msg.pose.pose.position.x = utm_msg.data[0]
            self.odom_msg.pose.pose.position.y = utm_msg.data[1]
            self.odom_msg.pose.pose.position.z = 0
    
        def imu_callback(self, imu_msg):
            self.is_imu=True
            self.odom_msg.pose.pose.orientation.x = imu_msg.orientation.x
            self.odom_msg.pose.pose.orientation.y = imu_msg.orientation.y
            self.odom_msg.pose.pose.orientation.z = imu_msg.orientation.z
            self.odom_msg.pose.pose.orientation.w = imu_msg.orientation.w
    
            
    
    if __name__ == '__main__':
        try:
            Odometry_publisher()
        except rospy.ROSInterruptException:
            pass
    
    ```
    

##### ❓ 모르는 용어 정리

1. UTM(Universal Transverse Mercator Coordinate System)

- 지구상의 모든 점을 2차원 평면에 나타냄
- 경도 6도 위도 8도
- 극지방으로 가도 직사각형을 유지하여 거리, 면적, 방향을 나타내기 편함
- 메르카토르 도법
    - 경도와 위도가 직각을 이룸
    - 적도부분은 실제와 거리가 같으나 고위도로 갈 수록 거리가 길어지고 왜곡
    - 왜곡을 줄이려 우리나라 기준 횡축 메르카토르 도법을 사용
    

2. Quaternion(사원수)

- 두 개의 3D 벡터의 몫, 3D 그래픽과 가속도계 기반 센서에서 방향 또는 회전 데이터를 나타내는데 사용


### 2024.02.28(수)

##### 프로젝트 소개
- 기획 의도
    - 자율 주행 기술을 활용해 학원 차량의 안전 및 편리성 증대

- 메인 기능
    - 경로 최적화
    - 회피 주행
    - 안전 주행 모드
    - 웹 사이트 차량 실시간 위치 제공

- 세부 사항
    - 경로 최적화
        - 실시간 교통 데이터 정보를 반영한 경로 탐색
        - 탑승 학생에 따른 경로 탐색
    - 회피 주행
        - 돌발 장애물 및 고정 장애물 인식 및 회피
    - 안전 주행 모드
        - 시속에 따른 안전 거리 유지
        - 저속 운행 및 어린이 보호구역 등 특정 위치 별 운행 속도 제한
    - 웹 사이트 차량 실시간 위치 제공
        - 실시간 GPS 정보를 반영한 웹사이트 차량 위치 확인 가능

#### 📖개인 공부

##### 📌 자율주행 기초


- 다양한 기업들이 현업에서 ROS를 이용하여 자율주행 알고리즘을 개발
- 알고리즘은 인지/판단/제어, Ground truth라고 하는 참 값을 이용하기도 함

##### 📌 ROS(Robot Operating System)
- 로봇 개발을 위한 소프트웨어 프레임워크
- 실행 프로그램을 독립적으로 설계하고 실행 시 프로세스 간 결합도를 낮춤
- 다양한 라이브러리, C++/Python 지원
- 라이다, 카메라 등의 데이터 센서 툴 보유
- [ROS Wiki](https://wiki.ros.org/Documentation) 활용하기

##### 메세지와 토픽

- node간의 데이터를 주고 받음
    - master를 통해 각 노드의 Publisher와 Subscriber의 정보를 공유해서 메세지를 주고 받음
- 메시지 타입을 맞춰줘야 함
- 메시지 발행은 하나의 노드에서, 수시는 여러 개의 노드가 가능

##### rosbridge

- json의 형태로 데이터 통신을 받아 ROS 메시지로 바꾸거나 그 반대의 역할을 함
- Local ip 내부에서의 통신 뿐만 아니라 외부 프로그램과 통신 할 수 있게 함

##### rviz, rqt

- 메시지를 발행하면 해당 시각화 툴이 토픽을 수신하여 데이터 타입에 따라 시각화
- 메시지, 노드, 센서 데이터 등을 확인할 수 있음
    - rqt_plot : 토픽을 이용한 그래프 생성
    - rviz : 라이다, tf, image 등의 메시지를 3D로 시각화
    - rqt_graph : 노드와 메시지 관계 그래프 생성


##### 스켈레톤 프로젝트
- Package_name : 패키지 파일
    - scripts : Python file 보관
    - include : 패키지 헤더파일 보관
    - CMakeList.txt : 패키지 cmake 빌드 설정파일
    - Package.xml : 패키지의 메타 정보를 제공하는 파일
    - Launch : launch 파일 보관
- CMakeList.txt : 워크스페이스에 대한 make 빌드 설정파일

- talker.py
    - 메세지 송신을 위한 기능을 가진 노드
    - ROSpy.Publisher함수로 Publisher 생성
    - 매개변수 : 토픽, 타입, 버퍼 개수
    - Subscriber 또는 rqt 등에서 확인 가능
- listener.py
    - 메세지 수신을 위한 기능을 가진 코드
    - ROSpy.Subscriber함수로 Subscriber 생성
    - 매개변수 : 토픽, 타입
- server.py
    - 서버 통신을 위한 노드
    - 요청에 따른 응답이 사용되는 메시지 통신 방식
- client.py
    - server요청 뒤에 응답을 기다림
- get_*_status.py
    - listener를 이용해 시뮬레이터에서 받아온 정보를 확인 및 출력
    - 각각 시뮬레이터에서 충돌 정보, 차량 상태, Object 상태, 신호등 정보 출력하는 Node 생성
- set_ctrl_cmd.py, set_ego_setting.py
    - talker node를 이용해 Ego차량의 움직임, 위치 제어 메세지를 송신하는 node 생성

##### ❓ 모르는 용어 정리

##### Ground truth(GT)

- AI 모델 출력값을 훈련 및 테스트하는 데 사용되는 실제 환경의 데이터

##### tf(좌표계 변환)

- 로봇에 부착된 다양한 센서들로부터 얻은 값들을 이용하기 위해 좌표계를 변환하는 것