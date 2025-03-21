#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
import os, rospkg

from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridgeError

# image parser binarization Node 는 시뮬레이터에서 송신하는 Camera 센서 정보를 받아 실시간으로 출력하는 예제입니다.
# 출력시 hsv 특정 영역의 색상 범위를 지정하여 원하는 색상의 영역만 특정하여 출력합니다.

# 노드 실행 순서 
# 1. HSV 색상 영역 지정
# 2. 특정 영역의 색상 검출
# 3. 이미지 출력

class IMGParser:
    def __init__(self):

        self.image_sub = rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.callback)

    def callback(self, msg):
        try:
            '''
            np_arr = np.fromstring(             )
            img_bgr = cv2.imdecode(             )

            '''
            np_arr = np.frombuffer(msg.data, np.uint8)
            img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
        except CvBridgeError as e:
            print(e)
        '''
        img_hsv = cv2.cvtColor(                 )

        '''
        img_hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
        #TODO: (1)
        '''
        # 특정 색상 영역을 검출하기 위해 범위를 지정합니다.
        # 하한 값 행렬과 상한 값 행렬을 정해 그 사이의 값 만을 출력 하도록 합니다.
        # 이번 예제에서는 노란색 영역을 검출합니다.

        색상(Hue): 약 20~40
        채도(Saturation): 일반적으로 100 이상
        명도(Value): 일반적으로 100 이상

        lower_ylane = np.array([    ,       ,       ])
        upper_ylane = np.array([    ,       ,       ])

        '''

        lower_ylane = np.array([10, 100, 100])
        upper_ylane = np.array([40, 255, 255])

        
        #TODO: (2)
        '''
        # cv2.inRange 함수는 특정 색상 영역을 추출할 수 있습니다. 
        # cv2.inRange 함수를 이용하여 HSV 이미지에서 색상 범위를 지정합니다.
        # 함수의 첫번째 변수에는 이미지 정보를 두번째는 하한 값 세번째는 상한 값 행렬식을 넣습니다.

        img_ylane = cv2.inRange(                    )

        img_ylane = cv2.cvtColor(                   )
        
        img_concat = np.concatenate(                )


        '''
        # cv2.inRange 함수를 사용하여 HSV 이미지에서 흰색 영역을 추출합니다.
        img_wlane = cv2.inRange(img_hsv, lower_ylane, upper_ylane)
        # 검출된 영역을 다시 BGR 형식으로 변환합니다.
        img_wlane = cv2.cvtColor(img_wlane, cv2.COLOR_GRAY2BGR)
        # 원본 이미지와 검출된 영역을 가로 방향으로 이어붙입니다.
        img_concat = np.concatenate((img_bgr, img_wlane), axis=1)

        #TODO: (3)
        '''
        # 이미지를 출력 합니다.

        cv2.imshow(         )
        cv2.waitKey(        ) 

        '''
        cv2.imshow("Image window", img_concat)
        cv2.waitKey(1)


if __name__ == '__main__':

    rospy.init_node('image_parser', anonymous=True)

    image_parser = IMGParser()

    rospy.spin() 