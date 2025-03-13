#!/usr/bin/env python
# -*- coding: utf-8 -*-
 
import rospy
import cv2
import numpy as np
import os, rospkg
import json

from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridgeError

# image_lane_bev 은 perspective view 형태의 이미지를 BEV(bird-eye-view) 형태로 전환하는 예제입니다.
# IPM (Inverse Perspective Mapping)
# Warping

# 노드 실행 순서 
# 1. 이미지 warping 영역 지정
# 2. 원본 이미지 내 Warping 영역과 매칭 될 목적지 지점 정의
# 3. 원근 맵 행렬 생성 함수를 이용하여 매핑 좌표에 대한 원근 맵 행렬을 생성
# 4. 원근 맵 행렬에 대한 기하학적 변환

def warp_image(img, source_prop):
    
    image_size = (img.shape[1], img.shape[0])
    # print(f'높이:{image_size[0]}, 너비: {image_size[1]}')
    # x = 너비
    # y = 높이
    x = img.shape[1]
    y = img.shape[0]
    
    #TODO: (2) 원본 이미지 내 Warping 영역과 매칭 될 목적지 지점 정의
    '''
    destination_points = np.float32(
    
    source_points = source_prop * np.float32([[x, y]]* 4)
    '''
     # (1) 이미지 전체를 warping 영역으로 설정
    destination_points = np.float32([[0, 0], [x, 0], [x, y], [0, y]])
     # (2) 원본 이미지 내 Warping 영역과 매칭 될 목적지 지점 정의
    source_points = source_prop * np.float32([[x, y]] * 4)
    # 이미지를 기하학적 변환을 해야합니다. 이미지를 인위적으로 확대, 축소, 위치 변경, 회전,
    # 왜곡을 하는 것으로 BEV는 원근이 있는 이미지를 원근을 없애는 작업을 해야합니다.
    # 기하학적 변환은 아핀변환과(Affine transformation) 원근변환(Perspective transformation)이 있습니다.
    # 우리는 원근 변환이라는 OpenCV를 사용합니다.

    #TODO: (3) 원근 맵 행렬 생성 함수를 이용하여 매핑 좌표에 대한 원근 맵 행렬을 생성합니다.
    '''
    perspective_transform = cv2.

    '''
    perspective_transform = cv2.getPerspectiveTransform(source_points, destination_points)
    #TODO: (4) 이후 원근 맵 행렬에 대한 기하학적 변환을 진행합니다.
    '''
    warped_img = cv2.

    '''
    warped_img = cv2.warpPerspective(img, perspective_transform, image_size)
    
    return warped_img


class IMGParser:
    def __init__(self):

        self.image_sub = rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.callback)
        self.img_bgr = None

        #TODO: (1) 이미지 warping 영역 지정
        '''
        Bird's eye view 를 하기 위한 영역을 지정해야 합니다. 이미지 warping을 위해 영역을 비율로 만들어줘야 합니다.
        self.source_prop = np.float32(
        '''
        # 이미지 warping 영역을 비율로 정의합니다.
        self.source_prop = np.float32([
            [0.2, 0.6],  # 이미지의 왼쪽 상단 지점
            [0.8, 0.6],  # 이미지의 오른쪽 상단 지점
            [1.0, 1.0],   # 이미지의 오른쪽 하단 지점
            [0.0, 1.0]    # 이미지의 왼쪽 하단 지점
        ])
        
    def callback(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            self.img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except CvBridgeError as e:
            print(e)

        #TODO: (2-0) 이미지 Warping 시작
        img_warp = warp_image(self.img_bgr, self.source_prop)

        img_concat = np.concatenate([self.img_bgr, img_warp], axis=1)

        cv2.imshow("Image window", img_concat)
        cv2.waitKey(1) 


def main():

    rospy.init_node('lane_birdview', anonymous=True)

    image_parser = IMGParser()

    rospy.spin()

if __name__ == '__main__':
    
    main()
