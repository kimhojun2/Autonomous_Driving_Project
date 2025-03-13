#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridgeError

class IMGParser:
    def __init__(self):
        self.image_sub = rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.callback)
        self.img_bgr = None
        self.source_prop = np.float32([
            [0.2, 0.6],  # 이미지의 왼쪽 상단 지점
            [0.8, 0.6],  # 이미지의 오른쪽 상단 지점
            [1.0, 1.0],  # 이미지의 오른쪽 하단 지점
            [0.0, 1.0]   # 이미지의 왼쪽 하단 지점
        ])

    def callback(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            self.img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except CvBridgeError as e:
            print(e)

        img_warp = self.warp_image(self.img_bgr, self.source_prop)

        img_concat = np.concatenate([self.img_bgr, img_warp], axis=1)

        cv2.namedWindow("Image window", cv2.WINDOW_NORMAL)
        cv2.imshow("Image window", img_concat)
        
        cv2.waitKey(1)

        # 마우스 클릭 이벤트를 처리하는 함수를 등록합니다.
        cv2.setMouseCallback("Image window", self.mouse_callback)

    def warp_image(self, img, source_prop):
        image_size = (img.shape[1], img.shape[0])
        x = img.shape[1]
        y = img.shape[0]

        destination_points = np.float32([[0, 0], [x, 0], [x, y], [0, y]])
        source_points = source_prop * np.float32([[x, y]] * 4)

        perspective_transform = cv2.getPerspectiveTransform(source_points, destination_points)
        warped_img = cv2.warpPerspective(img, perspective_transform, image_size)

        return warped_img

    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            print(f"Clicked at pixel coordinates: ({x}, {y})")

def main():
    rospy.init_node('lane_birdview', anonymous=True)
    image_parser = IMGParser()
    rospy.spin()

if __name__ == '__main__':
    main()
