#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError

class YOLODetector:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.callback)
        self.rate = rospy.Rate(60)
        self.yellow_lower = np.array([20, 100, 100])  # 노란색 범위
        self.yellow_upper = np.array([30, 255, 255])
        self.pink_lower = np.array([140, 100, 100])  # 분홍색 범위
        self.pink_upper = np.array([170, 255, 255])
        self.purple_lower = np.array([125, 50, 50])  # 보라색 범위
        self.purple_upper = np.array([175, 255, 255])

    def callback(self, msg):
        self.rate.sleep()
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            # HSV 색상 공간으로 변환
            hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)

            # 각 색상에 대한 마스크 생성
            yellow_mask = cv2.inRange(hsv, self.yellow_lower, self.yellow_upper)
            pink_mask = cv2.inRange(hsv, self.pink_lower, self.pink_upper)
            purple_mask = cv2.inRange(hsv, self.purple_lower, self.purple_upper)

            # 각 마스크 별로 물체 검출 및 박스 그리기
            masks = [yellow_mask, pink_mask, purple_mask]
            colors = [(0, 255, 255), (255, 192, 203), (255, 0, 255)]  # 노란색, 분홍색, 보라색
            for mask, color in zip(masks, colors):
                res = cv2.bitwise_and(img_bgr, img_bgr, mask=mask)
                contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                for cnt in contours:
                    area = cv2.contourArea(cnt)
                    if area > 500:
                        x, y, w, h = cv2.boundingRect(cnt)
                        cv2.rectangle(img_bgr, (x, y), (x + w, y + h), color, 2)

            cv2.imshow("YOLOv3 Detection", img_bgr)
            cv2.waitKey(1)

        except CvBridgeError as e:
            print(e)

if __name__ == '__main__':
    rospy.init_node('yolo_detector', anonymous=True)
    yolo_detector = YOLODetector()
    rospy.spin()
