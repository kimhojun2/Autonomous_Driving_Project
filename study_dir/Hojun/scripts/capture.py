#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Image, CompressedImage
import cv2
import numpy as np

class ImageCapture:
    def __init__(self):
        rospy.init_node('image_capture', anonymous=True)
        self.image_sub = rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.image_callback)
        self.save_path = "/home/morai/images/"  # 이미지를 저장할 경로
        self.capture_rate = rospy.Rate(3)  # 1초에 한 번씩 캡처

    def image_callback(self, data):
        np_arr = np.fromstring(data.data, np.uint8)
        img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        # 이미지 저장
        img_name = f"{self.save_path}captured_image_{rospy.get_rostime().secs}.jpg"
        cv2.imwrite(img_name, img_bgr)


    def run(self):
        while not rospy.is_shutdown():
            self.capture_rate.sleep()

if __name__ == '__main__':
    image_capture = ImageCapture()
    image_capture.run()
