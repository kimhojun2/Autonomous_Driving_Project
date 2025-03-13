#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np

from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridgeError

# YOLO 가중치 파일과 CFG 파일 로드
net = cv2.dnn.readNet("/home/morai/catkin_ws/src/ssafy_3/scripts/yolov2-tiny.cfg", "/home/morai/catkin_ws/src/ssafy_3/scripts/yolov2-tiny.weights")
classes = []
with open("/home/morai/catkin_ws/src/ssafy_3/scripts/coco.names", "r") as f:
    classes = [line.strip() for line in f.readlines()]
layer_names = net.getLayerNames()
output_layers = [layer_names[i - 1] for i in net.getUnconnectedOutLayers()]

def yolov3_detection(img):
    height, width, channels = img.shape
    blob = cv2.dnn.blobFromImage(img, 0.00392, (416, 416), (0, 0, 0), True, crop=False)
    net.setInput(blob)
    outs = net.forward(output_layers)

    class_ids = []
    confidences = []
    boxes = []

    for out in outs:
        for detection in out:
            scores = detection[5:]
            class_id = np.argmax(scores)
            confidence = scores[class_id]
            if confidence > 0.5:
                center_x = int(detection[0] * width)
                center_y = int(detection[1] * height)
                w = int(detection[2] * width)
                h = int(detection[3] * height)
                x = int(center_x - w / 2)
                y = int(center_y - h / 2)
                boxes.append([x, y, w, h])
                confidences.append(float(confidence))
                class_ids.append(class_id)

    indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)

    for i in range(len(boxes)):
        if i in indexes:
            x, y, w, h = boxes[i]
            label = str(classes[class_ids[i]])
            confidence = confidences[i]
            if classes[class_ids[i]] == 'traffic light':
                # cv2.rectangle(img, (x, y), (x + w, y + h), (0, 0, 255), 2)
                # cv2.putText(img, label + ' ' + str(round(confidence, 2)), (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

                roi = img[y:y+h, x:x+w]
                hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
                lower_red = np.array([0, 50, 50])
                upper_red = np.array([10, 255, 255])
                lower_green = np.array([50, 50, 50])
                upper_green = np.array([70, 255, 255])
                lower_yellow = np.array([20, 100, 100])
                upper_yellow = np.array([30, 255, 255])
                mask_red = cv2.inRange(hsv, lower_red, upper_red)
                mask_green = cv2.inRange(hsv, lower_green, upper_green)
                mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)

                # Traffic light 색상에 따라 다른 색상의 박스와 레이블을 그림
                if np.sum(mask_red) > np.sum(mask_green) and np.sum(mask_red) > np.sum(mask_yellow):
                    cv2.rectangle(img, (x, y), (x + w, y + h), (0, 0, 255), 2)  # 빨간색 박스
                    label = 'red light'
                    cv2.putText(img, label + ' ' + str(round(confidence, 2)), (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                elif np.sum(mask_green) > np.sum(mask_red) and np.sum(mask_green) > np.sum(mask_yellow):
                    cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)  # 녹색 박스
                    label = 'green light'
                    cv2.putText(img, label + ' ' + str(round(confidence, 2)), (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                elif np.sum(mask_yellow) > np.sum(mask_red) and np.sum(mask_yellow) > np.sum(mask_green):
                    cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 255), 2)  # 노란색 박스
                    label = 'yellow light'
                    cv2.putText(img, label + ' ' + str(round(confidence, 2)), (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
                
            else:
                cv2.rectangle(img, (x, y), (x + w, y + h), (0, 0, 0), 2)
                cv2.putText(img, label + ' ' + str(round(confidence, 2)), (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
            # cv2.putText(img, label + ' ' + str(round(confidence, 2)), (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    return img

class YOLODetector:
    def __init__(self):
        self.image_sub = rospy.Subscriber(
            "/image_jpeg/compressed",
            CompressedImage,
            self.callback)
        self.rate = rospy.Rate(20)

    def callback(self, msg):
        self.rate.sleep()
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            img_resized = cv2.resize(img_bgr, (800, 800))
            img_with_boxes = yolov3_detection(img_resized)
            cv2.imshow("YOLOv3 Detection", img_with_boxes)
            cv2.waitKey(1)
        except CvBridgeError as e:
            print(e)

if __name__ == '__main__':
    rospy.init_node('yolo_detector', anonymous=True)
    yolo_detector = YOLODetector()
    rospy.spin()
