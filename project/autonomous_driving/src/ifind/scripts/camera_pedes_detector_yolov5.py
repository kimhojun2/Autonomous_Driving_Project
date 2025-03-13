#!/usr/bin/env python
# -*- coding: utf-8 -*-

import torch
import numpy as np
import cv2
import rospy
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridgeError

# CUDA를 사용하지 않도록 설정
# torch.cuda.is_available = lambda: False

# YOLOv5 모델 로드
model = torch.hub.load('ultralytics/yolov5', 'yolov5s', force_reload=True)

# COCO dataset의 라벨 매핑
label_map = {
    0: 'person',
    1: 'bicycle',
    2: 'car',
    3: 'motorcycle',
    4: 'airplane',
    5: 'bus',
    6: 'train',
    7: 'truck',
    8: 'boat',
    9: 'traffic light',
    10: 'fire hydrant',
    # 필요에 따라 다른 라벨을 추가할 수 있습니다
}

class YOLODetector:
    def __init__(self):
        self.image_sub = rospy.Subscriber(
            "/image_jpeg/compressed",
            CompressedImage,
            self.callback)
        self.rate = rospy.Rate(30)

    def callback(self, msg):
        self.rate.sleep()
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            # YOLOv5 모델을 사용하여 객체 탐지 수행
            results = model(img_bgr)

            # 결과를 시각화하고 화면에 표시
            img_with_boxes = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
            for box, label_idx in zip(results.xyxy[0], results.names[0]):
                x1, y1, x2, y2, conf, cls = box
                label = label_map[int(cls)]  # 라벨 번호를 실제 라벨 이름으로 변환
                label_text = f'{label} {conf:.2f}'  # 라벨과 확률을 표시하는 텍스트 생성
                cv2.rectangle(img_with_boxes, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                cv2.putText(img_with_boxes, label_text, (int(x1), int(y1) - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            cv2.imshow("YOLOv5 Detection", img_with_boxes)
            cv2.waitKey(1)
        except CvBridgeError as e:
            print(e)

if __name__ == '__main__':
    rospy.init_node('yolo_detector', anonymous=True)
    yolo_detector = YOLODetector()
    rospy.spin()
