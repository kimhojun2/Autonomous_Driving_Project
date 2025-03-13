#!/usr/bin/env python
#-*- coding:utf-8 -*-

import rospy
import cv2
import numpy as np
import math
import time
from sensor_msgs.msg import PointCloud2, CompressedImage
import sensor_msgs.point_cloud2 as pc2
from numpy.linalg import inv
from cv_bridge import CvBridgeError



net = cv2.dnn.readNet("/home/morai/catkin_ws/src/ssafy_3/scripts/yolov2-tiny.cfg", "/home/morai/catkin_ws/src/ssafy_3/scripts/yolov2-tiny.weights")
classes = []
with open("/home/morai/catkin_ws/src/ssafy_3/scripts/coco.names", "r") as f:
    classes = [line.strip() for line in f.readlines()]
layer_names = net.getLayerNames()
output_layers = [layer_names[i - 1] for i in net.getUnconnectedOutLayers()]

parameters_cam = {
    "WIDTH": 640, # image width
    "HEIGHT": 480, # image height
    "FOV": 30, # Field of view
    "X": 1.79, # meter
    "Y": -0.05,
    "Z": 1.18,
    "YAW": 0.00, # radian    
    "PITCH": 0.00,
    "ROLL": 0.00 
}

parameters_lidar = {
    "X": 0.20, # meter
    "Y": 0.00,
    "Z": 1.61,
    "YAW": 0.00, # radian
    "PITCH": 0.00,
    "ROLL": 0.00
}


def getRotMat(RPY):        
    
    cosR = math.cos(RPY[0])
    cosP = math.cos(RPY[1])
    cosY = math.cos(RPY[2])
    sinR = math.sin(RPY[0])
    sinP = math.sin(RPY[1])
    sinY = math.sin(RPY[2])
    
    rotRoll = np.array([[1, 0, 0], [0, cosR, -sinR], [0, sinR, cosR]])
    rotPitch = np.array([[cosP, 0, sinP], [0, 1, 0], [-sinP, 0, cosP]])
    rotYaw = np.array([[cosY, -sinY, 0], [sinY, cosY, 0], [0, 0, 1]])
    
    rotMat = rotYaw.dot(rotPitch.dot(rotRoll))    
    return rotMat
    

def getSensorToVehicleMat(sensorRPY, sensorPosition):

    sensorRotationMat = getRotMat(sensorRPY)
    sensorTranslationMat = np.eye(4)
    sensorTranslationMat[:3, 3] = sensorPosition
    Tr_sensor_to_vehicle = np.eye(4)

    Tr_sensor_to_vehicle[:3, :3] = sensorRotationMat
    Tr_sensor_to_vehicle[:3, 3] = sensorPosition
    
    return Tr_sensor_to_vehicle
    
    
def getLiDARTOCameraTransformMat(camRPY, camPosition, lidarRPY, lidarPosition):

    Tr_lidar_to_vehicle = getSensorToVehicleMat(lidarRPY, lidarPosition)
    Tr_cam_to_vehicle = getSensorToVehicleMat(camRPY, camPosition)
    Tr_vehicle_to_cam = inv(Tr_cam_to_vehicle)
    Tr_lidar_to_cam = Tr_vehicle_to_cam.dot(Tr_lidar_to_vehicle)
    
    print(Tr_lidar_to_cam)
    return Tr_lidar_to_cam
    

def getTransformMat(params_cam, params_lidar):
    #With Respect to Vehicle ISO Coordinate    
    lidarPositionOffset = np.array([0, 0, -0.25]) # VLP16 사용해야 함
    camPositionOffset = np.array([0.1085, 0, 0])  # Camera Offset  

    camPosition = np.array([params_cam.get(i) for i in (["X","Y","Z"])]) + camPositionOffset    
    camRPY = np.array([params_cam.get(i) for i in (["ROLL","PITCH","YAW"])]) + np.array([-90*math.pi/180,0,-90*math.pi/180])
    lidarPosition = np.array([params_lidar.get(i) for i in (["X","Y","Z"])]) + lidarPositionOffset
    lidarRPY = np.array([params_lidar.get(i) for i in (["ROLL","PITCH","YAW"])])    
    Tr_lidar_to_cam = getLiDARTOCameraTransformMat(camRPY, camPosition, lidarRPY, lidarPosition)
    return Tr_lidar_to_cam if Tr_lidar_to_cam is not None else np.eye(4)


def getCameraMat(params_cam):
    focalLength = (params_cam["WIDTH"] / 2) / math.tan(math.radians(params_cam["FOV"] / 2))
    principalX = params_cam["WIDTH"] / 2
    principalY = params_cam["HEIGHT"] / 2
    CameraMat = np.array([[focalLength, 0, principalX],
                          [0, focalLength, principalY],
                          [0, 0, 1]])
    
    # print(CameraMat)
    return CameraMat


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
            
            elif classes[class_ids[i]] == 'person':
                
                if w <=5 or h<=9:
                    continue
                cv2.rectangle(img, (x, y), (x + w, y + h), (0, 0, 0), 2)
                cv2.putText(img, label + ' ' + str(round(confidence, 2)), (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
                print('Person', w, h, confidence)

            elif classes[class_ids[i]] == 'car':
                print('Car', w, h)
                if w <=30 and h<=30:
                    continue
                cv2.rectangle(img, (x, y), (x + w, y + h), (100, 20, 0), 2)
                cv2.putText(img, label + ' ' + str(round(confidence, 2)), (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (100, 20, 0), 2)

    return img

    
class LiDARToCameraTransform:
    def __init__(self, params_cam, params_lidar):
        self.scan_sub = rospy.Subscriber("/velodyne_points", PointCloud2, self.scan_callback)
        self.image_sub = rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.img_callback)
        self.pc_np = None
        self.img = None
        self.width = params_cam["WIDTH"]
        self.height = params_cam["HEIGHT"]
        self.TransformMat = getTransformMat(params_cam, params_lidar)
        self.CameraMat = getCameraMat(params_cam)

    #TODO : (4) LiDAR의 PointCloud2, Camera의 Image data 수신
    def img_callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        self.img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    def scan_callback(self, msg):
        point_list = []
        for point in pc2.read_points(msg, skip_nans=True):
            point_list.append((point[0], point[1], point[2], 1))
        self.pc_np = np.array(point_list, np.float32)

    def transformLiDARToCamera(self, pc_lidar):
        pc_wrt_cam = self.TransformMat.dot(pc_lidar)
        pc_wrt_cam = np.delete(pc_wrt_cam, 3, axis=0)
        return pc_wrt_cam

    def transformCameraToImage(self, pc_camera):
        pc_proj_to_img = self.CameraMat.dot(pc_camera)
        pc_proj_to_img = np.delete(pc_proj_to_img,np.where(pc_proj_to_img[2,:]<0),axis=1)
        pc_proj_to_img /= pc_proj_to_img[2,:]
        pc_proj_to_img = np.delete(pc_proj_to_img,np.where(pc_proj_to_img[0,:]>self.width),axis=1)
        pc_proj_to_img = np.delete(pc_proj_to_img,np.where(pc_proj_to_img[1,:]>self.height),axis=1)
        return pc_proj_to_img

def draw_pts_img(img, xi, yi):
    point_np = img

    for ctr in zip(xi, yi):
        point_np = cv2.circle(point_np, ctr, 2, (0,0,0),-1)
    return point_np

if __name__ == '__main__':
    rospy.init_node('ex_calib', anonymous=True)
    Transformer = LiDARToCameraTransform(parameters_cam, parameters_lidar)
    time.sleep(1)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        xyz_p = Transformer.pc_np[:, 0:3]
        xyz_p = np.insert(xyz_p,3,1,axis=1).T
        xyz_p = np.delete(xyz_p,np.where(xyz_p[0,:]<0),axis=1)
        xyz_p = np.delete(xyz_p, np.where(xyz_p[0, :] > 100), axis=1)
        #xyz_p = np.delete(xyz_p,np.where(xyz_p[0,:]>10),axis=1)
        xyz_p = np.delete(xyz_p,np.where(xyz_p[2,:]<-1.2),axis=1) #Ground Filter
        #print(xyz_p[0])

        xyz_c = Transformer.transformLiDARToCamera(xyz_p)
        #print(np.size(xyz_c[0]))

        xy_i = Transformer.transformCameraToImage(xyz_c)
        #print(np.size(xy_i[0]))

        #TODO: (6) PointCloud가 Image에 투영된 Processed Image 시각화
        xy_i = xy_i.astype(np.int32)
        projectionImage = draw_pts_img(Transformer.img, xy_i[0,:], xy_i[1,:])
        result = yolov3_detection(projectionImage)    
        cv2.imshow("LidartoCameraProjection", projectionImage)
        cv2.waitKey(1)