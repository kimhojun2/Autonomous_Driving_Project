#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage, PointCloud2
from geometry_msgs.msg import PoseArray,Pose, Point32
import sensor_msgs.point_cloud2 as pc2
from cv_bridge import CvBridge, CvBridgeError
import math
from numpy.linalg import inv

class LiDARToCameraTransform:
    def __init__(self, params_cam, params_lidar):
        self.bridge = CvBridge()
        # self.scan_sub = rospy.Subscriber("/velodyne_points", PointCloud2, self.scan_callback)
        self.scan_sub = rospy.Subscriber("/clusters", PoseArray, self.scan_callback)
        '''
        header:
            seq: 23
            stamp:
                secs: 1712029773
                nsecs: 485558201
            frame_id: "velodyne"
            poses:
            -
                position:
                x: 26.038862228393555
                y: 3.9749996662139893
                z: -0.4597836434841156
                orientation:
                x: 0.0
                y: 0.0
                z: 0.0
                w: 0.0
            -
                position:
                x: 15.259191513061523
                y: -2.740582227706909
                z: -1.3564049005508423
                orientation:
                x: 0.0
                y: 0.0
                z: 0.0
                w: 0.0
            -
                position:
                x: 15.243913650512695
                y: -2.7984230518341064
                z: -0.812264621257782
                orientation:
                x: 0.0
                y: 0.0
                z: 0.0
                w: 0.0
        '''
        self.image_sub = rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.img_callback)
        self.pc_np = None
        self.img = None
        self.width = params_cam["WIDTH"]
        self.height = params_cam["HEIGHT"]
        self.TransformMat = self.getTransformMat(params_cam, params_lidar)
        self.CameraMat = self.getCameraMat(params_cam)

    def img_callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        self.img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    def scan_callback(self, msg):
        point_list = []
        for pose in msg.poses:
            point_list.append((pose.position.x, pose.position.y, pose.position.z, 1))
        self.pc_np = np.array(point_list, np.float32)

    def getRotMat(self, RPY):
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

    def getSensorToVehicleMat(self, sensorRPY, sensorPosition):
        sensorRotationMat = self.getRotMat(sensorRPY)
        sensorTranslationMat = np.eye(4)
        sensorTranslationMat[:3, 3] = sensorPosition
        Tr_sensor_to_vehicle = np.eye(4)

        Tr_sensor_to_vehicle[:3, :3] = sensorRotationMat
        Tr_sensor_to_vehicle[:3, 3] = sensorPosition
        
        return Tr_sensor_to_vehicle

    def getLiDARTOCameraTransformMat(self, camRPY, camPosition, lidarRPY, lidarPosition):
        Tr_lidar_to_vehicle = self.getSensorToVehicleMat(lidarRPY, lidarPosition)
        Tr_cam_to_vehicle = self.getSensorToVehicleMat(camRPY, camPosition)
        Tr_vehicle_to_cam = inv(Tr_cam_to_vehicle)
        Tr_lidar_to_cam = Tr_vehicle_to_cam.dot(Tr_lidar_to_vehicle)
        
        return Tr_lidar_to_cam

    def getTransformMat(self, params_cam, params_lidar):
        lidarPositionOffset = np.array([0, 0, -0.25]) 
        camPositionOffset = np.array([0.1085, 0, 0])  

        camPosition = np.array([params_cam.get(i) for i in (["X","Y","Z"])]) + camPositionOffset    
        camRPY = np.array([params_cam.get(i) for i in (["ROLL","PITCH","YAW"])]) + np.array([-90*math.pi/180,0,-90*math.pi/180])
        lidarPosition = np.array([params_lidar.get(i) for i in (["X","Y","Z"])]) + lidarPositionOffset
        lidarRPY = np.array([params_lidar.get(i) for i in (["ROLL","PITCH","YAW"])])    
        Tr_lidar_to_cam = self.getLiDARTOCameraTransformMat(camRPY, camPosition, lidarRPY, lidarPosition)
        
        return Tr_lidar_to_cam

    def getCameraMat(self, params_cam):
        focalLength = (params_cam["WIDTH"] / 2) / math.tan(math.radians(params_cam["FOV"] / 2))
        principalX = params_cam["WIDTH"] / 2
        principalY = params_cam["HEIGHT"] / 2
        CameraMat = np.array([[focalLength, 0, principalX],
                              [0, focalLength, principalY],
                              [0, 0, 1]])
        
        return CameraMat
    
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
        point_np = cv2.circle(point_np, ctr, 2, (0,255,0),-1)
    return point_np

class YOLODetector:
    def __init__(self, Transformer):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.callback)
        self.rate = rospy.Rate(60)
        self.Transformer = Transformer
        self.yellow_lower = np.array([25, 100, 100])  # 노란색에 더 가까운 색조 범위와 높은 채도 및 명도
        self.yellow_upper = np.array([40, 255, 255])  # 노란색에 더 가까운 색조 범위와 높은 채도 및 명도
        self.pink_lower = np.array([140, 100, 100])
        self.pink_upper = np.array([170, 255, 255])
        self.purple_lower = np.array([125, 100, 80])  # Lower end of the purple spectrum
        self.purple_upper = np.array([150, 255, 255]) # Upper end of the purple spectrum

        self.pose_array_pub = rospy.Publisher("/lidar_points_pose_array", PoseArray, queue_size=10)

    def callback(self, msg):
        self.rate.sleep()
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
            purple_mask = cv2.inRange(hsv, self.purple_lower, self.purple_upper)
            masks = [purple_mask]
            colors = [(255, 0, 255)]  

            for mask, color in zip(masks, colors):
                res = cv2.bitwise_and(img_bgr, img_bgr, mask=mask)
                contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                for cnt in contours:
                    area = cv2.contourArea(cnt)
                    if area > 500:
                        x, y, w, h = cv2.boundingRect(cnt)
                        cv2.rectangle(img_bgr, (x, y), (x + w, y + h), color, 2)

                        # 바운딩 박스 내의 라이다 포인트를 출력
                        # xyz_p = self.Transformer.pc_np[:, 0:3]
                        xyz_p = self.Transformer.pc_np.reshape(-1, 4)[:, :3]
                        # xyz_p = np.insert(xyz_p,3,1,axis=1).T
                        # xyz_p = np.delete(xyz_p, np.where(xyz_p[0,:] < 0), axis=1)
                        # xyz_p = np.delete(xyz_p, np.where(xyz_p[0, :] > 100), axis=1)
                        # xyz_p = np.delete(xyz_p, np.where(xyz_p[2,:] < -1.2), axis=1) 
                        # xyz_p = np.delete(xyz_p, np.where(xyz_p[2,:] > 0), axis=1)
                        pose_array = PoseArray()
                        for point in xyz_p.T:
                            if len(point) >= 3:
                                pose = Pose()
                                pose.position.x = point[0]
                                pose.position.y = point[1]
                                pose.position.z = point[2]
                                pose_array.poses.append(pose)

                        self.pose_array_pub.publish(pose_array)
                        print("LiDAR Points within bounding box:")
                        print(xyz_p[0:3])
            

            cv2.imshow("YOLOv3 Detection", img_bgr)
            cv2.waitKey(1)

        except CvBridgeError as e:
            print(e)

if __name__ == '__main__':
    rospy.init_node('yolo_detector', anonymous=True)
    parameters_cam = {
        "WIDTH": 640,
        "HEIGHT": 480,
        "FOV": 30,
        "X": 1.79,
        "Y": -0.05,
        "Z": 1.18,
        "YAW": 0.00,
        "PITCH": 0.00,
        "ROLL": 0.00 
    }

    parameters_lidar = {
        "X": 0.20,
        "Y": 0.00,
        "Z": 1.61,
        "YAW": 0.00,
        "PITCH": 0.00,
        "ROLL": 0.00
    }

    Transformer = LiDARToCameraTransform(parameters_cam, parameters_lidar)
    yolo_detector = YOLODetector(Transformer)
    rospy.spin()
