#!/usr/bin/env python
#-*- coding:utf-8 -*-

import rospy
import cv2
import numpy as np
import os, rospkg
import json
import math
import random
from std_msgs.msg import Bool
import tf
from scipy.interpolate import interp1d

from cv_bridge import CvBridgeError
from sklearn import linear_model

from nav_msgs.msg import Odometry,Path
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import PoseStamped,Point
from morai_msgs.msg import CtrlCmd, EgoVehicleStatus


# ignore warning
from warnings import simplefilter
from sklearn.exceptions import ConvergenceWarning
simplefilter("ignore", category=ConvergenceWarning)

class IMGParser:
    def __init__(self, pkg_name = 'ssafy_3'):

        self.image_sub = rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.callback)
        # self.odom_sub = rospy.Subscriber("odom", Odometry, self.odom_callback)
        self.zone_pub = rospy.Publisher("/child_zone_detected", Bool, queue_size=1) # 새로운 퍼블리셔
        self.zone_detected = False # 감지 상태를 저장하는 플래그

        self.img_bgr = None
        # self.edges = None 
        self.is_status = False

        self.lower_child_zone = np.array([0, 100, 50])
        self.upper_child_zone = np.array([10, 255, 255])

        x = 640
        y = 480
        
        self.crop_pts = np.array([
            [x*0.3, y*0.3],     # 좌상단 점
            [x*0.7, y*0.3],   # 우상단 점
            [x*1.32, y], # 우하단 점
            [-x*0.32, y]    # 좌하단 점
        ])

        self.source_prop = np.float32([
            [0.39, 0.3],    # 좌상단 비율
            [0.61, 0.3],    # 우상단 비율
            [0.8, 0.7],     # 우하단 비율 (가장자리를 넘어가지 않도록 1.0으로 조정)
            [0.2, 0.7]      # 좌하단 비율 (가장자리를 넘어가지 않도록 0.0으로 조정)
        ])
                
        if np.sum(self.lower_child_zone) == 0 or np.sum(self.upper_child_zone) == 0 or \
        np.sum(self.crop_pts) == 0:
            print("you need to find the right value : check lines at 33 ~ 39")
            exit()  

        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            # if self.img_bgr is not None and self.is_status == True:
            #     img_warp = self.warp_image(self.img_bgr, self.source_prop)  
            #     img_lane = self.binarize(img_warp)

            #     # cv2.imshow("img_lane", img_lane)
            #     # cv2.imshow("img_warp", img_warp)
            #     # cv2.imshow("origin_img", self.img_bgr)
            #     # cv2.waitKey(1)

           
            rate.sleep()

    def odom_callback(self,msg): ## Vehicl Status Subscriber 
        self.status_msg=msg    
        self.is_status = True

    def process_image(self):
        if self.img_bgr is not None:
            img_warp = self.warp_image(self.img_bgr, self.source_prop)
            img_lane = self.binarize(img_warp)
            self.zone_detected = np.any(img_lane) # 영역 감지 여부 확인
            self.zone_pub.publish(self.zone_detected)
             # 감지 상태 발행

            cv2.imshow("img_lane", img_lane)
            cv2.imshow("img_warp", img_warp)
            cv2.imshow("origin_img", self.img_bgr)
            cv2.waitKey(1)
    def callback(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            self.img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            self.process_image()
        except CvBridgeError as e:
            print(e)

    def binarize(self, img):
        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        self.img_lane = cv2.inRange(img_hsv, self.lower_child_zone, self.upper_child_zone)


        return self.img_lane
    
    def warp_image(self, img, source_prop):
        
        image_size = (img.shape[1], img.shape[0])

        x = img.shape[1] # 이미지의 너비
        y = img.shape[0] # 이미지의 높이
        
        destination_points = np.float32([
        [0, 0],  # 좌하단
        [x, 0],  # 좌상단
        [x, y],  # 우상단
        [0, y]   # 우하단
        ])

        source_points = source_prop * np.float32([[x, y]]* 4)
        
        perspective_transform = cv2.getPerspectiveTransform(source_points, destination_points)
        
        warped_img = cv2.warpPerspective(img, perspective_transform, image_size, flags=cv2.INTER_LINEAR)
        
        return warped_img


class BEVTransform:
    def __init__(self, params_cam, xb=10.0, zb=10.0):
        self.xb = xb
        self.zb = zb

        self.theta = np.deg2rad(params_cam["PITCH"])
        self.width = params_cam["WIDTH"]
        self.height = params_cam["HEIGHT"]
        self.x = params_cam["X"]

        if params_cam["ENGINE"]=="UNITY":
            self.alpha_r = np.deg2rad(params_cam["FOV"]/2)

            self.fc_y = params_cam["HEIGHT"]/(2*np.tan(np.deg2rad(params_cam["FOV"]/2)))
            self.alpha_c = np.arctan2(params_cam["WIDTH"]/2, self.fc_y)

            self.fc_x = self.fc_y

        else:
            self.alpha_c = np.deg2rad(params_cam["FOV"]/2)

            self.fc_x = params_cam["WIDTH"]/(2*np.tan(np.deg2rad(params_cam["FOV"]/2)))
            self.alpha_r = np.arctan2(params_cam["HEIGHT"]/2, self.fc_x)

            self.fc_y = self.fc_x
            
        self.h = params_cam["Z"] + 0.34

        self.n = float(params_cam["WIDTH"])
        self.m = float(params_cam["HEIGHT"])

        self.RT_b2g = np.matmul(np.matmul(self.traslationMtx(xb, 0, zb), self.rotationMtx(np.deg2rad(-90), 0, 0)),
                                self.rotationMtx(0, 0, np.deg2rad(180)))

        self.proj_mtx = self.project2img_mtx(params_cam)

        self._build_tf(params_cam)


    def calc_Xv_Yu(self, U, V):
        Xv = self.h*(np.tan(self.theta)*(1-2*(V-1)/(self.m-1))*np.tan(self.alpha_r)-1)/\
            (-np.tan(self.theta)+(1-2*(V-1)/(self.m-1))*np.tan(self.alpha_r))

        Yu = (1-2*(U-1)/(self.n-1))*Xv*np.tan(self.alpha_c)

        return Xv, Yu


    def _build_tf(self, params_cam):
        v = np.array([params_cam["HEIGHT"]*0.5, params_cam["HEIGHT"]]).astype(np.float32)
        u = np.array([0, params_cam["WIDTH"]]).astype(np.float32)

        U, V = np.meshgrid(u, v)

        Xv, Yu = self.calc_Xv_Yu(U, V)

        xyz_g = np.concatenate([Xv.reshape([1,-1]) + params_cam["X"],
                                Yu.reshape([1,-1]),
                                np.zeros_like(Yu.reshape([1,-1])),
                                np.ones_like(Yu.reshape([1,-1]))], axis=0)
        
        xyz_bird = np.matmul(np.linalg.inv(self.RT_b2g), xyz_g)

        xyi = self.project_pts2img(xyz_bird)

        src_pts = np.concatenate([U.reshape([-1, 1]), V.reshape([-1, 1])], axis=1).astype(np.float32)
        dst_pts = xyi.astype(np.float32)

        self.perspective_tf = cv2.getPerspectiveTransform(src_pts, dst_pts)

        self.perspective_inv_tf = cv2.getPerspectiveTransform(dst_pts, src_pts)


    def warp_bev_img(self, img):
        img_warp = cv2.warpPerspective(img, self.perspective_tf, (self.width, self.height), flags=cv2.INTER_LINEAR)
        
        return img_warp

    
    def warp_inv_img(self, img_warp):    
        img_f = cv2.warpPerspective(img_warp, self.perspective_inv_tf, (self.width, self.height), flags=cv2.INTER_LINEAR)
        
        return img_f


    def recon_lane_pts(self, img):
        if cv2.countNonZero(img) != 0:
    
            UV_mark = cv2.findNonZero(img).reshape([-1,2])

            U, V = UV_mark[:, 0].reshape([-1,1]), UV_mark[:, 1].reshape([-1,1])
            
            Xv, Yu = self.calc_Xv_Yu(U, V)

            xyz_g = np.concatenate([Xv.reshape([1,-1]) + self.x,
                                Yu.reshape([1,-1]),
                                np.zeros_like(Yu.reshape([1,-1])),
                                np.ones_like(Yu.reshape([1,-1]))], axis=0)

            xyz_g = xyz_g[:, xyz_g[0,:]>=0]

        else:
            xyz_g = np.zeros((4, 10))

        return xyz_g


    def project_lane2img(self, x_pred, y_pred_l, y_pred_r):
        xyz_l_g = np.concatenate([x_pred.reshape([1,-1]),
                                  y_pred_l.reshape([1,-1]),
                                  np.zeros_like(y_pred_l.reshape([1,-1])),
                                  np.ones_like(y_pred_l.reshape([1,-1]))
                                  ], axis=0)

        xyz_r_g = np.concatenate([x_pred.reshape([1,-1]),
                                  y_pred_r.reshape([1,-1]),
                                  np.zeros_like(y_pred_r.reshape([1,-1])),
                                  np.ones_like(y_pred_r.reshape([1,-1]))
                                  ], axis=0)

        xyz_l_b = np.matmul(np.linalg.inv(self.RT_b2g), xyz_l_g)
        xyz_r_b = np.matmul(np.linalg.inv(self.RT_b2g), xyz_r_g)

        xyl = self.project_pts2img(xyz_l_b)
        xyr = self.project_pts2img(xyz_r_b)

        xyl = self.crop_pts(xyl)
        xyr = self.crop_pts(xyr)
        
        return xyl, xyr
        

    def project_pts2img(self, xyz_bird):
        xc, yc, zc = xyz_bird[0,:].reshape([1,-1]), xyz_bird[1,:].reshape([1,-1]), xyz_bird[2,:].reshape([1,-1])

        xn, yn = xc/(zc+0.0001), yc/(zc+0.0001)

        xyi = np.matmul(self.proj_mtx, np.concatenate([xn, yn, np.ones_like(xn)], axis=0))

        xyi = xyi[0:2,:].T
        
        return xyi

    def crop_pts(self, xyi):
        xyi = xyi[np.logical_and(xyi[:, 0]>=0, xyi[:, 0]<self.width), :]
        xyi = xyi[np.logical_and(xyi[:, 1]>=0, xyi[:, 1]<self.height), :]

        return xyi


    def traslationMtx(self,x, y, z):     
        M = np.array([[1,         0,              0,               x],
                    [0,         1,              0,               y],
                    [0,         0,              1,               z],
                    [0,         0,              0,               1],
                    ])
        
        return M

    def project2img_mtx(self,params_cam):    

        # focal lengths
        if params_cam["ENGINE"]=='UNITY':
            fc_x = params_cam["HEIGHT"]/(2*np.tan(np.deg2rad(params_cam["FOV"]/2)))
            fc_y = params_cam["HEIGHT"]/(2*np.tan(np.deg2rad(params_cam["FOV"]/2)))

        else:
            fc_x = params_cam["WIDTH"]/(2*np.tan(np.deg2rad(params_cam["FOV"]/2)))
            fc_y = params_cam["WIDTH"]/(2*np.tan(np.deg2rad(params_cam["FOV"]/2)))

        #the center of image
        cx = params_cam["WIDTH"]/2
        cy = params_cam["HEIGHT"]/2
        
        #transformation matrix from 3D to 2D
        R_f = np.array([[fc_x,  0,      cx],
                        [0,     fc_y,   cy]])

        return R_f


    def rotationMtx(self,yaw, pitch, roll):    
        R_x = np.array([[1,         0,              0,                0],
                        [0,         math.cos(roll), -math.sin(roll) , 0],
                        [0,         math.sin(roll), math.cos(roll)  , 0],
                        [0,         0,              0,               1],
                        ])
                        
        R_y = np.array([[math.cos(pitch),    0,      math.sin(pitch) , 0],
                        [0,                  1,      0               , 0],
                        [-math.sin(pitch),   0,      math.cos(pitch) , 0],
                        [0,         0,              0,               1],
                        ])
        
        R_z = np.array([[math.cos(yaw),    -math.sin(yaw),    0,    0],
                        [math.sin(yaw),    math.cos(yaw),     0,    0],
                        [0,                0,                 1,    0],
                        [0,         0,              0,               1],
                        ])
                        
        R = np.matmul(R_x, np.matmul(R_y, R_z))
    
        return R


if __name__ == '__main__':

    rospy.init_node('child_zone_image', anonymous=True)

    image_parser = IMGParser()

    rospy.spin() 
