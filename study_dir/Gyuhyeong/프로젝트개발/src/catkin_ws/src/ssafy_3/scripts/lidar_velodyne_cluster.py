#!/usr/bin/env python
#-*- coding:utf-8 -*-

import rospy
import cv2
import numpy as np
# import ros_numpy
import pcl
import pcl_helper

import ctypes
import os

from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import PoseArray,Pose
from sklearn.cluster import DBSCAN

# lidar_velodyne_cluster 는 LiDAR의 Point들을 물체 단위로 구분하는 Clustering 예제입니다.
# PointCloud Data를 입력받아 DBSCAN Algorithm을 활용하여 Clustering을 수행합니다.
# 교육생분들은 DBSCAN의 Parameter를 조절하여 적절한 Clustering 결과를 얻어내야 합니다.

# 노드 실행 순서
# 1. DBSCAN Parameter 입력
# 2. 각 Cluster를 대표하는 위치 값 계산
# 3. PointCloud Data로부터 Distance, Angle 값 계산

class SCANCluster:
    def __init__(self):

        self.scan_sub = rospy.Subscriber("/velodyne_points", PointCloud2, self.callback)

        self.cluster_pub = rospy.Publisher("clusters", PoseArray, queue_size=10)

        self.scan_sub_roi = rospy.Publisher("velodyne_points_roi", PointCloud2, queue_size=10)


        self.pc_np = None

        #TODO: (1) DBSCAN Parameter 입력
        
        # DBSCAN의 Parameter를 결정하는 영역입니다.
        # sklearn.cluster의 DBSCAN에 대해 조사하여 적절한 Parameter를 입력하기 바랍니다.

        self.dbscan = DBSCAN(eps=0.3, min_samples=10)
        


    def callback(self, msg):

        cloud = pcl_helper.ros_to_pcl(msg)

        # Voxel 다운 샘플링
        LEAF_SIZE = 0.1
        cloud = self.do_voxel_grid_downssampling(cloud,LEAF_SIZE)
        
        # 관심 영역 지정
        filtered_pcl = self.do_passthrough(cloud, 'y', -10, 10)
        filtered_pcl = self.do_passthrough(filtered_pcl, 'x', -20, 100)

        # 바닥 제거
        _, _, filtered_pcl = self.do_ransac_plane_normal_segmentation(filtered_pcl, 0.3)

        filtered_cloud = pcl_helper.pcl_to_ros(filtered_pcl)

        self.scan_sub_roi.publish(filtered_cloud)

        # PointCloud2 메시지를 numpy 배열로 변환
        self.pc_np = self.pointcloud2_to_xyz(filtered_cloud)
        
        # PoseArray 메시지 초기화
        cluster_msg = PoseArray()
        # 현재 시간과 프레임 ID 설정
        cluster_msg.header.stamp = rospy.Time.now()
        cluster_msg.header.frame_id = "velodyne"  # 또는 LiDAR 센서의 프레임 ID

        if len(self.pc_np) > 0:
            pc_xy = self.pc_np[:, :2]  # X, Y 좌표 추출

            # DBSCAN 클러스터링 수행
            db = self.dbscan.fit_predict(pc_xy)

            # 클러스터링된 레이블 중 최대값 찾기 (레이블 -1은 노이즈를 의미)
            n_cluster = max(db) + 1 if np.max(db) >= 0 else 0

            for cluster in range(n_cluster):
                # 현재 클러스터에 속하는 포인트들의 인덱스 추출
                idx = np.where(db == cluster)[0]
                if len(idx) > 0:
                    # 클러스터 포인트들의 중심점 계산
                    cluster_points = pc_xy[idx]
                    centroid = np.mean(cluster_points, axis=0)
                    # Pose 메시지 생성 및 중심점 설정
                    tmp_pose = Pose()
                    tmp_pose.position.x = centroid[0]
                    tmp_pose.position.y = centroid[1]
                    tmp_pose.position.z = 0  # Z 좌표는 0으로 설정 (2D 클러스터링 가정)
                    # 계산된 중심점을 PoseArray 메시지에 추가
                    cluster_msg.poses.append(tmp_pose)

        # 클러스터링 결과 발행
        self.cluster_pub.publish(cluster_msg)

    def pointcloud2_to_xyz(self, cloud_msg):

        point_list = []
        
        for point in pc2.read_points(cloud_msg, skip_nans=True):
            #TODO: (3) PointCloud Data로부터 Distance, Angle 값 계산
            
            # LiDAR의 PointCloud Data로부터 Distance와 Angle 값을 계산하는 영역입니다.
            # 각 Point의 XYZ 값을 활용하여 Distance와 Yaw Angle을 계산합니다.
            # Input : point (X, Y, Z, Intensity)            
            
            dist = np.sqrt(point[0]**2 + point[1]**2 + point[2]**2)
            angle = np.arctan2(point[1], point[0])
            
            if point[0] > 0 and 1.50 > point[2] > -1.25 and dist < 50:
                point_list.append((point[0], point[1], point[2], point[3], dist, angle))

        point_np = np.array(point_list, np.float32)

        return point_np

    def do_passthrough(self, pcl_data, filter_axis, axis_min, axis_max):
        '''
        Create a PassThrough  object and assigns a filter axis and range.
        :param pcl_data: point could data subscriber
        :param filter_axis: filter axis
        :param axis_min: Minimum  axis to the passthrough filter object
        :param axis_max: Maximum axis to the passthrough filter object
        :return: passthrough on point cloud
        '''
        passthrough = pcl_data.make_passthrough_filter()
        passthrough.set_filter_field_name(filter_axis)
        passthrough.set_filter_limits(axis_min, axis_max)
        return passthrough.filter()
    
    # Use RANSAC planse segmentation to separate plane and not plane points
    # Returns inliers (plane) and outliers (not plane)
    def do_ransac_plane_normal_segmentation(self, point_cloud, input_max_distance):
        segmenter = point_cloud.make_segmenter_normals(ksearch=50)
        segmenter.set_optimize_coefficients(True)
        segmenter.set_model_type(pcl.SACMODEL_NORMAL_PLANE)  #pcl_sac_model_plane
        segmenter.set_normal_distance_weight(0.1)
        segmenter.set_method_type(pcl.SAC_RANSAC) #pcl_sac_ransac
        segmenter.set_max_iterations(1000)
        segmenter.set_distance_threshold(input_max_distance) #0.03)  #max_distance
        indices, coefficients = segmenter.segment()

        inliers = point_cloud.extract(indices, negative=False)
        outliers = point_cloud.extract(indices, negative=True)

        return indices, inliers, outliers

    def do_voxel_grid_downssampling(self, pcl_data,leaf_size):
        '''
        Create a VoxelGrid filter object for a input point cloud
        :param pcl_data: point cloud data subscriber
        :param leaf_size: voxel(or leaf) size
        :return: Voxel grid downsampling on point cloud
        :https://github.com/fouliex/RoboticPerception
        '''
        vox = pcl_data.make_voxel_grid_filter()
        vox.set_leaf_size(leaf_size, leaf_size, leaf_size) # The bigger the leaf size the less information retained
        return  vox.filter()

if __name__ == '__main__':

    rospy.init_node('velodyne_clustering', anonymous=True)

    scan_cluster = SCANCluster()

    rospy.spin() 
