#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from nav_msgs.msg import Path
import json

class PathSaver:
    def __init__(self):
        self.path_sub = rospy.Subscriber('/global_path', Path, self.path_callback)
        self.saved_path = []

    def path_callback(self, msg):
        # 경로 데이터를 저장하는 로직
        for pose_stamped in msg.poses:
            x = pose_stamped.pose.position.x
            y = pose_stamped.pose.position.y
            z = pose_stamped.pose.position.z
            self.saved_path.append([x, y, z])

    def save_path_to_file(self, file_name):
        # 경로 데이터를 파일로 저장하는 로직
        with open(file_name, 'w') as file:
            json.dump(self.saved_path, file)
        rospy.loginfo("Path saved to %s", file_name)

if __name__ == '__main__':
    rospy.init_node('path_saver_node')
    path_saver = PathSaver()
    rospy.spin()
    # 종료 시점에 파일로 저장
    path_saver.save_path_to_file('saved_path.json')