#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
测试发布器 - 用于测试 Web UI
发布模拟的点云数据和无人机位姿
"""

import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
import struct

class TestPublisher:
    def __init__(self):
        rospy.init_node('test_publisher', anonymous=True)

        # 发布器
        self.pointcloud_pub = rospy.Publisher('/cloud_in', PointCloud2, queue_size=10)
        self.pose_pub = rospy.Publisher('/mavros/local_position/pose', PoseStamped, queue_size=10)

        # 参数
        self.rate = rospy.Rate(1)  # 1 Hz
        self.drone_height = 2.0
        self.time = 0.0

        rospy.loginfo("测试发布器已启动")

    def create_test_pointcloud(self):
        """创建测试点云 - 一个简单的平面"""
        points = []

        # 创建一个 4x4 米的平面，带有一些起伏
        for x in np.linspace(-2, 2, 50):
            for y in np.linspace(-2, 2, 50):
                z = 0.2 * np.sin(x) * np.cos(y)  # 波浪形表面

                # RGB 颜色（根据高度）
                r = int((z + 0.5) * 255)
                g = int((1 - abs(z)) * 255)
                b = 150
                rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, 255))[0]

                points.append([x, y, z, rgb])

        # 创建 PointCloud2 消息
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'map'

        fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            PointField('rgb', 12, PointField.UINT32, 1),
        ]

        # 打包点云数据
        cloud_data = []
        for point in points:
            cloud_data.append(struct.pack('fffI', point[0], point[1], point[2], int(point[3])))

        pointcloud = PointCloud2()
        pointcloud.header = header
        pointcloud.height = 1
        pointcloud.width = len(points)
        pointcloud.fields = fields
        pointcloud.is_bigendian = False
        pointcloud.point_step = 16
        pointcloud.row_step = pointcloud.point_step * pointcloud.width
        pointcloud.is_dense = True
        pointcloud.data = b''.join(cloud_data)

        return pointcloud

    def create_test_pose(self):
        """创建测试位姿 - 无人机在圆形轨迹上移动"""
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = 'map'

        # 圆形轨迹
        radius = 3.0
        pose_msg.pose.position.x = radius * np.cos(self.time)
        pose_msg.pose.position.y = radius * np.sin(self.time)
        pose_msg.pose.position.z = self.drone_height

        # 朝向运动方向
        yaw = self.time + np.pi / 2
        pose_msg.pose.orientation.x = 0.0
        pose_msg.pose.orientation.y = 0.0
        pose_msg.pose.orientation.z = np.sin(yaw / 2)
        pose_msg.pose.orientation.w = np.cos(yaw / 2)

        return pose_msg

    def run(self):
        """主循环"""
        rospy.loginfo("开始发布测试数据...")

        while not rospy.is_shutdown():
            # 发布点云
            pointcloud = self.create_test_pointcloud()
            self.pointcloud_pub.publish(pointcloud)

            # 发布位姿
            pose = self.create_test_pose()
            self.pose_pub.publish(pose)

            rospy.loginfo("已发布点云 (%d 点) 和位姿 (%.2f, %.2f, %.2f)",
                         pointcloud.width,
                         pose.pose.position.x,
                         pose.pose.position.y,
                         pose.pose.position.z)

            self.time += 0.1
            self.rate.sleep()

if __name__ == '__main__':
    try:
        publisher = TestPublisher()
        publisher.run()
    except rospy.ROSInterruptException:
        pass
