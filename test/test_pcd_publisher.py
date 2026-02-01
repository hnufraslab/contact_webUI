#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
PCD文件点云发布器（ROS 1版本）
读取test.pcd文件并发布到ROS，包含大幅降采样以提高性能
"""

import rospy
import struct
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
import sys
import os

class PCDPublisher:
    def __init__(self):
        rospy.init_node('pcd_publisher', anonymous=True)

        # 发布器
        self.cloud_pub = rospy.Publisher('/cloud_in', PointCloud2, queue_size=1)
        self.pose_pub = rospy.Publisher('/mavros/local_position/pose', PoseStamped, queue_size=1)

        # 参数
        self.target_points = 100000  # 目标点数（从1289万降采样到10万）
        self.publish_rate = 5  # 发布频率 Hz

        # 加载PCD文件
        self.pcd_file = os.path.join(os.path.dirname(__file__), 'test.pcd')
        print(f"正在加载PCD文件: {self.pcd_file}")

        self.points_xyz = None
        self.points_intensity = None
        self.load_pcd()

        print(f"PCD文件加载完成！")
        print(f"原始点数: {len(self.points_xyz)}")
        print(f"降采样后点数: {self.target_points}")
        print(f"降采样比例: 1:{len(self.points_xyz)//self.target_points}")

    def load_pcd(self):
        """加载并降采样PCD文件"""
        try:
            with open(self.pcd_file, 'rb') as f:
                # 读取头部
                header_lines = []
                while True:
                    line = f.readline().decode('ascii', errors='ignore').strip()
                    header_lines.append(line)
                    if line.startswith('DATA'):
                        break

                # 解析头部信息
                points_count = 0
                fields = []
                for line in header_lines:
                    if line.startswith('POINTS'):
                        points_count = int(line.split()[1])
                    elif line.startswith('FIELDS'):
                        fields = line.split()[1:]

                print(f"PCD文件包含 {points_count} 个点")
                print(f"字段: {fields}")

                # 读取二进制数据
                # 每个点: x,y,z,intensity,normal_x,normal_y,normal_z,curvature (8个float32)
                point_size = 32  # 8 * 4 bytes

                # 计算降采样步长
                downsample_step = max(1, points_count // self.target_points)

                points_xyz = []
                points_intensity = []

                print(f"开始读取和降采样数据...")
                for i in range(points_count):
                    data = f.read(point_size)
                    if len(data) < point_size:
                        break

                    # 只保留每隔downsample_step的点
                    if i % downsample_step == 0:
                        # 解析8个float32
                        values = struct.unpack('ffffffff', data)
                        x, y, z, intensity = values[0], values[1], values[2], values[3]

                        # 过滤无效点
                        if not (np.isnan(x) or np.isnan(y) or np.isnan(z)):
                            points_xyz.append([x, y, z])
                            points_intensity.append(intensity if not np.isnan(intensity) else 0.0)

                    # 显示进度
                    if i % 1000000 == 0 and i > 0:
                        print(f"  已处理 {i}/{points_count} 点 ({100*i//points_count}%)")

                self.points_xyz = np.array(points_xyz, dtype=np.float32)
                self.points_intensity = np.array(points_intensity, dtype=np.float32)

                print(f"数据加载完成！实际点数: {len(self.points_xyz)}")

                # 打印点云范围
                if len(self.points_xyz) > 0:
                    print(f"点云范围:")
                    print(f"  X: [{self.points_xyz[:,0].min():.2f}, {self.points_xyz[:,0].max():.2f}]")
                    print(f"  Y: [{self.points_xyz[:,1].min():.2f}, {self.points_xyz[:,1].max():.2f}]")
                    print(f"  Z: [{self.points_xyz[:,2].min():.2f}, {self.points_xyz[:,2].max():.2f}]")
                    print(f"  强度: [{self.points_intensity.min():.2f}, {self.points_intensity.max():.2f}]")

        except Exception as e:
            print(f"加载PCD文件失败: {e}")
            sys.exit(1)

    def create_pointcloud2(self):
        """创建PointCloud2消息"""
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "map"

        # 定义字段
        fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            PointField('intensity', 12, PointField.FLOAT32, 1),
        ]

        # 构建点云数据
        points = []
        for i in range(len(self.points_xyz)):
            x, y, z = self.points_xyz[i]
            intensity = self.points_intensity[i]
            points.append(struct.pack('ffff', x, y, z, intensity))

        cloud_data = b''.join(points)

        # 创建PointCloud2消息
        cloud_msg = PointCloud2()
        cloud_msg.header = header
        cloud_msg.height = 1
        cloud_msg.width = len(self.points_xyz)
        cloud_msg.fields = fields
        cloud_msg.is_bigendian = False
        cloud_msg.point_step = 16  # 4 fields * 4 bytes
        cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width
        cloud_msg.data = cloud_data
        cloud_msg.is_dense = True

        return cloud_msg

    def create_pose(self, t):
        """创建模拟的无人机位姿"""
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = "map"

        # 计算点云中心作为无人机位置参考
        if len(self.points_xyz) > 0:
            center_x = (self.points_xyz[:,0].min() + self.points_xyz[:,0].max()) / 2
            center_y = (self.points_xyz[:,1].min() + self.points_xyz[:,1].max()) / 2
            center_z = self.points_xyz[:,2].max() + 5.0  # 在点云上方5米

            # 无人机在点云上方做圆周运动
            radius = 10.0
            pose_msg.pose.position.x = center_x + radius * np.cos(t)
            pose_msg.pose.position.y = center_y + radius * np.sin(t)
            pose_msg.pose.position.z = center_z
        else:
            pose_msg.pose.position.x = 0.0
            pose_msg.pose.position.y = 0.0
            pose_msg.pose.position.z = 5.0

        # 朝向圆心
        pose_msg.pose.orientation.x = 0.0
        pose_msg.pose.orientation.y = 0.0
        pose_msg.pose.orientation.z = np.sin(t/2 + np.pi/2)
        pose_msg.pose.orientation.w = np.cos(t/2 + np.pi/2)

        return pose_msg

    def run(self):
        """主循环"""
        if self.points_xyz is None or len(self.points_xyz) == 0:
            print("错误: 没有有效的点云数据")
            return

        # 预先创建点云消息（因为点云是静态的）
        cloud_msg = self.create_pointcloud2()

        rate = rospy.Rate(self.publish_rate)
        t = 0.0

        print(f"\n开始发布点云数据...")
        print(f"点云Topic: /cloud_in")
        print(f"位姿Topic: /mavros/local_position/pose")
        print(f"发布频率: {self.publish_rate} Hz")
        print(f"按 Ctrl+C 停止\n")

        while not rospy.is_shutdown():
            # 更新时间戳
            cloud_msg.header.stamp = rospy.Time.now()

            # 发布点云
            self.cloud_pub.publish(cloud_msg)

            # 发布位姿
            pose_msg = self.create_pose(t)
            self.pose_pub.publish(pose_msg)

            t += 0.1
            rate.sleep()

if __name__ == '__main__':
    try:
        publisher = PCDPublisher()
        publisher.run()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        print("\n程序已停止")
