#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PCD文件点云发布器（ROS 2版本）
读取test.pcd文件并发布到ROS 2，支持裁剪功能
"""

import rclpy
from rclpy.node import Node
import struct
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import PoseStamped, Vector3
from std_msgs.msg import Header, Bool
import sys
import os

class PCDPublisher(Node):
    def __init__(self):
        super().__init__('pcd_publisher')

        # 发布器
        self.cloud_pub = self.create_publisher(PointCloud2, '/cloud_in', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/mavros/local_position/pose', 10)

        # 订阅裁剪框
        self.crop_box_sub = self.create_subscription(
            PoseStamped, '/planning/crop_box', self.crop_box_callback, 10)
        self.crop_size_sub = self.create_subscription(
            Vector3, '/planning/crop_box_size', self.crop_size_callback, 10)
        self.crop_reset_sub = self.create_subscription(
            Bool, '/planning/crop_reset', self.crop_reset_callback, 10)

        # 参数
        self.target_points = 10000  # 目标点数
        self.publish_rate = 2  # 发布频率 Hz

        # 裁剪状态
        self.crop_enabled = False
        self.crop_center = None
        self.crop_size = None
        self.crop_orientation = None

        # 加载PCD文件
        self.pcd_file = os.path.join(os.path.dirname(__file__), 'test.pcd')
        self.get_logger().info(f"正在加载PCD文件: {self.pcd_file}")

        self.points_xyz = None
        self.points_intensity = None
        self.load_pcd()

        self.get_logger().info(f"PCD文件加载完成！")
        self.get_logger().info(f"原始点数: {len(self.points_xyz)}")

        # 创建定时器
        self.t = 0.0
        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)

        # 预先创建点云消息
        self.full_cloud_msg = self.create_pointcloud2(self.points_xyz, self.points_intensity)
        self.cropped_cloud_msg = None

        self.get_logger().info("等待裁剪框命令...")
        self.get_logger().info("  订阅: /planning/crop_box, /planning/crop_box_size, /planning/crop_reset")

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
                for line in header_lines:
                    if line.startswith('POINTS'):
                        points_count = int(line.split()[1])

                self.get_logger().info(f"PCD文件包含 {points_count} 个点")

                # 读取二进制数据
                point_size = 32  # 8 * 4 bytes
                downsample_step = max(1, points_count // self.target_points)

                points_xyz = []
                points_intensity = []

                self.get_logger().info(f"开始读取和降采样数据...")
                for i in range(points_count):
                    data = f.read(point_size)
                    if len(data) < point_size:
                        break

                    if i % downsample_step == 0:
                        values = struct.unpack('ffffffff', data)
                        x, y, z, intensity = values[0], values[1], values[2], values[3]

                        if not (np.isnan(x) or np.isnan(y) or np.isnan(z)):
                            points_xyz.append([x, y, z])
                            points_intensity.append(intensity if not np.isnan(intensity) else 0.0)

                    if i % 1000000 == 0 and i > 0:
                        self.get_logger().info(f"  已处理 {i}/{points_count} 点")

                self.points_xyz = np.array(points_xyz, dtype=np.float32)
                self.points_intensity = np.array(points_intensity, dtype=np.float32)

                self.get_logger().info(f"数据加载完成！实际点数: {len(self.points_xyz)}")

                if len(self.points_xyz) > 0:
                    self.get_logger().info(f"点云范围:")
                    self.get_logger().info(f"  X: [{self.points_xyz[:,0].min():.2f}, {self.points_xyz[:,0].max():.2f}]")
                    self.get_logger().info(f"  Y: [{self.points_xyz[:,1].min():.2f}, {self.points_xyz[:,1].max():.2f}]")
                    self.get_logger().info(f"  Z: [{self.points_xyz[:,2].min():.2f}, {self.points_xyz[:,2].max():.2f}]")

        except Exception as e:
            self.get_logger().error(f"加载PCD文件失败: {e}")
            sys.exit(1)

    def crop_box_callback(self, msg):
        """接收裁剪框位姿"""
        self.crop_center = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        ])
        self.crop_orientation = np.array([
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w
        ])
        self.get_logger().info(f"收到裁剪框位姿: 中心({self.crop_center[0]:.2f}, {self.crop_center[1]:.2f}, {self.crop_center[2]:.2f})")
        self.try_apply_crop()

    def crop_size_callback(self, msg):
        """接收裁剪框尺寸"""
        self.crop_size = np.array([msg.x, msg.y, msg.z])
        self.get_logger().info(f"收到裁剪框尺寸: ({self.crop_size[0]:.2f}, {self.crop_size[1]:.2f}, {self.crop_size[2]:.2f})")
        self.try_apply_crop()

    def crop_reset_callback(self, msg):
        """接收重置裁剪命令"""
        if msg.data:
            self.get_logger().info("收到重置裁剪命令")
            self.crop_enabled = False
            self.crop_center = None
            self.crop_size = None
            self.crop_orientation = None
            self.cropped_cloud_msg = None
            self.get_logger().info("已重置为完整点云")

    def try_apply_crop(self):
        """尝试应用裁剪"""
        if self.crop_center is not None and self.crop_size is not None:
            self.apply_crop()

    def apply_crop(self):
        """应用裁剪框，生成裁剪后的点云"""
        self.get_logger().info("正在应用裁剪...")

        # 获取裁剪框参数（注意坐标系转换：Web端已转换为Three.js坐标系）
        # Three.js: X-右, Y-上, Z-前
        # ROS: X-前, Y-左, Z-上
        # 需要反向转换: ROS.x = Three.x, ROS.y = -Three.z, ROS.z = Three.y
        center_ros = np.array([
            self.crop_center[0],
            -self.crop_center[2],
            self.crop_center[1]
        ])

        size_ros = np.array([
            self.crop_size[0],
            self.crop_size[2],
            self.crop_size[1]
        ])

        half_size = size_ros / 2.0

        # 简单的轴对齐包围盒裁剪（忽略旋转）
        min_bound = center_ros - half_size
        max_bound = center_ros + half_size

        self.get_logger().info(f"裁剪范围 (ROS坐标系):")
        self.get_logger().info(f"  X: [{min_bound[0]:.2f}, {max_bound[0]:.2f}]")
        self.get_logger().info(f"  Y: [{min_bound[1]:.2f}, {max_bound[1]:.2f}]")
        self.get_logger().info(f"  Z: [{min_bound[2]:.2f}, {max_bound[2]:.2f}]")

        # 筛选在裁剪框内的点
        mask = (
            (self.points_xyz[:, 0] >= min_bound[0]) & (self.points_xyz[:, 0] <= max_bound[0]) &
            (self.points_xyz[:, 1] >= min_bound[1]) & (self.points_xyz[:, 1] <= max_bound[1]) &
            (self.points_xyz[:, 2] >= min_bound[2]) & (self.points_xyz[:, 2] <= max_bound[2])
        )

        cropped_xyz = self.points_xyz[mask]
        cropped_intensity = self.points_intensity[mask]

        self.get_logger().info(f"裁剪结果: {len(cropped_xyz)} / {len(self.points_xyz)} 点")

        if len(cropped_xyz) > 0:
            self.cropped_cloud_msg = self.create_pointcloud2(cropped_xyz, cropped_intensity)
            self.crop_enabled = True
            self.get_logger().info("裁剪完成，开始发布裁剪后的点云")
        else:
            self.get_logger().warn("裁剪后没有点，保持发布完整点云")
            self.crop_enabled = False

    def create_pointcloud2(self, points_xyz, points_intensity):
        """创建PointCloud2消息"""
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "map"

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        points = []
        for i in range(len(points_xyz)):
            x, y, z = points_xyz[i]
            intensity = points_intensity[i]
            points.append(struct.pack('ffff', x, y, z, intensity))

        cloud_data = b''.join(points)

        cloud_msg = PointCloud2()
        cloud_msg.header = header
        cloud_msg.height = 1
        cloud_msg.width = len(points_xyz)
        cloud_msg.fields = fields
        cloud_msg.is_bigendian = False
        cloud_msg.point_step = 16
        cloud_msg.row_step = 16 * len(points_xyz)
        cloud_msg.data = cloud_data
        cloud_msg.is_dense = True

        return cloud_msg

    def create_pose(self, t):
        """创建模拟的无人机位姿"""
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "map"

        if len(self.points_xyz) > 0:
            center_x = (self.points_xyz[:,0].min() + self.points_xyz[:,0].max()) / 2
            center_y = (self.points_xyz[:,1].min() + self.points_xyz[:,1].max()) / 2
            center_z = self.points_xyz[:,2].max() + 5.0

            radius = 10.0
            pose_msg.pose.position.x = float(center_x + radius * np.cos(t))
            pose_msg.pose.position.y = float(center_y + radius * np.sin(t))
            pose_msg.pose.position.z = float(center_z)
        else:
            pose_msg.pose.position.x = 0.0
            pose_msg.pose.position.y = 0.0
            pose_msg.pose.position.z = 5.0

        pose_msg.pose.orientation.x = 0.0
        pose_msg.pose.orientation.y = 0.0
        pose_msg.pose.orientation.z = float(np.sin(t/2 + np.pi/2))
        pose_msg.pose.orientation.w = float(np.cos(t/2 + np.pi/2))

        return pose_msg

    def timer_callback(self):
        """定时器回调"""
        if self.points_xyz is None or len(self.points_xyz) == 0:
            return

        # 选择发布完整点云还是裁剪后的点云
        if self.crop_enabled and self.cropped_cloud_msg is not None:
            cloud_msg = self.cropped_cloud_msg
        else:
            cloud_msg = self.full_cloud_msg

        # 更新时间戳
        cloud_msg.header.stamp = self.get_clock().now().to_msg()

        # 发布点云
        self.cloud_pub.publish(cloud_msg)

        # 发布位姿
        pose_msg = self.create_pose(self.t)
        self.pose_pub.publish(pose_msg)

        self.t += 0.1

def main(args=None):
    rclpy.init(args=args)

    try:
        publisher = PCDPublisher()
        print(f"\n开始发布点云数据...")
        print(f"点云Topic: /cloud_in")
        print(f"位姿Topic: /mavros/local_position/pose")
        print(f"\n裁剪控制Topics:")
        print(f"  /planning/crop_box - 裁剪框位姿")
        print(f"  /planning/crop_box_size - 裁剪框尺寸")
        print(f"  /planning/crop_reset - 重置裁剪")
        print(f"\n按 Ctrl+C 停止\n")
        rclpy.spin(publisher)
    except KeyboardInterrupt:
        print("\n程序已停止")
    finally:
        if 'publisher' in locals():
            publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
