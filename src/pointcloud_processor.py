#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
点云处理工具（ROS 1版本）
功能：
1. 订阅点云topic（默认/cloud_registered），根据点数进行降采样
2. 发布处理后的点云到/cloud_in
3. 接收web端的ROI裁剪框，对点云进行裁剪
4. 支持重置裁剪，输出原始点云
"""

import rospy
import struct
import os
import numpy as np
from datetime import datetime
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import PoseStamped, Vector3, TransformStamped
#from std_msgs.msg import Header, Bool
from std_msgs.msg import Header, Bool, Int32
import tf.transformations as tf_trans
import tf2_ros

from collections import deque


class PointCloudProcessor:
    def __init__(self):
        rospy.init_node('pointcloud_processor', anonymous=True)

        # 参数配置
        self.input_topic = rospy.get_param('~input_topic', '/cloud_registered')
        self.output_topic = rospy.get_param('~output_topic', '/cloud_in')
        self.crop_box_topic = rospy.get_param('~crop_box_topic', '/planning/crop_box')
        self.crop_box_size_topic = rospy.get_param('~crop_box_size_topic', '/planning/crop_box_size')
        self.crop_reset_topic = rospy.get_param('~crop_reset_topic', '/planning/crop_reset')
        #self.max_points = rospy.get_param('~max_points',500000 )  # 最大点数
        # self.max_points = int(rospy.get_param('~max_points', 800000))  # 默认最大点数
        # self.min_points = int(rospy.get_param('~min_points', 50000))
        # self.max_points_limit = int(rospy.get_param('~max_points_limit', 2000000))
        # self.cloud_max_points_topic = rospy.get_param(
        #     '~cloud_max_points_topic',
        #     '/planning/cloud_max_points'
        # )
        # self.publish_rate = rospy.get_param('~publish_rate', 5.0)  # 发布频率 Hz
        # self.save_cropped_pcd = rospy.get_param('~save_cropped_pcd', True)  # 是否保存裁剪后的PCD
        self.max_points = int(rospy.get_param('~max_points', 200000))  # 最终最多发布给网页的点数
        self.max_points_limit = int(rospy.get_param('~max_points_limit', 500000))  # 安全上限
        self.min_points = int(rospy.get_param('~min_points', 1000))

        self.publish_rate = float(rospy.get_param('~publish_rate', 2.0))  # 点云变密后建议降低发布频率

        # 多帧累计参数
        self.accumulate_enabled = rospy.get_param('~accumulate_enabled', True)
        self.accumulate_frames = int(rospy.get_param('~accumulate_frames', 20))
        self.accumulate_frames = max(1, min(self.accumulate_frames, 100))

        # 网页端动态调参 topic
        self.cloud_max_points_topic = rospy.get_param(
            '~cloud_max_points_topic',
            '/planning/cloud_max_points'
        )
        self.accumulation_frames_topic = rospy.get_param(
            '~accumulation_frames_topic',
            '/planning/accumulation_frames'
        )

        self.save_cropped_pcd = rospy.get_param('~save_cropped_pcd', True)  # 是否保存裁剪后的PCD
        self.pcd_output_dir = rospy.get_param('~pcd_output_dir', '/home/nvidia/zzx_ws/test/path')  # PCD保存目录

        rospy.loginfo("=" * 60)
        rospy.loginfo("点云处理工具启动")
        rospy.loginfo("=" * 60)
        rospy.loginfo(f"输入Topic: {self.input_topic}")
        rospy.loginfo(f"输出Topic: {self.output_topic}")
        rospy.loginfo(f"裁剪框Topic: {self.crop_box_topic}")
        rospy.loginfo(f"裁剪框尺寸Topic: {self.crop_box_size_topic}")
        rospy.loginfo(f"裁剪重置Topic: {self.crop_reset_topic}")
        rospy.loginfo(f"最大点数: {self.max_points}")
        rospy.loginfo(f"发布频率: {self.publish_rate} Hz")
        rospy.loginfo(f"累计点云: {self.accumulate_enabled}")
        rospy.loginfo(f"累计帧数: {self.accumulate_frames}")
        rospy.loginfo(f"最大点数安全上限: {self.max_points_limit}")
        rospy.loginfo(f"点云密度控制Topic: {self.cloud_max_points_topic}")
        rospy.loginfo(f"累计帧数控制Topic: {self.accumulation_frames_topic}")
        rospy.loginfo(f"保存裁剪PCD: {self.save_cropped_pcd}")
        if self.save_cropped_pcd:
            rospy.loginfo(f"PCD输出目录: {self.pcd_output_dir}")
        rospy.loginfo("=" * 60)

        # 输出坐标系frame_id (web_frame与Three.js坐标系一致)
        self.output_frame_id = 'web_frame'

        # 初始化TF广播器，发布camera_init到web_frame的静态变换
        self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster()
        self.publish_static_transform()
        rospy.loginfo(f"已发布静态TF: camera_init -> {self.output_frame_id}")

        # 初始化TF监听器，用于查询坐标变换
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # 坐标转换矩阵：ROS坐标系 -> web_frame (Three.js坐标系)
        # 绕X轴旋转-90度: new_x = x, new_y = z, new_z = -y
        self.ros_to_web_rotation = np.array([
            [1,  0,  0],
            [0,  0,  1],
            [0, -1,  0]
        ], dtype=np.float32)

        # 数据存储
        self.frame_buffer = deque(maxlen=self.accumulate_frames)  # 最近 N 帧点云缓存
        self.original_cloud = None  # 累计后的点云，已经转换到 web_frame
        self.current_cloud = None   # 当前要发布的点云，可能裁剪、可能降采样
        self.crop_box = None        # 裁剪框
        self.crop_box_size = None   # 裁剪框尺寸
        self.enable_crop = False    # 是否启用裁剪

        # 创建PCD输出目录
        if self.save_cropped_pcd:
            os.makedirs(self.pcd_output_dir, exist_ok=True)

        # 订阅器
        self.cloud_sub = rospy.Subscriber(
            self.input_topic,
            PointCloud2,
            self.cloud_callback,
            queue_size=1
        )

        self.crop_box_sub = rospy.Subscriber(
            self.crop_box_topic,
            PoseStamped,
            self.crop_box_callback,
            queue_size=1
        )

        self.crop_box_size_sub = rospy.Subscriber(
            self.crop_box_size_topic,
            Vector3,
            self.crop_box_size_callback,
            queue_size=1
        )

        self.crop_reset_sub = rospy.Subscriber(
            self.crop_reset_topic,
            Bool,
            self.crop_reset_callback,
            queue_size=1
        )

        self.cloud_max_points_sub = rospy.Subscriber(
            self.cloud_max_points_topic,
            Int32,
            self.cloud_max_points_callback,
            queue_size=1
        )

        self.accumulation_frames_sub = rospy.Subscriber(
            self.accumulation_frames_topic,
            Int32,
            self.accumulation_frames_callback,
            queue_size=1
        )

        # 发布器
        self.cloud_pub = rospy.Publisher(
            self.output_topic,
            PointCloud2,
            queue_size=1
        )

        # 定时发布
        self.timer = rospy.Timer(
            rospy.Duration(1.0 / self.publish_rate),
            self.publish_callback
        )

        rospy.loginfo("等待点云数据...")
        rospy.loginfo(f"点云密度控制Topic: {self.cloud_max_points_topic}")

    def publish_static_transform(self):
        """发布camera_init到web_frame的静态TF变换"""
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = 'camera_init'
        t.child_frame_id = self.output_frame_id

        # 平移为零
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0

        # 绕X轴旋转90度的四元数: [sin(π/4), 0, 0, cos(π/4)]
        # = [0.7071068, 0, 0, 0.7071068]
        t.transform.rotation.x = 0.7071068
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 0.7071068

        self.tf_broadcaster.sendTransform(t)
    
 

    #  def cloud_callback(self, msg):
    #     #接收并处理点云数据
    #     try:
    #         # 首次接收数据时输出提示
    #         if self.original_cloud is None:
    #             rospy.loginfo("✓ 成功接收到点云数据，开始处理...")

    #         # 解析点云
    #         points = self.parse_pointcloud2(msg)

    #         if points is None or len(points) == 0:
    #             rospy.logwarn("接收到空点云")
    #             return

    #         original_count = len(points)

    #         # 降采样（如果点数超过最大值）
    #         if original_count > self.max_points:
    #             downsample_ratio = self.max_points / original_count
    #             indices = np.random.choice(
    #                 original_count,
    #                 self.max_points,
    #                 replace=False
    #             )
    #             points = points[indices]
    #             rospy.loginfo(
    #                 f"点云降采样: {original_count} -> {len(points)} "
    #                 f"(比例: {downsample_ratio:.2%})"
    #             )

    #         # 保存原始点云（降采样后，已转换到web_frame坐标系）
    #         # 将点云从ROS坐标系转换到web_frame坐标系
    #         xyz = points[:, :3]
    #         xyz_web = np.dot(xyz, self.ros_to_web_rotation.T)
    #         points[:, :3] = xyz_web

    #         self.original_cloud = {
    #             'points': points,
    #             'frame_id': self.output_frame_id  # 使用web_frame作为frame_id
    #         }

    #         # 如果启用裁剪且有裁剪框，则进行裁剪
    #         if self.enable_crop and self.crop_box is not None and self.crop_box_size is not None:
    #             crop_box_full = {
    #                 'position': self.crop_box['position'],
    #                 'rotation': self.crop_box['rotation'],
    #                 'size': self.crop_box_size
    #             }
    #             cropped_points = self.crop_pointcloud(points, crop_box_full)
    #             if cropped_points is not None and len(cropped_points) > 0:
    #                 self.current_cloud = {
    #                     'points': cropped_points,
    #                     'frame_id': msg.header.frame_id
    #                 }
    #                 rospy.loginfo(
    #                     f"点云裁剪: {len(points)} -> {len(cropped_points)} "
    #                     f"(保留: {len(cropped_points)/len(points):.2%})"
    #                 )
    #             else:
    #                 rospy.logwarn("裁剪后点云为空，使用原始点云")
    #                 self.current_cloud = self.original_cloud
    #         else:
    #             # 不裁剪，直接使用原始点云
    #             self.current_cloud = self.original_cloud

    #     except Exception as e:
    #         rospy.logerr(f"处理点云失败: {e}") 

    # def cloud_callback(self, msg):
    #     """接收并处理点云数据"""
    #     try:
    #         # 首次接收数据时输出提示
    #         if self.original_cloud is None:
    #             rospy.loginfo("✓ 成功接收到点云数据，开始处理...")

    #         # 解析点云
    #         points = self.parse_pointcloud2(msg)

    #         if points is None or len(points) == 0:
    #             rospy.logwarn("接收到空点云")
    #             return

    #         original_count = len(points)

    #         # 将点云从 ROS 坐标系转换到 web_frame 坐标系
    #         xyz = points[:, :3]
    #         xyz_web = np.dot(xyz, self.ros_to_web_rotation.T)
    #         points[:, :3] = xyz_web

    #         # 注意：这里保存完整点云，不提前降采样
    #         # 真正发布给网页端前，再根据 self.max_points 降采样
    #         self.original_cloud = {
    #             'points': points,
    #             'frame_id': self.output_frame_id
    #         }

    #         rospy.loginfo(f"收到原始点云: {original_count} 点，当前网页端最大发布点数: {self.max_points}")

    #         # 根据当前裁剪框和密度设置，生成 current_cloud
    #         self.rebuild_current_cloud(save_crop_pcd=False)

    #     except Exception as e:
    #         rospy.logerr(f"处理点云失败: {e}")

    def cloud_callback(self, msg):
        """接收并处理点云数据：转换坐标、累计多帧、裁剪、降采样"""
        try:
            if self.original_cloud is None:
                rospy.loginfo("✓ 成功接收到点云数据，开始处理...")

            # 解析当前帧点云
            points = self.parse_pointcloud2(msg)

            if points is None or len(points) == 0:
                rospy.logwarn("接收到空点云")
                return

            current_frame_count = len(points)

            # 将点云从 ROS 坐标系转换到 web_frame 坐标系
            xyz = points[:, :3]
            xyz_web = np.dot(xyz, self.ros_to_web_rotation.T)
            points[:, :3] = xyz_web

            # 放入多帧缓存
            self.frame_buffer.append(points)

            # 是否启用多帧累计
            if self.accumulate_enabled and len(self.frame_buffer) > 0:
                accumulated_points = np.concatenate(list(self.frame_buffer), axis=0)
            else:
                accumulated_points = points

            self.original_cloud = {
                'points': accumulated_points,
                'frame_id': self.output_frame_id
            }

            rospy.loginfo(
                f"当前帧: {current_frame_count} 点, "
                f"缓存帧数: {len(self.frame_buffer)}/{self.accumulate_frames}, "
                f"累计点数: {len(accumulated_points)}"
            )

            # 根据当前裁剪框和 max_points 生成发布点云
            self.rebuild_current_cloud(save_crop_pcd=False)

        except Exception as e:
            rospy.logerr(f"处理点云失败: {e}")
            import traceback
            rospy.logerr(traceback.format_exc())
    
    def cloud_max_points_callback(self, msg):
        """接收网页端点云密度上限"""
        try:
            new_max_points = int(msg.data)
            new_max_points = max(self.min_points, min(new_max_points, self.max_points_limit))

            old_max_points = self.max_points
            self.max_points = new_max_points

            rospy.loginfo(f"网页设置点云最大发布点数: {old_max_points} -> {self.max_points}")

            self.rebuild_current_cloud(save_crop_pcd=False)

        except Exception as e:
            rospy.logerr(f"处理点云最大点数失败: {e}")


    def accumulation_frames_callback(self, msg):
        """接收网页端累计帧数设置"""
        try:
            new_frames = int(msg.data)
            new_frames = max(1, min(new_frames, 100))

            old_frames = self.accumulate_frames
            if new_frames == old_frames:
                return

            old_buffer = list(self.frame_buffer)
            self.accumulate_frames = new_frames

            # 重新创建 deque，保留最近 new_frames 帧
            self.frame_buffer = deque(old_buffer[-new_frames:], maxlen=new_frames)

            rospy.loginfo(f"网页设置累计帧数: {old_frames} -> {self.accumulate_frames}")

            if len(self.frame_buffer) > 0:
                accumulated_points = np.concatenate(list(self.frame_buffer), axis=0)
                self.original_cloud = {
                    'points': accumulated_points,
                    'frame_id': self.output_frame_id
                }
                self.rebuild_current_cloud(save_crop_pcd=False)

        except Exception as e:
            rospy.logerr(f"处理累计帧数失败: {e}")


    def downsample_points(self, points):
        """根据 self.max_points 对点云进行均匀降采样"""
        if points is None or len(points) == 0:
            return points

        total_points = len(points)

        if total_points <= self.max_points:
            return points

        indices = np.linspace(
            0,
            total_points - 1,
            self.max_points,
            dtype=np.int64
        )

        sampled_points = points[indices]

        rospy.loginfo(
            f"点云降采样: {total_points} -> {len(sampled_points)} "
            f"(比例: {len(sampled_points) / total_points:.2%})"
        )

        return sampled_points


    def rebuild_current_cloud(self, save_crop_pcd=False):
        """
        根据累计点云、裁剪框和 max_points 重新生成 current_cloud。
        处理顺序：
        1. 使用累计点云
        2. 如果启用裁剪，先裁剪
        3. 再按 max_points 限制发布点数
        """
        if self.original_cloud is None:
            return

        points = self.original_cloud['points']
        frame_id = self.original_cloud['frame_id']

        # 先裁剪，再降采样
        if self.enable_crop and self.crop_box is not None and self.crop_box_size is not None:
            crop_box_full = {
                'position': self.crop_box['position'],
                'rotation': self.crop_box['rotation'],
                'size': self.crop_box_size
            }

            cropped_points = self.crop_pointcloud(points, crop_box_full)

            if cropped_points is not None and len(cropped_points) > 0:
                rospy.loginfo(
                    f"点云裁剪: {len(points)} -> {len(cropped_points)} "
                    f"(保留: {len(cropped_points) / len(points):.2%})"
                )
                points = cropped_points

                if save_crop_pcd and self.save_cropped_pcd:
                    pcd_file = self.save_pcd(cropped_points)
                    if pcd_file:
                        rospy.loginfo(f"✓ 裁剪点云已保存到: {pcd_file}")
            else:
                rospy.logwarn("裁剪后点云为空，使用未裁剪累计点云")

        publish_points = self.downsample_points(points)

        self.current_cloud = {
            'points': publish_points,
            'frame_id': frame_id
        }

        rospy.loginfo(
            f"发布到网页端点云数量: {len(publish_points)} / 当前可用点数: {len(points)}"
        )

    def crop_box_callback(self, msg):
        """接收裁剪框位姿（已经在web_frame坐标系下，无需转换）"""
        try:
            rospy.loginfo("=" * 60)
            rospy.loginfo("收到裁剪框位姿消息")

            # 直接使用web_frame坐标系下的坐标，无需转换
            position = np.array([
                msg.pose.position.x,
                msg.pose.position.y,
                msg.pose.position.z
            ])

            # 直接使用四元数，无需转换
            quaternion = [
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
                msg.pose.orientation.w
            ]

            # 归一化四元数
            q_norm = np.sqrt(sum(q**2 for q in quaternion))
            if q_norm > 0:
                quaternion = [q/q_norm for q in quaternion]

            rotation_matrix = tf_trans.quaternion_matrix(quaternion)[:3, :3]

            rospy.loginfo(f"裁剪框位置 (web_frame): X={position[0]:.2f}, Y={position[1]:.2f}, Z={position[2]:.2f}")
            rospy.loginfo(f"裁剪框四元数: [{quaternion[0]:.3f}, {quaternion[1]:.3f}, {quaternion[2]:.3f}, {quaternion[3]:.3f}]")

            # 保存裁剪框位姿
            self.crop_box = {
                'position': position,
                'rotation': rotation_matrix
            }

            # 如果已经有尺寸信息，则启用裁剪
            if self.crop_box_size is not None:
                self.enable_crop = True
                rospy.loginfo("✓ 启用裁剪模式")
                self.apply_crop()

        except Exception as e:
            rospy.logerr(f"处理裁剪框位姿失败: {e}")
            import traceback
            rospy.logerr(traceback.format_exc())

    def crop_box_size_callback(self, msg):
        """接收裁剪框尺寸（已经在web_frame坐标系下，无需转换）"""
        try:
            rospy.loginfo("收到裁剪框尺寸消息")

            # 直接使用尺寸，无需坐标转换
            size = np.array([msg.x, msg.y, msg.z])
            rospy.loginfo(f"裁剪框尺寸 (web_frame): X={size[0]:.2f}, Y={size[1]:.2f}, Z={size[2]:.2f}")

            self.crop_box_size = size

            # 如果已经有位姿信息，则启用裁剪
            if self.crop_box is not None:
                self.enable_crop = True
                rospy.loginfo("✓ 启用裁剪模式")
                self.apply_crop()

        except Exception as e:
            rospy.logerr(f"处理裁剪框尺寸失败: {e}")
            import traceback
            rospy.logerr(traceback.format_exc())

    def crop_reset_callback(self, msg):
        """接收裁剪重置命令"""
        try:
            rospy.loginfo("=" * 60)
            rospy.loginfo("收到裁剪重置命令")

            self.enable_crop = False
            self.crop_box = None
            self.crop_box_size = None

            # 恢复原始点云
            # if self.original_cloud is not None:
            #     self.current_cloud = self.original_cloud
            #     rospy.loginfo("✓ 已恢复原始点云")

            # rospy.loginfo("=" * 60)
            if self.original_cloud is not None:
                self.rebuild_current_cloud(save_crop_pcd=False)
                rospy.loginfo("✓ 已恢复累计点云")

        except Exception as e:
            rospy.logerr(f"处理重置命令失败: {e}")

    # def apply_crop(self):
    #     """应用裁剪"""
    #     if self.original_cloud is None:
    #         rospy.logwarn("没有原始点云，无法裁剪")
    #         return

    #     if self.crop_box is None or self.crop_box_size is None:
    #         rospy.logwarn("裁剪框信息不完整")
    #         return

    #     # 构建完整的裁剪框信息
    #     crop_box_full = {
    #         'position': self.crop_box['position'],
    #         'rotation': self.crop_box['rotation'],
    #         'size': self.crop_box_size
    #     }

    #     points = self.original_cloud['points']

    #     # 输出调试信息
    #     rospy.loginfo("=" * 60)
    #     rospy.loginfo("开始裁剪点云")
    #     rospy.loginfo(f"原始点云数量: {len(points)}")
    #     rospy.loginfo(f"点云坐标范围:")
    #     rospy.loginfo(f"  X: [{points[:, 0].min():.2f}, {points[:, 0].max():.2f}]")
    #     rospy.loginfo(f"  Y: [{points[:, 1].min():.2f}, {points[:, 1].max():.2f}]")
    #     rospy.loginfo(f"  Z: [{points[:, 2].min():.2f}, {points[:, 2].max():.2f}]")
    #     rospy.loginfo(f"裁剪框中心: [{crop_box_full['position'][0]:.2f}, {crop_box_full['position'][1]:.2f}, {crop_box_full['position'][2]:.2f}]")
    #     rospy.loginfo(f"裁剪框尺寸: [{crop_box_full['size'][0]:.2f}, {crop_box_full['size'][1]:.2f}, {crop_box_full['size'][2]:.2f}]")
    #     rospy.loginfo(f"裁剪框范围:")
    #     rospy.loginfo(f"  X: [{crop_box_full['position'][0] - crop_box_full['size'][0]/2:.2f}, {crop_box_full['position'][0] + crop_box_full['size'][0]/2:.2f}]")
    #     rospy.loginfo(f"  Y: [{crop_box_full['position'][1] - crop_box_full['size'][1]/2:.2f}, {crop_box_full['position'][1] + crop_box_full['size'][1]/2:.2f}]")
    #     rospy.loginfo(f"  Z: [{crop_box_full['position'][2] - crop_box_full['size'][2]/2:.2f}, {crop_box_full['position'][2] + crop_box_full['size'][2]/2:.2f}]")

    #     cropped_points = self.crop_pointcloud(points, crop_box_full)

    #     if cropped_points is not None and len(cropped_points) > 0:
    #         self.current_cloud = {
    #             'points': cropped_points,
    #             'frame_id': self.original_cloud['frame_id']
    #         }
    #         rospy.loginfo(
    #             f"✓ 点云裁剪完成: {len(points)} -> {len(cropped_points)} "
    #             f"(保留: {len(cropped_points)/len(points):.2%})"
    #         )

    #         # 保存裁剪后的点云为PCD文件
    #         if self.save_cropped_pcd:
    #             pcd_file = self.save_pcd(cropped_points)
    #             if pcd_file:
    #                 rospy.loginfo(f"✓ 裁剪点云已保存到: {pcd_file}")

    #         rospy.loginfo("=" * 60)
    #     else:
    #         rospy.logwarn("裁剪后点云为空，使用原始点云")
    #         rospy.logwarn("可能原因：裁剪框位置与点云不重叠")
    #         self.current_cloud = self.original_cloud
    #         rospy.loginfo("=" * 60)

    def apply_crop(self):
        """应用裁剪"""
        if self.original_cloud is None:
            rospy.logwarn("没有原始点云，无法裁剪")
            return

        if self.crop_box is None or self.crop_box_size is None:
            rospy.logwarn("裁剪框信息不完整")
            return

        rospy.loginfo("=" * 60)
        rospy.loginfo("开始裁剪累计点云")
        rospy.loginfo(f"累计点云数量: {len(self.original_cloud['points'])}")

        self.rebuild_current_cloud(save_crop_pcd=True)

        rospy.loginfo("=" * 60)

    def save_pcd(self, points):
        """将裁剪后的点云保存为ASCII格式PCD文件（转换回camera_init坐标系）

        Args:
            points: numpy array of shape (N, 4), columns: x, y, z, intensity (web_frame坐标系)

        Returns:
            str: 保存的PCD文件路径，失败返回None
        """
        try:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"cropped_{timestamp}.pcd"
            filepath = os.path.join(self.pcd_output_dir, filename)

            # 用tf2查询 web_frame -> camera_init 的变换
            transform = self.tf_buffer.lookup_transform(
                'camera_init', self.output_frame_id, rospy.Time(0)
            )
            t = transform.transform.translation
            q = transform.transform.rotation
            rot = tf_trans.quaternion_matrix([q.x, q.y, q.z, q.w])[:3, :3]
            translation = np.array([t.x, t.y, t.z])

            xyz_web = points[:, :3]
            xyz_ros = np.dot(xyz_web, rot.T) + translation

            num_points = len(points)
            has_intensity = points.shape[1] >= 4

            with open(filepath, 'w') as f:
                f.write("# .PCD v0.7 - Point Cloud Data file format\n")
                f.write("VERSION 0.7\n")
                if has_intensity:
                    f.write("FIELDS x y z intensity\n")
                    f.write("SIZE 4 4 4 4\n")
                    f.write("TYPE F F F F\n")
                    f.write("COUNT 1 1 1 1\n")
                else:
                    f.write("FIELDS x y z\n")
                    f.write("SIZE 4 4 4\n")
                    f.write("TYPE F F F\n")
                    f.write("COUNT 1 1 1\n")
                f.write(f"WIDTH {num_points}\n")
                f.write("HEIGHT 1\n")
                f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
                f.write(f"POINTS {num_points}\n")
                f.write("DATA ascii\n")

                for i, point in enumerate(points):
                    if has_intensity:
                        f.write(f"{xyz_ros[i,0]:.6f} {xyz_ros[i,1]:.6f} {xyz_ros[i,2]:.6f} {point[3]:.6f}\n")
                    else:
                        f.write(f"{xyz_ros[i,0]:.6f} {xyz_ros[i,1]:.6f} {xyz_ros[i,2]:.6f}\n")

            rospy.loginfo(f"PCD文件已保存 (camera_init坐标系): {filepath} ({num_points} 个点)")
            return filepath

        except Exception as e:
            rospy.logerr(f"保存PCD文件失败: {e}")
            import traceback
            rospy.logerr(traceback.format_exc())
            return None

    def roi_callback(self, msg):
        """接收ROI裁剪框"""
        try:
            rospy.loginfo("=" * 60)
            rospy.loginfo("收到 ROI 裁剪框消息")

            # 提取位置
            position = np.array([
                msg.pose.position.x,
                msg.pose.position.y,
                msg.pose.position.z
            ])

            # 提取四元数并转换为旋转矩阵
            quaternion = [
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
                msg.pose.orientation.w
            ]
            rotation_matrix = tf_trans.quaternion_matrix(quaternion)[:3, :3]

            rospy.loginfo(f"位置: [{position[0]:.2f}, {position[1]:.2f}, {position[2]:.2f}]")
            rospy.loginfo(f"四元数: [{quaternion[0]:.3f}, {quaternion[1]:.3f}, {quaternion[2]:.3f}, {quaternion[3]:.3f}]")

            # 检查是否是重置信号（位置和姿态都为零）
            if np.allclose(position, 0) and np.allclose(quaternion, [0, 0, 0, 1]):
                rospy.loginfo("✓ 这是重置信号，禁用裁剪")
                self.enable_crop = False
                self.roi_box = None
                # 恢复原始点云
                if self.original_cloud is not None:
                    self.current_cloud = self.original_cloud
                    rospy.loginfo("✓ 已恢复原始点云")
            else:
                # 保存ROI框信息
                self.roi_box = {
                    'position': position,
                    'rotation': rotation_matrix,
                    'size': np.array([2.0, 2.0, 2.0])  # 默认2x2x2米的box
                }
                self.enable_crop = True

                rospy.loginfo("✓ 启用裁剪模式")

                # 如果有原始点云，立即进行裁剪
                if self.original_cloud is not None:
                    points = self.original_cloud['points']
                    cropped_points = self.crop_pointcloud(points, self.roi_box)
                    if cropped_points is not None and len(cropped_points) > 0:
                        self.current_cloud = {
                            'points': cropped_points,
                            'frame_id': self.original_cloud['frame_id']
                        }
                        rospy.loginfo(
                            f"点云裁剪: {len(points)} -> {len(cropped_points)}"
                        )

        except Exception as e:
            rospy.logerr(f"处理ROI框失败: {e}")

    def crop_pointcloud(self, points, roi_box):
        """根据ROI框裁剪点云"""
        try:
            if points is None or len(points) == 0:
                return None

            # 提取xyz坐标
            xyz = points[:, :3]

            # 将点云转换到ROI框的局部坐标系
            # 1. 平移到原点
            translated = xyz - roi_box['position']

            # 2. 旋转到局部坐标系
            # 正确的变换：local = R^T @ world
            # 对于多个点，使用 (R^T @ points^T)^T = points @ R
            local_points = np.dot(translated, roi_box['rotation'])

            # 3. 检查点是否在box内（局部坐标系下的AABB检测）
            half_size = roi_box['size'] / 2.0
            mask = (
                (np.abs(local_points[:, 0]) <= half_size[0]) &
                (np.abs(local_points[:, 1]) <= half_size[1]) &
                (np.abs(local_points[:, 2]) <= half_size[2])
            )

            # 返回裁剪后的点
            cropped_points = points[mask]

            return cropped_points

        except Exception as e:
            rospy.logerr(f"裁剪点云失败: {e}")
            return None

    def parse_pointcloud2(self, msg):
        """解析PointCloud2消息"""
        try:
            # 获取字段信息
            field_names = [field.name for field in msg.fields]
            field_offsets = {field.name: field.offset for field in msg.fields}

            # 检查必需的字段
            if 'x' not in field_names or 'y' not in field_names or 'z' not in field_names:
                rospy.logerr("点云缺少xyz字段")
                return None

            # 确定是否有intensity字段及其偏移量
            has_intensity = 'intensity' in field_names
            x_offset = field_offsets['x']
            y_offset = field_offsets['y']
            z_offset = field_offsets['z']
            intensity_offset = field_offsets.get('intensity', 12)

            # 解析点云数据
            point_step = msg.point_step
            num_points = msg.width * msg.height

            points_list = []

            for i in range(num_points):
                base_offset = i * point_step

                # 提取xyz（使用实际的偏移量）
                x = struct.unpack_from('f', msg.data, base_offset + x_offset)[0]
                y = struct.unpack_from('f', msg.data, base_offset + y_offset)[0]
                z = struct.unpack_from('f', msg.data, base_offset + z_offset)[0]

                # 过滤无效点
                if np.isnan(x) or np.isnan(y) or np.isnan(z):
                    continue

                # 提取intensity（如果有）
                if has_intensity:
                    intensity = struct.unpack_from('f', msg.data, base_offset + intensity_offset)[0]
                    if np.isnan(intensity):
                        intensity = 0.0
                else:
                    intensity = 1.0

                points_list.append([x, y, z, intensity])

            if len(points_list) == 0:
                return None

            return np.array(points_list, dtype=np.float32)

        except Exception as e:
            rospy.logerr(f"解析点云失败: {e}")
            import traceback
            rospy.logerr(traceback.format_exc())
            return None

    def create_pointcloud2(self, points, frame_id):
        """创建PointCloud2消息"""
        try:
            header = Header()
            header.stamp = rospy.Time.now()
            header.frame_id = frame_id

            # 定义字段
            fields = [
                PointField('x', 0, PointField.FLOAT32, 1),
                PointField('y', 4, PointField.FLOAT32, 1),
                PointField('z', 8, PointField.FLOAT32, 1),
                PointField('intensity', 12, PointField.FLOAT32, 1),
            ]

            # 构建点云数据
            cloud_data = []
            for point in points:
                x, y, z, intensity = point
                cloud_data.append(struct.pack('ffff', x, y, z, intensity))

            cloud_bytes = b''.join(cloud_data)

            # 创建PointCloud2消息
            cloud_msg = PointCloud2()
            cloud_msg.header = header
            cloud_msg.height = 1
            cloud_msg.width = len(points)
            cloud_msg.fields = fields
            cloud_msg.is_bigendian = False
            cloud_msg.point_step = 16  # 4 fields * 4 bytes
            cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width
            cloud_msg.data = cloud_bytes
            cloud_msg.is_dense = True

            return cloud_msg

        except Exception as e:
            rospy.logerr(f"创建点云消息失败: {e}")
            return None

    def publish_callback(self, event):
        """定时发布点云"""
        if self.current_cloud is None:
            return

        try:
            # 创建并发布点云消息
            cloud_msg = self.create_pointcloud2(
                self.current_cloud['points'],
                self.current_cloud['frame_id']
            )

            if cloud_msg is not None:
                self.cloud_pub.publish(cloud_msg)

        except Exception as e:
            rospy.logerr(f"发布点云失败: {e}")

    def run(self):
        """主循环"""
        rospy.spin()


if __name__ == '__main__':
    try:
        processor = PointCloudProcessor()
        processor.run()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        rospy.loginfo("\n程序已停止")
