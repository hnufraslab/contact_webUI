#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
gRPC Planner Bridge - 连接 contact_webUI 和 Docker gRPC 服务的中间件

功能：
1. 订阅 ROS 点云，发送到 Docker 的 UpdatePointCloud
2. 接收触发信号，调用 ConvertToMesh，发布网格数据
3. 订阅游标位置，实时调用 GetClosestPoint，发布最近点
4. 接收规划请求，调用 PlanTrajectory，发布轨迹和法向
"""

import rospy
import grpc
import numpy as np
import struct
import sys
import os
import threading
import csv
from datetime import datetime
from collections import deque

# 导入 ROS 消息类型
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import Point, PointStamped, PoseStamped, Vector3
from std_msgs.msg import Header, Bool, Float64, Float64MultiArray, MultiArrayDimension
from visualization_msgs.msg import Marker, MarkerArray

# 导入 gRPC 生成的代码
sys.path.append(os.path.join(os.path.dirname(__file__), '../../grpc_planner'))
import planner_pb2
import planner_pb2_grpc


class GrpcPlannerBridge:
    """gRPC Planner 桥接中间件"""

    def __init__(self):
        rospy.init_node('grpc_planner_bridge', anonymous=True)

        # 参数配置
        self.grpc_server = rospy.get_param('~grpc_server', 'localhost:50051')
        self.pointcloud_topic = rospy.get_param('~pointcloud_topic', '/cloud_in')
        self.cursor_rate_limit = rospy.get_param('~cursor_rate_limit', 10.0)  # Hz
        self.save_trajectory_csv = rospy.get_param('~save_trajectory_csv', True)  # 是否保存CSV
        self.csv_output_dir = rospy.get_param('~csv_output_dir', '/home/nvidia/zzx_ws/test/path')  # CSV保存目录

        rospy.loginfo("=" * 60)
        rospy.loginfo("gRPC Planner Bridge 启动")
        rospy.loginfo("=" * 60)
        rospy.loginfo(f"gRPC 服务器: {self.grpc_server}")
        rospy.loginfo(f"点云 Topic: {self.pointcloud_topic}")
        rospy.loginfo(f"游标更新频率限制: {self.cursor_rate_limit} Hz")
        rospy.loginfo(f"保存轨迹CSV: {self.save_trajectory_csv}")
        if self.save_trajectory_csv:
            rospy.loginfo(f"CSV输出目录: {self.csv_output_dir}")
        rospy.loginfo("=" * 60)

        # gRPC 连接
        self.grpc_channel = None
        self.grpc_stub = None
        self.grpc_connected = False

        # 状态标志
        self.surface_fitted = False  # 曲面是否已拟合
        self.mesh_generated = False  # 网格是否已生成

        # UV 参数缓存
        self.start_uv = [0.2, 0.2]  # [u, v]
        self.goal_uv = [0.8, 0.8]   # [u, v]

        # 游标位置缓存（用于频率限制）
        self.last_cursor_time = rospy.Time(0)
        self.cursor_min_interval = rospy.Duration(1.0 / self.cursor_rate_limit)

        # 点云缓存
        self.latest_pointcloud = None
        self.pointcloud_lock = threading.Lock()

        # 创建CSV输出目录
        if self.save_trajectory_csv:
            os.makedirs(self.csv_output_dir, exist_ok=True)

        # 初始化 gRPC 连接
        self.connect_grpc()

        # ROS 订阅器
        self.setup_subscribers()

        # ROS 发布器
        self.setup_publishers()

        rospy.loginfo("✓ gRPC Planner Bridge 初始化完成")

    def connect_grpc(self):
        """连接到 gRPC 服务器"""
        try:
            rospy.loginfo(f"正在连接到 gRPC 服务器: {self.grpc_server}")
            self.grpc_channel = grpc.insecure_channel(self.grpc_server)
            self.grpc_stub = planner_pb2_grpc.PlannerServiceStub(self.grpc_channel)

            # 测试连接（尝试调用一个简单的方法）
            # 这里我们不做实际测试，因为服务可能还没有数据
            self.grpc_connected = True
            rospy.loginfo("✓ 成功连接到 gRPC 服务器")

        except Exception as e:
            rospy.logerr(f"连接 gRPC 服务器失败: {e}")
            self.grpc_connected = False

    def setup_subscribers(self):
        """设置 ROS 订阅器"""
        # 订阅点云
        self.pointcloud_sub = rospy.Subscriber(
            self.pointcloud_topic,
            PointCloud2,
            self.pointcloud_callback,
            queue_size=1
        )

        # 订阅网格转换触发信号
        self.convert_mesh_sub = rospy.Subscriber(
            '/planner/convert_mesh_trigger',
            Bool,
            self.convert_mesh_callback,
            queue_size=1
        )

        # 订阅游标位置
        self.cursor_sub = rospy.Subscriber(
            '/planner/cursor_position',
            PointStamped,
            self.cursor_callback,
            queue_size=1
        )

        # 订阅起始点 UV 参数
        self.start_uv_sub = rospy.Subscriber(
            '/planner/start_uv',
            Vector3,
            self.start_uv_callback,
            queue_size=1
        )

        # 订阅终点 UV 参数
        self.goal_uv_sub = rospy.Subscriber(
            '/planner/goal_uv',
            Vector3,
            self.goal_uv_callback,
            queue_size=1
        )

        # 订阅轨迹规划触发信号
        self.plan_trigger_sub = rospy.Subscriber(
            '/planner/plan_trigger',
            Bool,
            self.plan_trigger_callback,
            queue_size=1
        )

        rospy.loginfo("✓ ROS 订阅器设置完成")

    def setup_publishers(self):
        """设置 ROS 发布器"""
        # 发布网格数据（使用 MarkerArray）
        self.mesh_pub = rospy.Publisher(
            '/planner/mesh',
            MarkerArray,
            queue_size=1
        )

        # 发布轨迹（使用 MarkerArray）
        self.trajectory_pub = rospy.Publisher(
            '/planner/trajectory',
            MarkerArray,
            queue_size=1
        )

        # 发布起始点曲面点和法向量
        self.start_surface_point_pub = rospy.Publisher(
            '/planner/start_surface_point',
            PointStamped,
            queue_size=1
        )

        self.start_surface_normal_pub = rospy.Publisher(
            '/planner/start_surface_normal',
            Vector3,
            queue_size=1
        )

        # 发布终点曲面点和法向量
        self.goal_surface_point_pub = rospy.Publisher(
            '/planner/goal_surface_point',
            PointStamped,
            queue_size=1
        )

        self.goal_surface_normal_pub = rospy.Publisher(
            '/planner/goal_surface_normal',
            Vector3,
            queue_size=1
        )

        # 发布UAV轨迹配置（位置和姿态）
        self.uav_config_pub = rospy.Publisher(
            '/planner/uav_configurations',
            Float64MultiArray,
            queue_size=1
        )

        rospy.loginfo("✓ ROS 发布器设置完成")

    def parse_pointcloud2(self, msg):
        """解析 PointCloud2 消息为 numpy 数组"""
        try:
            # 获取字段信息
            field_names = [field.name for field in msg.fields]
            field_offsets = {field.name: field.offset for field in msg.fields}

            # 检查必需的字段
            if 'x' not in field_names or 'y' not in field_names or 'z' not in field_names:
                rospy.logerr("点云缺少 xyz 字段")
                return None

            x_offset = field_offsets['x']
            y_offset = field_offsets['y']
            z_offset = field_offsets['z']

            # 解析点云数据
            point_step = msg.point_step
            num_points = msg.width * msg.height

            points_list = []

            for i in range(num_points):
                base_offset = i * point_step

                # 提取 xyz
                x = struct.unpack_from('f', msg.data, base_offset + x_offset)[0]
                y = struct.unpack_from('f', msg.data, base_offset + y_offset)[0]
                z = struct.unpack_from('f', msg.data, base_offset + z_offset)[0]

                # 过滤无效点
                if np.isnan(x) or np.isnan(y) or np.isnan(z):
                    continue

                points_list.append([x, y, z])

            if len(points_list) == 0:
                return None

            return np.array(points_list, dtype=np.float64)

        except Exception as e:
            rospy.logerr(f"解析点云失败: {e}")
            return None

    def pointcloud_callback(self, msg):
        """点云回调函数 - 接收点云并缓存（不立即发送）"""
        try:
            # 解析点云
            points = self.parse_pointcloud2(msg)

            if points is None or len(points) == 0:
                rospy.logwarn("接收到空点云")
                return

            # 缓存点云
            with self.pointcloud_lock:
                self.latest_pointcloud = points

            rospy.loginfo_throttle(5.0, f"点云已缓存: {len(points)} 个点")

        except Exception as e:
            rospy.logerr(f"点云处理失败: {e}")

    def statistical_outlier_filter(self, points, mean_k=10, std_dev_mul_thresh=1.0):
        """使用 PCL 统计离群点滤波

        Args:
            points: numpy array of shape (N, 3)
            mean_k: 邻域点数量
            std_dev_mul_thresh: 标准差倍数阈值

        Returns:
            filtered_points: 滤波后的点云
        """
        try:
            import pcl

            rospy.loginfo(f"开始 PCL 统计离群点滤波: {len(points)} 个点")

            # 转换为 PCL 点云格式
            cloud = pcl.PointCloud(points.astype(np.float32))

            # 创建统计滤波器
            sor = cloud.make_statistical_outlier_filter()
            sor.set_mean_k(mean_k)
            sor.set_std_dev_mul_thresh(std_dev_mul_thresh)

            # 执行滤波
            filtered_cloud = sor.filter()

            # 转换回 numpy 数组
            filtered_points = np.asarray(filtered_cloud).astype(np.float64)

            removed_count = len(points) - len(filtered_points)
            rospy.loginfo(f"✓ PCL 统计滤波完成: 移除 {removed_count} 个点，保留 {len(filtered_points)} 个点")

            return filtered_points

        except ImportError:
            rospy.logwarn("python-pcl 未安装，跳过离群点滤波")
            rospy.logwarn("安装方法: pip install python-pcl")
            return points
        except Exception as e:
            rospy.logerr(f"PCL 统计滤波失败: {e}")
            return points

    def convert_mesh_callback(self, msg):
        """网格转换回调函数 - 先发送滤波后的点云，再调用 ConvertToMesh"""
        if not self.grpc_connected:
            rospy.logwarn("gRPC 未连接，无法转换网格")
            return

        # 检查是否有缓存的点云
        with self.pointcloud_lock:
            if self.latest_pointcloud is None:
                rospy.logwarn("没有缓存的点云数据，请先发布点云")
                return
            points = self.latest_pointcloud.copy()

        try:
            rospy.loginfo("=" * 60)
            rospy.loginfo("开始处理点云并转换网格")
            rospy.loginfo("=" * 60)

            # 1. 离群点滤波
            filtered_points = self.statistical_outlier_filter(points, mean_k=50, std_dev_mul_thresh=1.0)

            # 2. 发送滤波后的点云到 Docker
            rospy.loginfo(f"发送滤波后的点云到 Docker: {len(filtered_points)} 个点")

            flat_points = filtered_points.flatten().tolist()
            request = planner_pb2.UpdatePointCloudRequest(
                points=flat_points,
                num_points=len(filtered_points)
            )

            response = self.grpc_stub.UpdatePointCloud(request)

            if not response.success:
                rospy.logerr(f"点云更新失败: {response.message}")
                self.surface_fitted = False
                return

            rospy.loginfo(f"✓ 点云更新成功: {response.message}")
            self.surface_fitted = True

            # 3. 调用 ConvertToMesh
            rospy.loginfo("调用 ConvertToMesh...")

            mesh_request = planner_pb2.ConvertToMeshRequest(resolution=64.0)
            mesh_response = self.grpc_stub.ConvertToMesh(mesh_request)

            if mesh_response.success:
                rospy.loginfo(f"✓ 网格转换成功: {mesh_response.num_triangles} 个三角形")
                self.mesh_generated = True

                # 发布网格数据
                self.publish_mesh(mesh_response.triangles)
                self.publish_status()
            else:
                rospy.logwarn(f"网格转换失败: {mesh_response.message}")
                self.mesh_generated = False

            rospy.loginfo("=" * 60)

        except grpc.RpcError as e:
            rospy.logerr(f"gRPC 调用失败: {e}")
            self.grpc_connected = False
        except Exception as e:
            rospy.logerr(f"处理失败: {e}")

    def publish_mesh(self, triangles):
        """发布网格数据为 MarkerArray"""
        try:
            marker_array = MarkerArray()

            # 创建一个 TRIANGLE_LIST marker
            marker = Marker()
            marker.header.frame_id = "web_frame"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "surface_mesh"
            marker.id = 0
            marker.type = Marker.TRIANGLE_LIST
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0

            # 设置颜色和透明度
            marker.color.r = 0.5
            marker.color.g = 0.8
            marker.color.b = 1.0
            marker.color.a = 0.6

            # 设置缩放
            marker.scale.x = 1.0
            marker.scale.y = 1.0
            marker.scale.z = 1.0

            # 添加三角形顶点
            for triangle in triangles:
                vertices = triangle.vertices  # [x1, y1, z1, x2, y2, z2, x3, y3, z3]
                for i in range(0, 9, 3):
                    point = Point()
                    point.x = vertices[i]
                    point.y = vertices[i + 1]
                    point.z = vertices[i + 2]
                    marker.points.append(point)

            marker_array.markers.append(marker)

            # 发布
            self.mesh_pub.publish(marker_array)
            rospy.loginfo(f"✓ 发布网格数据: {len(triangles)} 个三角形")

        except Exception as e:
            rospy.logerr(f"发布网格失败: {e}")

    def cursor_callback(self, msg):
        """游标回调函数 - 实时调用 GetClosestPoint"""
        if not self.grpc_connected:
            return

        if not self.surface_fitted:
            return

        # 频率限制
        current_time = rospy.Time.now()
        if (current_time - self.last_cursor_time) < self.cursor_min_interval:
            return

        self.last_cursor_time = current_time

        try:
            # 调用 gRPC 服务
            request = planner_pb2.GetClosestPointRequest(
                x=msg.point.x,
                y=msg.point.y,
                z=msg.point.z
            )

            response = self.grpc_stub.GetClosestPoint(request)

            if response.success:
                # 发布最近点
                self.publish_closest_point(response, msg.point)

        except grpc.RpcError as e:
            rospy.logerr(f"gRPC 调用失败: {e}")
        except Exception as e:
            rospy.logerr(f"最近点查询失败: {e}")

    def publish_closest_point(self, response, cursor_pos):
        """发布最近点信息"""
        try:
            # 创建自定义消息（我们需要先定义这个消息类型）
            # 暂时使用 PointStamped 和额外的 topic 发布 UV 参数
            
            # 发布最近点位置
            closest_point_msg = PointStamped()
            closest_point_msg.header.frame_id = "web_frame"
            closest_point_msg.header.stamp = rospy.Time.now()
            closest_point_msg.point.x = response.closest_x
            closest_point_msg.point.y = response.closest_y
            closest_point_msg.point.z = response.closest_z

            # 发布到专门的 topic
            if not hasattr(self, 'closest_point_pos_pub'):
                self.closest_point_pos_pub = rospy.Publisher(
                    '/planner/closest_point_position',
                    PointStamped,
                    queue_size=1
                )

            self.closest_point_pos_pub.publish(closest_point_msg)

            # 发布 UV 参数（使用 Vector3，z 设为 0）
            uv_msg = Vector3()
            uv_msg.x = response.u
            uv_msg.y = response.v
            uv_msg.z = 0.0

            if not hasattr(self, 'closest_point_uv_pub'):
                self.closest_point_uv_pub = rospy.Publisher(
                    '/planner/closest_point_uv',
                    Vector3,
                    queue_size=1
                )

            self.closest_point_uv_pub.publish(uv_msg)

        except Exception as e:
            rospy.logerr(f"发布最近点失败: {e}")

    def start_uv_callback(self, msg):
        """起始点 UV 参数回调函数 - 调用 GetSurfacePoint"""
        if not self.grpc_connected:
            return

        if not self.surface_fitted:
            return

        try:
            # 更新缓存的 UV 参数
            self.start_uv = [msg.x, msg.y]

            # 调用 gRPC 服务获取曲面点和法向量
            request = planner_pb2.GetSurfacePointRequest(
                u=msg.x,
                v=msg.y
            )

            response = self.grpc_stub.GetSurfacePoint(request)

            if response.success:
                # 发布曲面点
                point_msg = PointStamped()
                point_msg.header.frame_id = "web_frame"
                point_msg.header.stamp = rospy.Time.now()
                point_msg.point.x = response.x
                point_msg.point.y = response.y
                point_msg.point.z = response.z
                self.start_surface_point_pub.publish(point_msg)

                # 发布法向量
                normal_msg = Vector3()
                normal_msg.x = response.nx
                normal_msg.y = response.ny
                normal_msg.z = response.nz
                self.start_surface_normal_pub.publish(normal_msg)

                rospy.loginfo_throttle(1.0, f"起始点 UV=({msg.x:.2f}, {msg.y:.2f}) -> 曲面点=({response.x:.2f}, {response.y:.2f}, {response.z:.2f})")

        except grpc.RpcError as e:
            rospy.logerr(f"gRPC 调用失败: {e}")
        except Exception as e:
            rospy.logerr(f"起始点 UV 查询失败: {e}")

    def goal_uv_callback(self, msg):
        """终点 UV 参数回调函数 - 调用 GetSurfacePoint"""
        if not self.grpc_connected:
            return

        if not self.surface_fitted:
            return

        try:
            # 更新缓存的 UV 参数
            self.goal_uv = [msg.x, msg.y]

            # 调用 gRPC 服务获取曲面点和法向量
            request = planner_pb2.GetSurfacePointRequest(
                u=msg.x,
                v=msg.y
            )

            response = self.grpc_stub.GetSurfacePoint(request)

            if response.success:
                # 发布曲面点
                point_msg = PointStamped()
                point_msg.header.frame_id = "web_frame"
                point_msg.header.stamp = rospy.Time.now()
                point_msg.point.x = response.x
                point_msg.point.y = response.y
                point_msg.point.z = response.z
                self.goal_surface_point_pub.publish(point_msg)

                # 发布法向量
                normal_msg = Vector3()
                normal_msg.x = response.nx
                normal_msg.y = response.ny
                normal_msg.z = response.nz
                self.goal_surface_normal_pub.publish(normal_msg)

                rospy.loginfo_throttle(1.0, f"终点 UV=({msg.x:.2f}, {msg.y:.2f}) -> 曲面点=({response.x:.2f}, {response.y:.2f}, {response.z:.2f})")

        except grpc.RpcError as e:
            rospy.logerr(f"gRPC 调用失败: {e}")
        except Exception as e:
            rospy.logerr(f"终点 UV 查询失败: {e}")

    def plan_trigger_callback(self, msg):
        """轨迹规划触发回调函数"""
        if not self.grpc_connected:
            rospy.logwarn("gRPC 未连接，无法规划轨迹")
            return

        if not self.surface_fitted:
            rospy.logwarn("曲面尚未拟合，请先发送点云")
            return

        try:
            rospy.loginfo("=" * 60)
            rospy.loginfo(f"开始轨迹规划: start_uv=({self.start_uv[0]:.2f}, {self.start_uv[1]:.2f}), goal_uv=({self.goal_uv[0]:.2f}, {self.goal_uv[1]:.2f})")
            rospy.loginfo("=" * 60)

            # 调用 gRPC 服务
            request = planner_pb2.PlanTrajectoryRequest(
                start_u=self.start_uv[0],
                start_v=self.start_uv[1],
                goal_u=self.goal_uv[0],
                goal_v=self.goal_uv[1],
                num_samples=5000,
                planning_timeout=20.0
            )

            response = self.grpc_stub.PlanTrajectory(request)

            if response.success:
                rospy.loginfo(f"✓ 轨迹规划成功: {response.num_trajectory_points} 个点, 耗时 {response.planning_time:.3f}s")

                # 发布轨迹（只显示曲面轨迹和法向量）
                self.publish_surface_trajectory(response)

                # 发布UAV配置信息
                self.publish_uav_configurations(response)
            else:
                rospy.logwarn(f"轨迹规划失败: {response.message}")

            rospy.loginfo("=" * 60)

        except grpc.RpcError as e:
            rospy.logerr(f"gRPC 调用失败: {e}")
        except Exception as e:
            rospy.logerr(f"轨迹规划处理失败: {e}")

    def plan_trajectory_callback(self, msg):
        """轨迹规划回调函数"""
        if not self.grpc_connected:
            rospy.logwarn("gRPC 未连接，无法规划轨迹")
            return

        if not self.surface_fitted:
            rospy.logwarn("曲面尚未拟合，请先发送点云")
            return

        try:
            rospy.loginfo(f"调用 PlanTrajectory: start_uv=({msg.start_u}, {msg.start_v}), goal_uv=({msg.goal_u}, {msg.goal_v})")

            # 调用 gRPC 服务
            request = planner_pb2.PlanTrajectoryRequest(
                start_u=msg.start_u,
                start_v=msg.start_v,
                goal_u=msg.goal_u,
                goal_v=msg.goal_v,
                num_samples=msg.num_samples if hasattr(msg, 'num_samples') else 5000,
                planning_timeout=msg.planning_timeout if hasattr(msg, 'planning_timeout') else 20.0
            )

            response = self.grpc_stub.PlanTrajectory(request)

            if response.success:
                rospy.loginfo(f"✓ 轨迹规划成功: {response.num_trajectory_points} 个点, 耗时 {response.planning_time:.3f}s")

                # 发布轨迹
                self.publish_trajectory(response)

                # 发布UAV配置信息
                self.publish_uav_configurations(response)
            else:
                rospy.logwarn(f"轨迹规划失败: {response.message}")

        except grpc.RpcError as e:
            rospy.logerr(f"gRPC 调用失败: {e}")
        except Exception as e:
            rospy.logerr(f"轨迹规划处理失败: {e}")

    def publish_trajectory(self, response):
        """发布轨迹数据"""
        try:
            marker_array = MarkerArray()

            # 1. 发布 UAV 轨迹路径（线条）
            trajectory_line = Marker()
            trajectory_line.header.frame_id = "web_frame"
            trajectory_line.header.stamp = rospy.Time.now()
            trajectory_line.ns = "uav_trajectory"
            trajectory_line.id = 0
            trajectory_line.type = Marker.LINE_STRIP
            trajectory_line.action = Marker.ADD
            trajectory_line.pose.orientation.w = 1.0

            trajectory_line.scale.x = 0.02  # 线宽
            trajectory_line.color.r = 1.0
            trajectory_line.color.g = 0.0
            trajectory_line.color.b = 0.0
            trajectory_line.color.a = 1.0

            for point in response.trajectory:
                p = Point()
                p.x = point.x
                p.y = point.y
                p.z = point.z
                trajectory_line.points.append(p)

            marker_array.markers.append(trajectory_line)

            # 2. 发布曲面轨迹路径（线条）
            surface_line = Marker()
            surface_line.header.frame_id = "web_frame"
            surface_line.header.stamp = rospy.Time.now()
            surface_line.ns = "surface_trajectory"
            surface_line.id = 1
            surface_line.type = Marker.LINE_STRIP
            surface_line.action = Marker.ADD
            surface_line.pose.orientation.w = 1.0

            surface_line.scale.x = 0.02
            surface_line.color.r = 0.0
            surface_line.color.g = 1.0
            surface_line.color.b = 0.0
            surface_line.color.a = 1.0

            for point in response.trajectory:
                p = Point()
                p.x = point.sx
                p.y = point.sy
                p.z = point.sz
                surface_line.points.append(p)

            marker_array.markers.append(surface_line)

            # 3. 发布法向量（箭头）
            for i, point in enumerate(response.trajectory):
                if i % 5 != 0:  # 每 5 个点显示一个法向量
                    continue

                arrow = Marker()
                arrow.header.frame_id = "web_frame"
                arrow.header.stamp = rospy.Time.now()
                arrow.ns = "surface_normals"
                arrow.id = 100 + i
                arrow.type = Marker.ARROW
                arrow.action = Marker.ADD

                # 起点：曲面点
                start = Point()
                start.x = point.sx
                start.y = point.sy
                start.z = point.sz

                # 终点：曲面点 + 法向量 * 0.1
                end = Point()
                end.x = point.sx + point.nx * 0.1
                end.y = point.sy + point.ny * 0.1
                end.z = point.sz + point.nz * 0.1

                arrow.points.append(start)
                arrow.points.append(end)

                arrow.scale.x = 0.01  # 箭头轴直径
                arrow.scale.y = 0.02  # 箭头头部直径
                arrow.scale.z = 0.03  # 箭头头部长度

                arrow.color.r = 0.0
                arrow.color.g = 0.0
                arrow.color.b = 1.0
                arrow.color.a = 0.8

                marker_array.markers.append(arrow)

            # 发布
            self.trajectory_pub.publish(marker_array)
            rospy.loginfo(f"✓ 发布轨迹数据: {len(response.trajectory)} 个点")

        except Exception as e:
            rospy.logerr(f"发布轨迹失败: {e}")

    def publish_surface_trajectory(self, response):
        """发布曲面轨迹数据（不显示 UAV 轨迹）"""
        try:
            marker_array = MarkerArray()

            # 1. 发布曲面轨迹路径（线条）
            surface_line = Marker()
            surface_line.header.frame_id = "web_frame"
            surface_line.header.stamp = rospy.Time.now()
            surface_line.ns = "surface_trajectory"
            surface_line.id = 0
            surface_line.type = Marker.LINE_STRIP
            surface_line.action = Marker.ADD
            surface_line.pose.orientation.w = 1.0

            surface_line.scale.x = 0.03  # 线宽
            surface_line.color.r = 1.0
            surface_line.color.g = 0.5
            surface_line.color.b = 0.0
            surface_line.color.a = 1.0

            for point in response.trajectory:
                p = Point()
                p.x = point.sx
                p.y = point.sy
                p.z = point.sz
                surface_line.points.append(p)

            marker_array.markers.append(surface_line)

            # 2. 发布法向量（箭头）
            for i, point in enumerate(response.trajectory):
                if i % 5 != 0:  # 每 5 个点显示一个法向量
                    continue

                arrow = Marker()
                arrow.header.frame_id = "web_frame"
                arrow.header.stamp = rospy.Time.now()
                arrow.ns = "surface_normals"
                arrow.id = 100 + i
                arrow.type = Marker.ARROW
                arrow.action = Marker.ADD

                # 起点：曲面点
                start = Point()
                start.x = point.sx
                start.y = point.sy
                start.z = point.sz

                # 终点：曲面点 + 法向量 * 0.1
                end = Point()
                end.x = point.sx + point.nx * 0.1
                end.y = point.sy + point.ny * 0.1
                end.z = point.sz + point.nz * 0.1

                arrow.points.append(start)
                arrow.points.append(end)

                arrow.scale.x = 0.01  # 箭头轴直径
                arrow.scale.y = 0.02  # 箭头头部直径
                arrow.scale.z = 0.03  # 箭头头部长度

                arrow.color.r = 0.0
                arrow.color.g = 0.0
                arrow.color.b = 1.0
                arrow.color.a = 0.8

                marker_array.markers.append(arrow)

            # 发布
            self.trajectory_pub.publish(marker_array)
            rospy.loginfo(f"✓ 发布曲面轨迹数据: {len(response.trajectory)} 个点")

        except Exception as e:
            rospy.logerr(f"发布曲面轨迹失败: {e}")

    def publish_uav_configurations(self, response):
        """发布UAV配置信息（位置和姿态）到ROS topic"""
        try:
            rospy.loginfo("=" * 60)
            rospy.loginfo("开始发布UAV配置信息...")
            rospy.loginfo("=" * 60)

            # 创建 Float64MultiArray 消息
            config_msg = Float64MultiArray()

            # 设置维度信息
            # dim[0]: 轨迹点数量
            # dim[1]: 每个点的配置维度 (x, y, z, psi, theta = 5个值)
            dim0 = MultiArrayDimension()
            dim0.label = "trajectory_points"
            dim0.size = len(response.trajectory)
            dim0.stride = len(response.trajectory) * 5

            dim1 = MultiArrayDimension()
            dim1.label = "configuration"
            dim1.size = 5
            dim1.stride = 5

            config_msg.layout.dim = [dim0, dim1]
            config_msg.layout.data_offset = 0

            # 填充数据：按行优先顺序 [x0, y0, z0, psi0, theta0, x1, y1, z1, psi1, theta1, ...]
            config_data = []
            for i, point in enumerate(response.trajectory):
                config_data.extend([point.x, point.y, point.z, point.psi, point.theta])
                # 打印前3个和最后3个点的详细信息
                if i < 3 or i >= len(response.trajectory) - 3:
                    rospy.loginfo(f"  Point[{i:3d}]: x={point.x:7.3f}, y={point.y:7.3f}, z={point.z:7.3f}, "
                                 f"psi={point.psi:7.3f}, theta={point.theta:7.3f}")
                elif i == 3:
                    rospy.loginfo(f"  ... ({len(response.trajectory) - 6} more points) ...")

            config_msg.data = config_data

            rospy.loginfo(f"数据数组长度: {len(config_data)} (应该是 {len(response.trajectory) * 5})")

            # 发布消息
            self.uav_config_pub.publish(config_msg)

            rospy.loginfo("=" * 60)
            rospy.loginfo(f"✓ 已发布 {len(response.trajectory)} 个UAV配置点到 /planner/uav_configurations")
            rospy.loginfo(f"  数据格式: Float64MultiArray [{len(response.trajectory)} x 5]")
            rospy.loginfo(f"  每行: [x, y, z, psi, theta]")
            rospy.loginfo(f"  Topic: /planner/uav_configurations")
            rospy.loginfo(f"  消息类型: std_msgs/Float64MultiArray")
            rospy.loginfo("=" * 60)

            # 保存为CSV文件
            if self.save_trajectory_csv:
                csv_file = self.save_trajectory_to_csv(response)
                if csv_file:
                    rospy.loginfo(f"✓ 轨迹已保存到: {csv_file}")

        except Exception as e:
            rospy.logerr(f"发布UAV配置失败: {e}")
            import traceback
            rospy.logerr(traceback.format_exc())

    def save_trajectory_to_csv(self, response):
        """将UAV轨迹配置保存为固定CSV文件并额外保存时间戳备份

        Args:
            response: PlanTrajectoryResponse对象

        Returns:
            str: 固定轨迹CSV文件路径，失败返回None
        """
        try:
            fixed_filepath = os.path.join(self.csv_output_dir, 'exp_path.csv')
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            backup_filepath = os.path.join(self.csv_output_dir, f"exp_path_{timestamp}.csv")

            if os.path.exists(fixed_filepath):
                os.remove(fixed_filepath)
                rospy.loginfo(f"已删除旧轨迹文件: {fixed_filepath}")

            rows = []
            for point in response.trajectory:
                rows.append([point.x, -point.z, point.y, -point.theta, point.psi])

            for filepath in [fixed_filepath, backup_filepath]:
                with open(filepath, 'w', newline='') as csvfile:
                    writer = csv.writer(csvfile)
                    writer.writerow(['x', 'y', 'z', 'psi', 'theta'])
                    writer.writerows(rows)

            rospy.loginfo(f"固定轨迹CSV文件已保存: {fixed_filepath}")
            rospy.loginfo(f"轨迹备份CSV文件已保存: {backup_filepath}")
            rospy.loginfo(f"  包含 {len(response.trajectory)} 个轨迹点")

            return fixed_filepath

        except Exception as e:
            rospy.logerr(f"保存CSV文件失败: {e}")
            import traceback
            rospy.logerr(traceback.format_exc())
            return None

    def publish_status(self):
        """发布状态信息"""
        # 这里可以发布一个简单的状态消息
        # 暂时使用 loginfo
        pass

    def run(self):
        """主循环"""
        rospy.loginfo("gRPC Planner Bridge 运行中...")
        rospy.spin()

    def shutdown(self):
        """关闭"""
        if self.grpc_channel:
            self.grpc_channel.close()
        rospy.loginfo("gRPC Planner Bridge 已关闭")


# 自定义消息类型（临时定义，实际应该在 msg 文件中定义）
class PlanTrajectoryRequest:
    """轨迹规划请求消息"""
    def __init__(self):
        self.start_u = 0.0
        self.start_v = 0.0
        self.goal_u = 0.0
        self.goal_v = 0.0
        self.num_samples = 5000
        self.planning_timeout = 20.0


if __name__ == '__main__':
    try:
        bridge = GrpcPlannerBridge()
        bridge.run()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        rospy.loginfo("\n程序已停止")
