#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Launch 文件 - 启动完整的 gRPC Planner 集成系统

组件：
1. pointcloud_processor - 点云处理节点
2. grpc_planner_bridge - gRPC 桥接中间件
3. rosbridge_server - WebUI 通信桥接
"""

import os
import sys
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """生成 launch 描述"""

    # 声明参数
    grpc_server_arg = DeclareLaunchArgument(
        'grpc_server',
        default_value='localhost:50051',
        description='gRPC 服务器地址'
    )

    pointcloud_topic_arg = DeclareLaunchArgument(
        'pointcloud_topic',
        default_value='/cloud_in',
        description='点云 topic'
    )

    # 获取参数
    grpc_server = LaunchConfiguration('grpc_server')
    pointcloud_topic = LaunchConfiguration('pointcloud_topic')

    # 1. 点云处理节点
    pointcloud_processor_node = Node(
        package='contact_webui',  # 替换为实际的包名
        executable='pointcloud_processor.py',
        name='pointcloud_processor',
        output='screen',
        parameters=[{
            'input_topic': '/cloud_registered',
            'output_topic': pointcloud_topic,
            'max_points': 500000,
            'publish_rate': 5.0
        }]
    )

    # 2. gRPC Planner 桥接节点
    grpc_bridge_node = Node(
        package='contact_webui',  # 替换为实际的包名
        executable='grpc_planner_bridge.py',
        name='grpc_planner_bridge',
        output='screen',
        parameters=[{
            'grpc_server': grpc_server,
            'pointcloud_topic': pointcloud_topic,
            'cursor_rate_limit': 10.0
        }]
    )

    # 3. rosbridge_server（用于 WebUI 通信）
    rosbridge_server = ExecuteProcess(
        cmd=['rosbridge_websocket'],
        output='screen'
    )

    return LaunchDescription([
        grpc_server_arg,
        pointcloud_topic_arg,
        pointcloud_processor_node,
        grpc_bridge_node,
        rosbridge_server
    ])


if __name__ == '__main__':
    generate_launch_description()
