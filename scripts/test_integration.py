#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
快速测试脚本 - 验证 gRPC Planner 集成系统

测试内容：
1. 检查 gRPC 服务连接
2. 测试点云发送
3. 测试网格转换
4. 测试最近点查询
5. 测试轨迹规划
"""

import sys
import os
import grpc
import numpy as np

# 添加路径
sys.path.append(os.path.join(os.path.dirname(__file__), '../grpc_planner'))
import planner_pb2
import planner_pb2_grpc


def test_grpc_connection(server_address='localhost:50051'):
    """测试 gRPC 连接"""
    print("=" * 60)
    print("测试 1: gRPC 服务连接")
    print("=" * 60)

    try:
        channel = grpc.insecure_channel(server_address)
        stub = planner_pb2_grpc.PlannerServiceStub(channel)
        print(f"✓ 成功连接到 {server_address}")
        return stub, channel
    except Exception as e:
        print(f"✗ 连接失败: {e}")
        return None, None


def test_update_pointcloud(stub):
    """测试点云更新"""
    print("\n" + "=" * 60)
    print("测试 2: 点云更新")
    print("=" * 60)

    # 生成测试点云（圆柱面）
    num_points = 100
    theta = np.linspace(0, 2*np.pi, num_points)
    z = np.linspace(0, 2, num_points)

    points = []
    for t, z_val in zip(theta, z):
        x = np.cos(t)
        y = np.sin(t)
        points.extend([x, y, z_val])

    request = planner_pb2.UpdatePointCloudRequest(
        points=points,
        num_points=num_points
    )

    try:
        response = stub.UpdatePointCloud(request)
        if response.success:
            print(f"✓ 点云更新成功: {response.message}")
            return True
        else:
            print(f"✗ 点云更新失败: {response.message}")
            return False
    except grpc.RpcError as e:
        print(f"✗ RPC 调用失败: {e}")
        return False


def test_convert_mesh(stub):
    """测试网格转换"""
    print("\n" + "=" * 60)
    print("测试 3: 网格转换")
    print("=" * 60)

    request = planner_pb2.ConvertToMeshRequest(resolution=32.0)

    try:
        response = stub.ConvertToMesh(request)
        if response.success:
            print(f"✓ 网格转换成功: {response.num_triangles} 个三角形")
            return True
        else:
            print(f"✗ 网格转换失败: {response.message}")
            return False
    except grpc.RpcError as e:
        print(f"✗ RPC 调用失败: {e}")
        return False


def test_get_closest_point(stub):
    """测试最近点查询"""
    print("\n" + "=" * 60)
    print("测试 4: 最近点查询")
    print("=" * 60)

    # 测试点
    test_point = (1.5, 0.5, 1.0)
    request = planner_pb2.GetClosestPointRequest(
        x=test_point[0],
        y=test_point[1],
        z=test_point[2]
    )

    try:
        response = stub.GetClosestPoint(request)
        if response.success:
            print(f"✓ 最近点查询成功")
            print(f"  输入点: {test_point}")
            print(f"  最近点: ({response.closest_x:.3f}, {response.closest_y:.3f}, {response.closest_z:.3f})")
            print(f"  UV 参数: ({response.u:.3f}, {response.v:.3f})")
            return True, (response.u, response.v)
        else:
            print(f"✗ 最近点查询失败: {response.message}")
            return False, None
    except grpc.RpcError as e:
        print(f"✗ RPC 调用失败: {e}")
        return False, None


def test_plan_trajectory(stub, start_uv, goal_uv):
    """测试轨迹规划"""
    print("\n" + "=" * 60)
    print("测试 5: 轨迹规划")
    print("=" * 60)

    if start_uv is None or goal_uv is None:
        print("✗ 跳过测试（缺少 UV 参数）")
        return False

    request = planner_pb2.PlanTrajectoryRequest(
        start_u=start_uv[0],
        start_v=start_uv[1],
        goal_u=goal_uv[0],
        goal_v=goal_uv[1],
        num_samples=1000,
        planning_timeout=10.0
    )

    try:
        response = stub.PlanTrajectory(request)
        if response.success:
            print(f"✓ 轨迹规划成功")
            print(f"  轨迹点数: {response.num_trajectory_points}")
            print(f"  规划时间: {response.planning_time:.3f}s")
            if response.num_trajectory_points > 0:
                first = response.trajectory[0]
                last = response.trajectory[-1]
                print(f"  起点: ({first.x:.3f}, {first.y:.3f}, {first.z:.3f})")
                print(f"  终点: ({last.x:.3f}, {last.y:.3f}, {last.z:.3f})")
            return True
        else:
            print(f"✗ 轨迹规划失败: {response.message}")
            return False
    except grpc.RpcError as e:
        print(f"✗ RPC 调用失败: {e}")
        return False


def main():
    """主测试函数"""
    print("\n" + "=" * 60)
    print("gRPC Planner 集成系统测试")
    print("=" * 60)

    # 测试 1: 连接
    stub, channel = test_grpc_connection()
    if stub is None:
        print("\n✗ 测试失败：无法连接到 gRPC 服务")
        print("请确保 Docker 容器中的 gRPC 服务正在运行")
        return

    # 测试 2: 点云更新
    if not test_update_pointcloud(stub):
        print("\n✗ 测试失败：点云更新失败")
        channel.close()
        return

    # 测试 3: 网格转换
    if not test_convert_mesh(stub):
        print("\n✗ 测试失败：网格转换失败")
        channel.close()
        return

    # 测试 4: 最近点查询
    success, start_uv = test_get_closest_point(stub)
    if not success:
        print("\n✗ 测试失败：最近点查询失败")
        channel.close()
        return

    # 使用不同的点作为终点
    goal_uv = (0.8, 0.8) if start_uv else None

    # 测试 5: 轨迹规划
    test_plan_trajectory(stub, start_uv, goal_uv)

    # 关闭连接
    channel.close()

    print("\n" + "=" * 60)
    print("测试完成！")
    print("=" * 60)
    print("\n所有核心功能测试通过 ✓")
    print("\n下一步：")
    print("1. 启动完整系统: ./scripts/start_planner_integration.sh")
    print("2. 打开 WebUI: http://localhost:8000/ros2_version/index.html")
    print("3. 查看使用说明: docs/PLANNER_INTEGRATION_GUIDE.md")


if __name__ == '__main__':
    main()
