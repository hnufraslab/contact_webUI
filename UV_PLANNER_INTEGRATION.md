# UV 参数轨迹规划集成说明

## 功能概述

已成功集成基于 UV 参数的轨迹规划功能，替代了原有的点选方式。用户现在可以通过滑块调整 UV 参数来实时查看曲面点和法向量，然后执行轨迹规划。

## 修改内容

### 1. HTML 界面修改 (`index.html`)

**新增 UI 组件：**
- 起始点 UV 滑块（U: 0-1, V: 0-1）
- 终点 UV 滑块（U: 0-1, V: 0-1）
- 规划参数输入（采样数量、超时时间）
- 曲面点坐标显示
- "执行轨迹规划"按钮

**移除的组件：**
- "设置起始点"和"设置末端点"按钮（点选模式）
- "确认执行"按钮

### 2. Python Bridge 修改 (`grpc_planner_bridge.py`)

**新增 ROS Topics：**

订阅器：
- `/planner/start_uv` (geometry_msgs/Vector3) - 起始点 UV 参数
- `/planner/goal_uv` (geometry_msgs/Vector3) - 终点 UV 参数
- `/planner/plan_trigger` (std_msgs/Bool) - 轨迹规划触发信号

发布器：
- `/planner/start_surface_point` (geometry_msgs/PointStamped) - 起始点曲面坐标
- `/planner/start_surface_normal` (geometry_msgs/Vector3) - 起始点法向量
- `/planner/goal_surface_point` (geometry_msgs/PointStamped) - 终点曲面坐标
- `/planner/goal_surface_normal` (geometry_msgs/Vector3) - 终点法向量
- `/planner/trajectory` (visualization_msgs/MarkerArray) - 轨迹数据

**新增函数：**
- `start_uv_callback()` - 处理起始点 UV 参数，调用 GetSurfacePoint
- `goal_uv_callback()` - 处理终点 UV 参数，调用 GetSurfacePoint
- `plan_trigger_callback()` - 触发轨迹规划，调用 PlanTrajectory
- `publish_surface_trajectory()` - 发布曲面轨迹（不显示 UAV 轨迹）

### 3. JavaScript 修改 (`main.js`)

**新增功能：**
- UV 滑块事件监听和实时发布
- 订阅曲面点和法向量数据
- 3D 可视化：球体标记（起点绿色、终点红色）+ 法向量箭头（蓝色）
- 轨迹可视化：曲面轨迹线（橙色）+ 法向量箭头

**新增函数：**
- `setupUVPublishers()` - 初始化 UV 参数发布器
- `publishStartUV()` / `publishGoalUV()` - 发布 UV 参数
- `updateStartSurfacePoint()` / `updateGoalSurfacePoint()` - 更新曲面点显示
- `updateStartSurfaceNormal()` / `updateGoalSurfaceNormal()` - 更新法向量显示
- `executePlanning()` - 触发轨迹规划
- `updateTrajectory()` - 更新轨迹显示
- `updatePlanningButton()` - 更新按钮状态

## 工作流程

```
1. 用户调整 UV 滑块
   ↓
2. JavaScript 发布 UV 参数到 ROS
   ↓
3. grpc_planner_bridge 接收 UV 参数
   ↓
4. 调用 gRPC GetSurfacePoint 接口
   ↓
5. 返回曲面点和法向量
   ↓
6. 发布到 ROS Topics
   ↓
7. JavaScript 订阅并在 3D 场景中显示
   ↓
8. 用户点击"执行轨迹规划"
   ↓
9. JavaScript 发布触发信号
   ↓
10. grpc_planner_bridge 调用 PlanTrajectory
    ↓
11. 返回轨迹数据并发布
    ↓
12. JavaScript 显示曲面轨迹和法向量
```

## 使用方法

### 1. 启动系统

```bash
# 启动 gRPC Planner Bridge
cd /home/nvidia/zzx_ws/test/integration/contact_webUI
python3 src/grpc_planner_bridge.py
```

### 2. 打开 Web 界面

在浏览器中访问：`http://localhost:8000/index.html`

### 3. 操作步骤

1. **连接 ROS**：点击"连接"按钮，连接到 `ws://localhost:9090`
2. **发送点云**：确保点云数据发布到 `/cloud_in`
3. **触发网格转换**：
   ```bash
   rostopic pub /planner/convert_mesh_trigger std_msgs/Bool "data: true" --once
   ```
4. **调整起始点 UV**：拖动"起始 U"和"起始 V"滑块，实时查看曲面点和法向量
5. **调整终点 UV**：拖动"终点 U"和"终点 V"滑块
6. **执行规划**：点击"执行轨迹规划"按钮
7. **查看结果**：在 3D 场景中查看曲面轨迹（橙色线）和法向量（蓝色箭头）

## 可视化说明

### 标记颜色
- **绿色球体**：起始点曲面位置
- **红色球体**：终点曲面位置
- **蓝色箭头**：法向量方向
- **橙色线条**：规划的曲面轨迹

### 参数调整
- **UV 范围**：0.0 - 1.0（步长 0.01）
- **采样数量**：1000 - 20000（默认 5000）
- **超时时间**：5 - 60 秒（默认 20 秒）

## 注意事项

1. **曲面必须先拟合**：在调整 UV 参数前，必须先发送点云并触发网格转换
2. **实时更新**：UV 滑块会实时调用 gRPC 接口，频繁调整可能增加计算负载
3. **轨迹显示**：当前只显示曲面轨迹和法向量，不显示 UAV 轨迹
4. **浏览器缓存**：如果界面没有更新，请按 `Ctrl + Shift + R` 强制刷新

## 测试命令

```bash
# 测试起始点 UV 发布
rostopic pub /planner/start_uv geometry_msgs/Vector3 "x: 0.3
y: 0.3
z: 0.0" --once

# 测试终点 UV 发布
rostopic pub /planner/goal_uv geometry_msgs/Vector3 "x: 0.7
y: 0.7
z: 0.0" --once

# 测试规划触发
rostopic pub /planner/plan_trigger std_msgs/Bool "data: true" --once

# 查看曲面点
rostopic echo /planner/start_surface_point
rostopic echo /planner/goal_surface_point

# 查看轨迹
rostopic echo /planner/trajectory
```

## 故障排查

### 问题：滑块调整后没有显示曲面点
- 检查 gRPC 服务是否运行
- 检查曲面是否已拟合（先触发网格转换）
- 查看 bridge 日志：`rosrun rqt_console rqt_console`

### 问题：执行规划按钮禁用
- 确保起始点和终点都已设置（显示坐标）
- 检查 ROS 连接状态

### 问题：轨迹不显示
- 检查规划是否成功（查看日志）
- 确认 `/planner/trajectory` topic 有数据发布

## 版本信息

- HTML 版本：v8
- JavaScript 版本：v8
- Python Bridge：已更新支持 GetSurfacePoint 和 PlanTrajectory 接口
- 更新日期：2026-02-06
