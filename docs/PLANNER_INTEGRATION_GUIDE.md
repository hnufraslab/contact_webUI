# gRPC Planner 集成系统使用说明

## 系统架构

本系统连接 contact_webUI 和 Docker 中的 gRPC Planner 服务，实现以下功能：

1. **点云更新**：将 ROS 点云发送到 Docker 进行曲面拟合
2. **曲面网格化**：生成曲面网格并在 WebUI 中可视化
3. **游标交互**：实时显示游标在曲面上的最近点，用于选择起点/终点
4. **轨迹规划**：规划曲面轨迹并显示法向量

## 组件说明

### 1. grpc_planner_bridge.py
**位置**: `contact_webUI/src/grpc_planner_bridge.py`

**功能**: ROS 节点，作为 WebUI 和 Docker gRPC 服务之间的桥梁

**订阅的 Topics**:
- `/cloud_in` - 点云数据
- `/planner/convert_mesh_trigger` - 网格转换触发信号
- `/planner/cursor_position` - 游标位置
- `/planner/plan_trajectory_request` - 轨迹规划请求

**发布的 Topics**:
- `/planner/mesh` - 曲面网格 (MarkerArray)
- `/planner/closest_point_position` - 最近点位置 (PointStamped)
- `/planner/closest_point_uv` - 最近点 UV 参数 (Vector3)
- `/planner/trajectory` - 轨迹数据 (MarkerArray)

**参数**:
- `grpc_server`: gRPC 服务器地址 (默认: localhost:50051)
- `pointcloud_topic`: 点云 topic (默认: /cloud_in)
- `cursor_rate_limit`: 游标更新频率限制 (默认: 10.0 Hz)

### 2. planner_visualizer.js
**位置**: `contact_webUI/ros2_version/js/planner_visualizer.js`

**功能**: WebUI 前端可视化组件

**主要方法**:
- `triggerConvertMesh()` - 触发网格转换
- `sendCursorPosition(x, y, z)` - 发送游标位置
- `setCursorMode(enabled)` - 启用/禁用游标模式
- `sendPlanTrajectoryRequest(startUV, goalUV)` - 发送轨迹规划请求

## 快速启动

### 方法 1: 使用启动脚本（推荐）

```bash
cd /home/fraslab/zzx/Docker/integration/contact_webUI
./scripts/start_planner_integration.sh
```

### 方法 2: 手动启动各组件

#### 步骤 1: 启动 Docker gRPC 服务

```bash
# 确保 Docker 容器中的 gRPC 服务正在运行
# 服务应该监听在 localhost:50051
```

#### 步骤 2: 启动 rosbridge_server

```bash
roslaunch rosbridge_server rosbridge_websocket.launch
```

#### 步骤 3: 启动点云处理节点

```bash
cd /home/fraslab/zzx/Docker/integration/contact_webUI
python src/pointcloud_processor.py
```

#### 步骤 4: 启动 gRPC 桥接节点

```bash
cd /home/fraslab/zzx/Docker/integration/contact_webUI
python src/grpc_planner_bridge.py
```

#### 步骤 5: 启动 WebUI

```bash
# 使用任意 HTTP 服务器，例如：
cd /home/fraslab/zzx/Docker/integration/contact_webUI
python -m http.server 8000

# 然后在浏览器中打开：
# http://localhost:8000/ros2_version/index.html
```

## 使用流程

### 1. 连接系统

1. 打开 WebUI: `http://localhost:8000/ros2_version/index.html`
2. 在侧边栏输入 ROS Bridge URL: `ws://localhost:9090`
3. 点击"连接"按钮

### 2. 发送点云数据

点云数据会自动从 `/cloud_in` topic 发送到 Docker 的 gRPC 服务进行曲面拟合。

### 3. 生成曲面网格

1. 在"曲面规划"部分，点击"生成曲面网格"按钮
2. 等待几秒，曲面网格将显示在 3D 视图中
3. 可以使用"显示曲面"复选框控制显示/隐藏

### 4. 选择起点和终点

1. 点击"设置起始点"按钮，进入游标模式
2. 在 3D 视图中移动鼠标，会实时显示曲面上的最近点（紫色球体）
3. 移动到合适位置后，点击鼠标确认选择
4. 重复步骤选择终点

### 5. 规划轨迹

1. 确保起点和终点都已设置
2. 点击"确认执行"按钮
3. 系统将规划轨迹并显示：
   - **红色线条**: UAV 轨迹
   - **绿色线条**: 曲面轨迹
   - **蓝色箭头**: 曲面法向量

### 6. 查看结果

- 使用鼠标左键拖拽旋转视角
- 使用鼠标右键拖拽平移视角
- 使用滚轮缩放视角
- 可以使用"显示轨迹"复选框控制轨迹显示/隐藏

## 配置参数

### gRPC 服务器地址

如果 Docker gRPC 服务不在 localhost:50051，可以修改：

```bash
# 方法 1: 修改 grpc_planner_bridge.py 中的默认值
self.grpc_server = rospy.get_param('~grpc_server', 'your_host:your_port')

# 方法 2: 启动时传递参数
rosrun your_package grpc_planner_bridge.py _grpc_server:=your_host:your_port
```

### 游标更新频率

默认为 10 Hz，可以调整以平衡性能和响应速度：

```bash
rosrun your_package grpc_planner_bridge.py _cursor_rate_limit:=20.0
```

## 故障排查

### 问题 1: 无法连接到 gRPC 服务

**症状**: 日志显示 "gRPC 调用失败"

**解决方案**:
1. 检查 Docker 容器是否运行: `docker ps`
2. 检查端口是否开放: `nc -z localhost 50051`
3. 检查 Docker 网络配置

### 问题 2: 点云未发送到 Docker

**症状**: 没有收到点云数据

**解决方案**:
1. 检查点云 topic 是否有数据: `rostopic echo /cloud_in`
2. 检查 pointcloud_processor 是否运行
3. 查看 grpc_planner_bridge 日志

### 问题 3: 游标不显示最近点

**症状**: 移动鼠标时没有紫色球体

**解决方案**:
1. 确保已生成曲面网格
2. 确保已进入点选模式（点击"设置起始点"或"设置末端点"）
3. 检查浏览器控制台是否有错误

### 问题 4: 轨迹规划失败

**症状**: 点击"确认执行"后没有轨迹显示

**解决方案**:
1. 确保起点和终点的 UV 参数有效（在 [0, 1] 范围内）
2. 检查 gRPC 服务日志
3. 尝试选择不同的起点和终点

## 开发说明

### 添加新的 gRPC 接口

1. 在 `planner.proto` 中定义新的消息和服务
2. 重新生成 Python 代码: `python -m grpc_tools.protoc ...`
3. 在 `grpc_planner_bridge.py` 中添加相应的回调函数
4. 在 `planner_visualizer.js` 中添加前端调用

### 自定义可视化

修改 `planner_visualizer.js` 中的颜色、大小等参数：

```javascript
// 曲面颜色
marker.color.r = 0.5;
marker.color.g = 0.8;
marker.color.b = 1.0;
marker.color.a = 0.6;

// 轨迹颜色
this.createTrajectoryLine(marker, 0xff0000);  // 红色 UAV 轨迹
this.createTrajectoryLine(marker, 0x00ff00);  // 绿色曲面轨迹
```

## 性能优化

1. **降低游标更新频率**: 减少 `cursor_rate_limit` 参数
2. **减少点云数量**: 调整 `pointcloud_processor` 的 `max_points` 参数
3. **降低网格分辨率**: 修改 `ConvertToMeshRequest` 的 `resolution` 参数

## 技术支持

如有问题，请检查：
1. ROS 日志: `rosnode list`, `rostopic list`
2. gRPC 桥接日志: 查看终端输出
3. 浏览器控制台: F12 打开开发者工具

## 版本信息

- ROS 版本: ROS 1 Noetic / ROS 2
- Python 版本: 3.6+
- gRPC 版本: 1.x
- Three.js 版本: 0.128.0
