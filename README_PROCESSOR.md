# 点云处理工具使用说明

## 功能概述

这个点云处理工具提供以下功能：

1. **点云订阅与降采样**
   - 订阅输入点云 topic（默认：`/cloud_registered`）
   - 自动检测点云数量，超过阈值则进行降采样
   - 发布处理后的点云到输出 topic（默认：`/cloud_in`）

2. **ROI 裁剪**
   - 接收 web 端发送的裁剪框（`/planning/roi_box`）
   - 根据裁剪框的位置和姿态对点云进行裁剪
   - 支持旋转的裁剪框（使用四元数表示姿态）

3. **重置裁剪**
   - 当 web 端发送重置信号时，恢复输出原始点云
   - 重置信号：位置和姿态都为零的 PoseStamped 消息

## 快速启动

### 方法 1：使用启动脚本（推荐）

```bash
cd /home/nvidia/zzx_ws/src/contact_webUI
./start_processor.sh
```

### 方法 2：使用 roslaunch

```bash
# 使用默认参数
roslaunch contact_webui pointcloud_processor.launch

# 自定义参数
roslaunch contact_webui pointcloud_processor.launch \
    input_topic:=/your_input_topic \
    output_topic:=/your_output_topic \
    max_points:=300000 \
    publish_rate:=10.0
```

### 方法 3：直接运行 Python 脚本

```bash
# 使用默认参数
rosrun contact_webui pointcloud_processor.py

# 使用自定义参数
rosrun contact_webui pointcloud_processor.py \
    _input_topic:=/your_input_topic \
    _output_topic:=/your_output_topic \
    _max_points:=300000 \
    _publish_rate:=10.0
```

## 参数配置

| 参数名 | 默认值 | 说明 |
|--------|--------|------|
| `input_topic` | `/cloud_registered` | 输入点云 topic |
| `output_topic` | `/cloud_in` | 输出点云 topic |
| `roi_topic` | `/planning/roi_box` | ROI 裁剪框 topic |
| `max_points` | `500000` | 最大点数（超过则降采样） |
| `publish_rate` | `5.0` | 发布频率（Hz） |

## 工作流程

```
输入点云 (/cloud_registered)
    ↓
检查点数 > max_points?
    ↓ 是
降采样到 max_points
    ↓
保存为原始点云
    ↓
是否启用裁剪?
    ↓ 是
根据 ROI 框裁剪
    ↓
输出点云 (/cloud_in)
```

## 与 Web 端交互

### 1. 设置裁剪框

Web 端通过 `/planning/roi_box` topic 发送 `geometry_msgs/PoseStamped` 消息：

```python
# 示例：设置一个位于 (1, 2, 3)，无旋转的裁剪框
pose_msg = PoseStamped()
pose_msg.header.frame_id = "map"
pose_msg.pose.position.x = 1.0
pose_msg.pose.position.y = 2.0
pose_msg.pose.position.z = 3.0
pose_msg.pose.orientation.w = 1.0  # 无旋转
```

### 2. 重置裁剪

发送位置和姿态都为零的消息：

```python
# 重置信号
pose_msg = PoseStamped()
pose_msg.header.frame_id = "map"
pose_msg.pose.position.x = 0.0
pose_msg.pose.position.y = 0.0
pose_msg.pose.position.z = 0.0
pose_msg.pose.orientation.x = 0.0
pose_msg.pose.orientation.y = 0.0
pose_msg.pose.orientation.z = 0.0
pose_msg.pose.orientation.w = 1.0
```

## 测试

### 1. 检查节点是否运行

```bash
rosnode list | grep pointcloud_processor
```

### 2. 查看 topic

```bash
# 查看输入 topic
rostopic hz /cloud_registered

# 查看输出 topic
rostopic hz /cloud_in

# 查看 ROI topic
rostopic echo /planning/roi_box
```

### 3. 发送测试 ROI 框

```bash
# 发送一个测试裁剪框
rostopic pub /planning/roi_box geometry_msgs/PoseStamped \
  "header:
    frame_id: 'map'
  pose:
    position: {x: 0.0, y: 0.0, z: 0.0}
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}"

# 发送重置信号
rostopic pub /planning/roi_box geometry_msgs/PoseStamped \
  "header:
    frame_id: 'map'
  pose:
    position: {x: 0.0, y: 0.0, z: 0.0}
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}"
```

## 完整测试流程

### 1. 启动 roscore

```bash
roscore
```

### 2. 启动点云发布器（测试数据）

```bash
# 使用合成数据
python test/test_publisher.py

# 或使用 PCD 文件
python test/test_pcd_publisher.py
```

注意：需要修改测试发布器的输出 topic 为 `/cloud_registered`

### 3. 启动点云处理工具

```bash
./start_processor.sh
```

### 4. 启动 Web UI

```bash
./start_all.sh
```

### 5. 在浏览器中测试

访问 `http://localhost:8000`，在 Web UI 中：
- 查看点云是否正常显示
- 调整 ROI 框位置
- 观察点云裁剪效果
- 点击重置按钮恢复原始点云

## 性能优化建议

1. **降采样阈值**：根据硬件性能调整 `max_points` 参数
   - 高性能设备：500000 - 1000000
   - 中等性能：200000 - 500000
   - 低性能设备：50000 - 200000

2. **发布频率**：根据需求调整 `publish_rate`
   - 实时交互：10 Hz
   - 一般使用：5 Hz
   - 节省资源：1-2 Hz

3. **降采样策略**：当前使用随机采样，可根据需求修改为：
   - 体素网格降采样（更均匀）
   - 距离相关降采样（保留近处细节）

## 故障排查

### 问题 1：节点无法启动

**检查**：
```bash
# 检查 ROS 环境
echo $ROS_DISTRO

# 检查 Python 依赖
python -c "import numpy; import rospy"
```

**解决**：
```bash
# Source ROS 环境
source /opt/ros/noetic/setup.bash

# 安装依赖
pip install numpy
```

### 问题 2：没有接收到点云

**检查**：
```bash
# 检查输入 topic 是否存在
rostopic list | grep cloud_registered

# 检查 topic 类型
rostopic type /cloud_registered
```

**解决**：确保有节点在发布点云到 `/cloud_registered`

### 问题 3：裁剪不生效

**检查**：
```bash
# 查看 ROI topic
rostopic echo /planning/roi_box

# 查看节点日志
rosnode info pointcloud_processor
```

**解决**：确保 Web UI 正确发布 ROI 框消息

## 代码结构

```
src/pointcloud_processor.py
├── PointCloudProcessor (主类)
│   ├── __init__()              # 初始化节点和参数
│   ├── cloud_callback()        # 处理输入点云
│   ├── roi_callback()          # 处理 ROI 框
│   ├── crop_pointcloud()       # 裁剪点云
│   ├── parse_pointcloud2()     # 解析点云消息
│   ├── create_pointcloud2()    # 创建点云消息
│   └── publish_callback()      # 定时发布
```

## 扩展功能建议

1. **多种降采样算法**：添加体素网格、统计滤波等
2. **多个 ROI 框**：支持同时使用多个裁剪框
3. **点云滤波**：添加离群点去除、平滑等功能
4. **性能监控**：添加处理时间、内存使用等统计
5. **动态参数调整**：使用 dynamic_reconfigure

## 相关文件

- `src/pointcloud_processor.py` - 主程序
- `launch/pointcloud_processor.launch` - 启动文件
- `start_processor.sh` - 启动脚本
- `package.xml` - ROS package 配置
- `CMakeLists.txt` - 构建配置
