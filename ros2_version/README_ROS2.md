# ROS 2 Web端交互式点云裁剪与目标下发工具

一个基于 Web 的 ROS 2 点云可视化和交互工具，用于无人机接触式路径规划任务。

## 功能特性

- ✅ 实时点云可视化（订阅 `sensor_msgs/msg/PointCloud2`）
- ✅ 无人机位姿显示（订阅 `geometry_msgs/msg/PoseStamped`）
- ✅ 交互式 3D ROI 裁剪框（可平移、旋转、缩放）
- ✅ 起始点和末端点选择
- ✅ 参数下发到 ROS 2 Topics
- ✅ 触屏设备支持
- ✅ 响应式 Web 界面

## 与 ROS 1 版本的区别

| 项目 | ROS 1 版本 | ROS 2 版本 |
|------|-----------|-----------|
| Launch 文件 | XML 格式 (.launch) | Python 格式 (.launch.py) |
| 测试脚本 | Python 2/3 兼容 | Python 3 专用 |
| 启动命令 | `roslaunch` | `ros2 launch` |
| 节点运行 | `rosrun` | `ros2 run` |
| Topic 查看 | `rostopic` | `ros2 topic` |

## 系统要求

- ROS 2 (Foxy/Galactic/Humble/Iron/Jazzy)
- Python 3.x
- 现代浏览器（Chrome/Firefox/Edge）
- rosbridge_suite 包

## 安装步骤

### 1. 安装 rosbridge_suite

```bash
# ROS 2 Humble/Iron/Jazzy
sudo apt-get install ros-${ROS_DISTRO}-rosbridge-suite

# 或者从源码编译
cd ~/ros2_ws/src
git clone https://github.com/RobotWebTools/rosbridge_suite.git
cd ~/ros2_ws
colcon build --packages-select rosbridge_suite
```

### 2. 设置脚本权限

```bash
cd ~/contact_webUI/ros2_version
chmod +x start_ros2.sh
chmod +x test/test_publisher.py
```

## 使用方法

### 方式一：使用 launch 文件（推荐）

1. 启动 rosbridge 服务器：

```bash
cd ~/contact_webUI
ros2 launch ros2_version/launch/webui.launch.py
```

2. （可选）启动测试数据发布器：

```bash
# 新终端
python3 ros2_version/test/test_publisher.py
```

3. 启动 Web 服务器：

```bash
# 新终端
cd ros2_version
python3 -m http.server 8000
```

4. 在浏览器中访问：

```
http://localhost:8000
```

### 方式二：使用快速启动脚本

```bash
cd ~/contact_webUI/ros2_version
./start_ros2.sh
```

然后按照提示操作。

## 操作指南

操作方式与 ROS 1 版本完全相同：

### 1. 连接到 ROS

- 在侧边栏输入 ROS Bridge URL（默认：`ws://localhost:9090`）
- 点击"连接"按钮
- 等待连接状态变为"已连接"（绿色指示灯）

### 2. 配置 Topics

- **点云 Topic**: 默认 `/cloud_in`
- **无人机位姿 Topic**: 默认 `/mavros/local_position/pose`

连接成功后会自动订阅这些 Topics。

### 3. 创建 ROI 裁剪框

1. 点击"创建裁剪框"按钮
2. 场景中会出现一个绿色半透明盒子
3. 使用变换控制器调整（平移/旋转/缩放）

### 4. 选择目标点

1. 点击"设置起始点"按钮
2. 在 3D 场景中点击点云表面
3. 重复步骤选择"设置末端点"

### 5. 执行任务

点击"确认执行"按钮，参数将发布到 ROS 2 Topics。

## ROS 2 Topics

### 订阅的 Topics

| Topic | 消息类型 | 说明 |
|-------|---------|------|
| `/cloud_in` | sensor_msgs/msg/PointCloud2 | 全局点云数据 |
| `/mavros/local_position/pose` | geometry_msgs/msg/PoseStamped | 无人机位姿 |

### 发布的 Topics

| Topic | 消息类型 | 说明 |
|-------|---------|------|
| `/planning/roi_box` | geometry_msgs/msg/PoseStamped | ROI 裁剪框位姿 |
| `/planning/goal_points` | geometry_msgs/msg/PoseArray | 起始点和末端点 |

## 项目结构

```
ros2_version/
├── index.html              # 主页面
├── css/
│   └── style.css          # 样式文件
├── js/
│   ├── main.js            # 主应用逻辑
│   ├── pointcloud.js      # 点云处理模块
│   └── roi_selector.js    # ROI 选择器模块
├── launch/
│   └── webui.launch.py    # ROS 2 launch 文件（Python 格式）
├── test/
│   └── test_publisher.py  # ROS 2 测试数据发布器
├── start_ros2.sh          # 快速启动脚本
└── README_ROS2.md         # 本文档
```

## 测试

### 使用测试发布器

```bash
python3 ros2_version/test/test_publisher.py
```

### 验证数据发布

在另一个终端中检查 Topics：

```bash
# 查看所有 topics
ros2 topic list

# 查看点云
ros2 topic echo /cloud_in

# 查看位姿
ros2 topic echo /mavros/local_position/pose

# 查看发布的 ROI
ros2 topic echo /planning/roi_box

# 查看发布的目标点
ros2 topic echo /planning/goal_points

# 查看 topic 频率
ros2 topic hz /cloud_in
```

## 故障排除

### 1. 无法连接到 ROS

```bash
# 检查 rosbridge 是否运行
ros2 node list | grep rosbridge

# 检查端口是否被占用
netstat -tuln | grep 9090
```

### 2. 看不到点云

```bash
# 检查 topic 是否存在
ros2 topic list | grep cloud

# 检查发布频率
ros2 topic hz /cloud_in

# 查看 topic 信息
ros2 topic info /cloud_in
```

### 3. rosbridge_suite 未安装

```bash
# 安装
sudo apt-get install ros-${ROS_DISTRO}-rosbridge-suite

# 验证安装
ros2 pkg list | grep rosbridge
```

## ROS 2 特定命令

```bash
# 启动 launch 文件
ros2 launch ros2_version/launch/webui.launch.py

# 查看节点
ros2 node list

# 查看 topics
ros2 topic list

# 查看消息类型
ros2 interface show sensor_msgs/msg/PointCloud2

# 查看节点信息
ros2 node info /rosbridge_websocket
```

## 从 ROS 1 迁移

如果您之前使用 ROS 1 版本：

1. **Launch 文件**: 使用 Python 格式的 `.launch.py` 文件
2. **命令**: 将 `roslaunch` 改为 `ros2 launch`，`rostopic` 改为 `ros2 topic`
3. **测试脚本**: 使用 Python 3 和 rclpy 库
4. **前端代码**: 无需修改，rosbridge 协议兼容

## 性能优化

与 ROS 1 版本相同的优化策略：
- 点云最大点数限制: 500,000 点
- 自动下采样
- 更新频率限制: 100ms
- WebSocket 节流: 200ms

## 许可证

MIT License

## 贡献

欢迎提交 Issue 和 Pull Request！

## 联系方式

如有问题，请联系项目维护者。
