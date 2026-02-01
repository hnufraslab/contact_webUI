# ROS 点云裁剪与目标下发工具

一个基于Web的ROS点云可视化和交互工具，用于无人机接触式路径规划任务。支持ROS 1和ROS 2。

## 功能特性

- ✅ 实时点云可视化（订阅 `sensor_msgs/PointCloud2`）
- ✅ 无人机位姿显示（订阅 `geometry_msgs/PoseStamped`）
- ✅ 交互式3D ROI裁剪框（可平移、旋转、缩放）
- ✅ 起始点和末端点选择
- ✅ 参数下发到ROS Topics
- ✅ 触屏设备支持
- ✅ 响应式Web界面

## 技术栈

- **通讯桥接**: rosbridge_suite (WebSocket)
- **前端渲染**: Three.js
- **ROS通信**: roslib.js
- **3D交互**: OrbitControls + TransformControls

## 系统要求

- ROS 1 (Kinetic/Melodic/Noetic) 或 ROS 2 (Foxy/Galactic/Humble/Iron/Jazzy)
- Python 3.x
- 现代浏览器（Chrome/Firefox/Edge）
- rosbridge_suite 包

## 快速开始

### 1. 安装依赖

```bash
# ROS 1
sudo apt-get install ros-${ROS_DISTRO}-rosbridge-suite

# ROS 2
sudo apt-get install ros-${ROS_DISTRO}-rosbridge-suite
```

### 2. 一键启动（推荐）

使用test.pcd文件进行完整测试：

```bash
# 启动所有服务（rosbridge + PCD发布器 + Web服务器）
./start_all.sh

# 停止所有服务
./stop.sh
```

启动后，在浏览器中访问：**http://localhost:8000**

### 3. 手动启动

#### ROS 1版本

```bash
# 终端1: 启动rosbridge
./start.sh
# 或
roslaunch contact_webUI/launch/webui.launch

# 终端2: 启动测试数据发布器（可选）
python test/test_publisher.py

# 终端3: 启动Web服务器
python3 -m http.server 8000

# 浏览器访问
http://localhost:8000
```

#### ROS 2版本

```bash
# 终端1: 启动rosbridge
cd ros2_version && ./start_ros2.sh
# 或
ros2 launch ros2_version/launch/webui.launch.py

# 终端2: 启动测试数据发布器（可选）
python3 ros2_version/test/test_pcd_publisher.py

# 终端3: 启动Web服务器
cd ros2_version && python3 -m http.server 8000

# 浏览器访问
http://localhost:8000
```

## 使用说明

### 1. 连接到ROS

- 在侧边栏输入ROS Bridge URL（默认：`ws://localhost:9090`）
- 点击"连接"按钮
- 等待连接状态变为"已连接"（绿色指示灯）

### 2. 配置Topics

- **点云Topic**: 默认 `/cloud_in`
- **无人机位姿Topic**: 默认 `/mavros/local_position/pose`

连接成功后会自动订阅这些Topics。

### 3. 创建ROI裁剪框

1. 点击"创建裁剪框"按钮
2. 场景中会出现一个绿色半透明盒子
3. 使用变换控制器调整：
   - **平移模式**: 拖拽箭头移动盒子
   - **旋转模式**: 拖拽圆环旋转盒子
   - **缩放模式**: 拖拽立方体缩放盒子
4. 在"变换模式"下拉菜单中切换模式

### 4. 选择目标点

1. 点击"设置起始点"按钮
2. 在3D场景中点击点云表面选择起始位置
3. 绿色标记会显示在选中位置
4. 重复步骤选择"设置末端点"（红色标记）
5. 按ESC键可取消点选模式

### 5. 执行任务

1. 确保已创建ROI裁剪框
2. 确保已设置起始点和末端点
3. 点击"确认执行"按钮
4. 参数将发布到以下Topics：
   - `/planning/roi_box` (geometry_msgs/PoseStamped)
   - `/planning/goal_points` (geometry_msgs/PoseArray)

### 6. 3D视图控制

- **旋转**: 鼠标左键拖拽
- **平移**: 鼠标右键拖拽
- **缩放**: 鼠标滚轮

## ROS Topics

### 订阅的Topics

| Topic | 消息类型 | 说明 |
|-------|---------|------|
| `/cloud_in` | sensor_msgs/PointCloud2 | 全局点云数据 |
| `/mavros/local_position/pose` | geometry_msgs/PoseStamped | 无人机位姿 |

### 发布的Topics

| Topic | 消息类型 | 说明 |
|-------|---------|------|
| `/planning/roi_box` | geometry_msgs/PoseStamped | ROI裁剪框位姿 |
| `/planning/goal_points` | geometry_msgs/PoseArray | 起始点和末端点 |

## 项目结构

```
contact_webUI/
├── index.html              # 主页面
├── css/style.css          # 样式文件
├── js/
│   ├── main.js            # 主应用逻辑
│   ├── pointcloud.js      # 点云处理模块
│   └── roi_selector.js    # ROI选择器模块
├── launch/
│   └── webui.launch       # ROS 1 launch文件
├── test/
│   ├── test_publisher.py  # ROS 1测试发布器
│   ├── test_pcd_publisher.py  # ROS 1 PCD发布器
│   └── test.pcd           # 测试点云数据
├── ros2_version/          # ROS 2版本（结构相同）
├── start_all.sh           # 一键启动脚本
├── stop.sh                # 停止所有服务
└── README.md              # 本文档
```

## 测试

### 使用测试发布器

测试发布器会生成：
- 一个4x4米的波浪形点云平面
- 无人机在圆形轨迹上移动

```bash
# ROS 1
python test/test_publisher.py

# ROS 2
python3 ros2_version/test/test_pcd_publisher.py
```

### 使用真实PCD文件

将PCD文件放在`test/test.pcd`，然后运行：

```bash
./start_all.sh
```

脚本会自动：
- 加载PCD文件
- 进行降采样（从百万级降到10万点）
- 发布到`/cloud_in` topic

### 验证数据发布

```bash
# ROS 1
rostopic list
rostopic echo /cloud_in
rostopic echo /planning/roi_box
rostopic echo /planning/goal_points

# ROS 2
ros2 topic list
ros2 topic echo /cloud_in
ros2 topic echo /planning/roi_box
ros2 topic echo /planning/goal_points
```

## 故障排除

### 1. 无法连接到ROS

```bash
# ROS 1
rosnode list | grep rosbridge

# ROS 2
ros2 node list | grep rosbridge

# 检查端口
netstat -tuln | grep 9090
```

### 2. 看不到点云

```bash
# ROS 1
rostopic list | grep cloud
rostopic hz /cloud_in

# ROS 2
ros2 topic list | grep cloud
ros2 topic hz /cloud_in
```

### 3. 浏览器控制台报错

按F12打开开发者工具查看详细错误信息

### 4. 性能问题

- 减小点云大小（在点云发布端进行下采样）
- 降低点云发布频率
- 调整`pointcloud.js`中的`maxPoints`参数

## 自定义配置

### 修改默认ROI尺寸

编辑`js/roi_selector.js`:

```javascript
this.defaultSize = { x: 2, y: 2, z: 2 };  // 修改为所需尺寸
```

### 修改点云更新频率

编辑`js/pointcloud.js`:

```javascript
this.updateInterval = 100;  // 毫秒
```

### 修改发布的Topic名称

在Web界面的侧边栏中直接修改，或编辑`js/main.js`

## ROS 1 vs ROS 2

本项目同时支持ROS 1和ROS 2：

- **ROS 1版本**: 位于项目根目录
- **ROS 2版本**: 位于`ros2_version/`目录

两个版本功能完全相同，前端代码完全一致，只是后端适配了不同的ROS版本。

### 主要差异

| 项目 | ROS 1 | ROS 2 |
|------|-------|-------|
| Launch文件 | XML格式 | Python格式 |
| 启动命令 | `roslaunch` | `ros2 launch` |
| Topic命令 | `rostopic` | `ros2 topic` |
| 节点命令 | `rosnode` | `ros2 node` |

## 性能优化

- 点云最大点数限制: 500,000点
- 自动下采样
- 更新频率限制: 100ms
- WebSocket节流: 200ms

## 许可证

MIT License

## 贡献

欢迎提交Issue和Pull Request！

## 联系方式

如有问题，请联系项目维护者。
