# gRPC Planner 集成系统

连接 contact_webUI 和 Docker gRPC Planner 的完整中间件系统。

## 快速开始

### 1. 测试连接

```bash
cd /home/fraslab/zzx/Docker/integration/contact_webUI
python scripts/test_integration.py
```

### 2. 启动系统

```bash
./scripts/start_planner_integration.sh
```

### 3. 打开 WebUI

浏览器访问: `http://localhost:8000/ros2_version/index.html`

## 功能特性

✅ **点云更新**: 自动将 ROS 点云发送到 Docker 进行曲面拟合
✅ **曲面网格化**: 生成并可视化曲面网格
✅ **游标交互**: 实时显示曲面最近点，选择起点/终点
✅ **轨迹规划**: 规划曲面轨迹并显示法向量

## 核心组件

- **grpc_planner_bridge.py** - ROS 节点，连接 WebUI 和 Docker gRPC 服务
- **planner_visualizer.js** - 前端可视化组件
- **start_planner_integration.sh** - 一键启动脚本

## 文档

- [使用说明](docs/PLANNER_INTEGRATION_GUIDE.md) - 详细的使用指南
- [项目总结](docs/PROJECT_SUMMARY.md) - 完整的项目文档

## 系统架构

```
ROS 点云 → pointcloud_processor → /cloud_in
                                      ↓
                            grpc_planner_bridge
                                      ↓
                            Docker gRPC Service
                                      ↓
                            grpc_planner_bridge
                                      ↓
                            ROS Topics (mesh, trajectory, etc.)
                                      ↓
                            rosbridge_websocket
                                      ↓
                            WebUI (Three.js)
```

## 依赖要求

- ROS 1 Noetic 或 ROS 2
- Python 3.6+
- gRPC Python 库
- Docker (运行 gRPC Planner 服务)
- rosbridge_server
- 现代浏览器（支持 WebGL）

## 配置

编辑 `src/grpc_planner_bridge.py` 修改默认参数：

```python
self.grpc_server = 'localhost:50051'  # gRPC 服务器地址
self.cursor_rate_limit = 10.0         # 游标更新频率
```

## 故障排查

**问题**: 无法连接到 gRPC 服务
**解决**: 检查 Docker 容器是否运行，端口是否开放

**问题**: 游标不显示最近点
**解决**: 确保已生成曲面网格，并进入点选模式

详细故障排查请查看 [使用说明](docs/PLANNER_INTEGRATION_GUIDE.md)

## 开发者

如需扩展功能，请参考：
- `docs/PROJECT_SUMMARY.md` - 技术细节
- `docs/PLANNER_INTEGRATION_GUIDE.md` - 开发指南

## License

请根据项目需求添加适当的许可证。
