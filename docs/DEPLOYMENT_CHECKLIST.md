# 部署检查清单

## 部署前检查

### 1. Docker 环境
- [ ] Docker 容器正在运行
- [ ] gRPC 服务监听在 localhost:50051
- [ ] 可以通过 `nc -z localhost 50051` 连接

### 2. ROS 环境
- [ ] ROS 已正确安装（ROS 1 Noetic 或 ROS 2）
- [ ] rosbridge_server 已安装
- [ ] Python 3.6+ 可用

### 3. Python 依赖
- [ ] grpcio 已安装: `pip install grpcio`
- [ ] grpcio-tools 已安装: `pip install grpcio-tools`
- [ ] numpy 已安装: `pip install numpy`

### 4. 文件权限
- [ ] 启动脚本可执行: `chmod +x scripts/start_planner_integration.sh`
- [ ] 测试脚本可执行: `chmod +x scripts/test_integration.py`

## 部署步骤

### 步骤 1: 验证 gRPC 连接
```bash
cd /home/fraslab/zzx/Docker/integration/contact_webUI
python scripts/test_integration.py
```

**预期结果**: 所有 5 个测试通过 ✓

### 步骤 2: 启动系统
```bash
./scripts/start_planner_integration.sh
```

**预期结果**:
- rosbridge_server 启动
- pointcloud_processor 启动
- grpc_planner_bridge 启动
- 无错误信息

### 步骤 3: 验证 ROS Topics
```bash
# 在新终端中
rostopic list
```

**预期结果**: 应该看到以下 topics
```
/cloud_in
/planner/convert_mesh_trigger
/planner/cursor_position
/planner/plan_trajectory_request
/planner/mesh
/planner/closest_point_position
/planner/closest_point_uv
/planner/trajectory
```

### 步骤 4: 打开 WebUI
浏览器访问: `http://localhost:8000/ros2_version/index.html`

**预期结果**:
- 页面正常加载
- 可以看到新的"曲面规划"面板
- 可以看到增强的"目标点选择"面板

### 步骤 5: 测试连接
1. 在 WebUI 中输入 ROS Bridge URL: `ws://localhost:9090`
2. 点击"连接"按钮

**预期结果**:
- 连接状态变为"已连接"（绿色）
- 日志显示"ROS 连接成功"

### 步骤 6: 测试点云
发布测试点云数据到 `/cloud_registered`

**预期结果**:
- grpc_planner_bridge 日志显示"发送点云到 Docker"
- 日志显示"点云更新成功"

### 步骤 7: 测试网格生成
1. 在 WebUI 中点击"生成曲面网格"按钮

**预期结果**:
- 日志显示"正在生成曲面网格..."
- 几秒后，3D 视图中显示半透明蓝色曲面
- 日志显示"网格转换成功: X 个三角形"

### 步骤 8: 测试游标交互
1. 点击"设置起始点"按钮
2. 在 3D 视图中移动鼠标

**预期结果**:
- 出现紫色球体跟随鼠标移动
- 球体始终在曲面上
- 起始 UV 参数实时更新

### 步骤 9: 测试轨迹规划
1. 选择起始点（点击确认）
2. 选择末端点（点击确认）
3. 点击"确认执行"按钮

**预期结果**:
- 日志显示"正在规划轨迹..."
- 几秒后显示：
  - 红色线条（UAV 轨迹）
  - 绿色线条（曲面轨迹）
  - 蓝色箭头（法向量）
- 日志显示"轨迹规划成功: X 个点"

## 常见问题

### 问题 1: 测试脚本失败
**症状**: `test_integration.py` 报错 "无法连接到 gRPC 服务"

**解决方案**:
1. 检查 Docker 容器: `docker ps`
2. 检查端口: `nc -z localhost 50051`
3. 查看 Docker 日志: `docker logs <container_id>`

### 问题 2: rosbridge 连接失败
**症状**: WebUI 显示"连接错误"

**解决方案**:
1. 检查 rosbridge_server 是否运行: `ps aux | grep rosbridge`
2. 检查端口 9090: `netstat -an | grep 9090`
3. 尝试重启 rosbridge_server

### 问题 3: 点云未发送
**症状**: 没有"点云更新成功"日志

**解决方案**:
1. 检查 pointcloud_processor 是否运行
2. 检查 `/cloud_in` topic: `rostopic echo /cloud_in`
3. 查看 grpc_planner_bridge 日志

### 问题 4: 网格不显示
**症状**: 点击"生成曲面网格"后没有显示

**解决方案**:
1. 检查浏览器控制台（F12）是否有错误
2. 检查 `/planner/mesh` topic: `rostopic echo /planner/mesh`
3. 确保点云已成功发送到 Docker

### 问题 5: 游标不工作
**症状**: 移动鼠标时没有紫色球体

**解决方案**:
1. 确保已生成曲面网格
2. 确保已点击"设置起始点"或"设置末端点"
3. 检查 `/planner/closest_point_position` topic

## 性能检查

### CPU 使用率
```bash
top -p $(pgrep -f grpc_planner_bridge)
```
**预期**: < 50%

### 内存使用
```bash
ps aux | grep grpc_planner_bridge
```
**预期**: < 500MB

### 网络延迟
```bash
rostopic hz /planner/closest_point_position
```
**预期**: ~10 Hz（游标模式下）

## 部署完成

如果所有检查都通过，系统已成功部署！

### 下一步
1. 阅读使用说明: `docs/PLANNER_INTEGRATION_GUIDE.md`
2. 查看项目总结: `docs/PROJECT_SUMMARY.md`
3. 根据需求调整配置参数

### 维护建议
- 定期检查 Docker 容器状态
- 监控 ROS 节点运行状态
- 查看日志文件排查问题
- 定期更新依赖包

## 回滚方案

如果部署失败，可以：
1. 停止所有服务: `pkill -f grpc_planner_bridge; pkill -f pointcloud_processor; pkill -f rosbridge`
2. 检查日志找出问题
3. 修复问题后重新部署

## 联系支持

如有问题，请提供：
- 错误日志
- ROS 版本
- Python 版本
- Docker 容器状态
- 浏览器控制台输出
