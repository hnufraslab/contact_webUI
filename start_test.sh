#!/bin/bash
# 完整测试启动脚本 - ROS 2版本

echo "=========================================="
echo "ROS 2 无人机接触作业工具 - 完整测试"
echo "=========================================="
echo ""

# 检查 ROS 环境
if [ -z "$ROS_DISTRO" ]; then
    echo "正在设置 ROS 2 环境..."
    source /opt/ros/jazzy/setup.bash
fi

echo "ROS 版本: $ROS_DISTRO"
echo ""

# 检查 test.pcd 文件
if [ ! -f "test/test.pcd" ]; then
    echo "错误: 找不到 test/test.pcd 文件"
    exit 1
fi

echo "找到 test.pcd 文件 ($(du -h test/test.pcd | cut -f1))"
echo ""

echo "=========================================="
echo "启动步骤:"
echo "=========================================="
echo ""
echo "终端 1 (当前终端) - 启动 rosbridge:"
echo "  ros2 launch ros2_version/launch/webui.launch.py"
echo ""
echo "终端 2 - 启动 PCD 发布器:"
echo "  python3 ros2_version/test/test_pcd_publisher.py"
echo ""
echo "终端 3 - 启动 Web 服务器:"
echo "  cd ros2_version && python3 -m http.server 8000"
echo ""
echo "然后在浏览器中访问: http://localhost:8000"
echo ""
echo "=========================================="
echo "现在启动 rosbridge..."
echo "=========================================="
echo ""

# 启动 rosbridge
ros2 launch ros2_version/launch/webui.launch.py
