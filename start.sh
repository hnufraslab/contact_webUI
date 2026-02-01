#!/bin/bash
# 快速启动脚本

echo "=========================================="
echo "ROS 无人机接触作业工具 - 快速启动"
echo "=========================================="
echo ""

# 检查 ROS 环境
if [ -z "$ROS_DISTRO" ]; then
    echo "错误: 未检测到 ROS 环境"
    echo "请先运行: source /opt/ros/<distro>/setup.bash"
    exit 1
fi

echo "检测到 ROS 版本: $ROS_DISTRO"
echo ""

# 启动 rosbridge
echo "正在启动 rosbridge_websocket..."
roslaunch rosbridge_server rosbridge_websocket.launch &
ROSBRIDGE_PID=$!
sleep 3

echo ""
echo "=========================================="
echo "rosbridge 已启动 (PID: $ROSBRIDGE_PID)"
echo "=========================================="
echo ""
echo "接下来的步骤:"
echo ""
echo "1. (可选) 启动测试数据发布器:"
echo "   python test/test_publisher.py"
echo ""
echo "2. 启动 Web 服务器:"
echo "   python -m http.server 8000"
echo ""
echo "3. 在浏览器中打开:"
echo "   http://localhost:8000"
echo ""
echo "按 Ctrl+C 停止 rosbridge"
echo ""

# 等待用户中断
wait $ROSBRIDGE_PID
