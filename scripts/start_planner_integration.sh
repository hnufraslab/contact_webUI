#!/bin/bash
# 启动 gRPC Planner 集成系统

echo "=========================================="
echo "启动 gRPC Planner 集成系统"
echo "=========================================="

# 检查 Docker 容器是否运行
echo "检查 Docker gRPC 服务..."
if ! nc -z localhost 50051 2>/dev/null; then
    echo "警告: Docker gRPC 服务 (localhost:50051) 未运行"
    echo "请先启动 Docker 容器中的 gRPC 服务"
    read -p "是否继续? (y/n) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

# 设置工作目录
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR/.."

echo ""
echo "启动组件..."
echo ""

# 1. 启动 rosbridge_server
echo "[1/3] 启动 rosbridge_server..."
roslaunch rosbridge_server rosbridge_websocket.launch &
ROSBRIDGE_PID=$!
sleep 2

# 2. 启动点云处理节点
echo "[2/3] 启动点云处理节点..."
python src/pointcloud_processor.py &
POINTCLOUD_PID=$!
sleep 1

# 3. 启动位姿转换节点
echo "[3/4] 启动位姿转换节点..."
python src/pose_to_web_frame.py &
POSE_CONVERTER_PID=$!
sleep 1

# 4. 启动 gRPC 桥接节点
echo "[4/4] 启动 gRPC 桥接节点..."
python src/grpc_planner_bridge.py &
BRIDGE_PID=$!
sleep 1

echo ""
echo "=========================================="
echo "所有组件已启动！"
echo "=========================================="
echo "rosbridge_server PID: $ROSBRIDGE_PID"
echo "pointcloud_processor PID: $POINTCLOUD_PID"
echo "pose_to_web_frame PID: $POSE_CONVERTER_PID"
echo "grpc_planner_bridge PID: $BRIDGE_PID"
echo ""
echo "WebUI 地址: http://localhost:8000/ros2_version/index.html"
echo ""
echo "按 Ctrl+C 停止所有服务..."
echo "=========================================="

# 捕获 Ctrl+C 信号
trap "echo ''; echo '正在停止所有服务...'; kill $ROSBRIDGE_PID $POINTCLOUD_PID $POSE_CONVERTER_PID $BRIDGE_PID 2>/dev/null; exit 0" INT

# 等待
wait
