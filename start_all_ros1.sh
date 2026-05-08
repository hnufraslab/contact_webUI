#!/bin/bash
# 一键启动脚本 - ROS 1无人机接触作业工具完整测试环境
# 包含: roscore, rosbridge, gRPC bridge, Docker planner service, Web服务器

set -e  # 遇到错误立即退出

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 打印带颜色的消息
print_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# 清屏
clear

echo "=========================================="
echo "  ROS 1 无人机接触作业工具 - 一键启动脚本"
echo "  包含曲面拟合功能"
echo "=========================================="
echo ""

# 检查ROS环境
print_info "检查ROS环境..."
if [ -z "$ROS_DISTRO" ]; then
    print_info "设置ROS 1环境..."
    # 尝试常见的ROS 1发行版
    if [ -f "/opt/ros/noetic/setup.bash" ]; then
        source /opt/ros/noetic/setup.bash
        print_success "ROS 1 Noetic环境已加载"
    elif [ -f "/opt/ros/melodic/setup.bash" ]; then
        source /opt/ros/melodic/setup.bash
        print_success "ROS 1 Melodic环境已加载"
    elif [ -f "/opt/ros/kinetic/setup.bash" ]; then
        source /opt/ros/kinetic/setup.bash
        print_success "ROS 1 Kinetic环境已加载"
    else
        print_error "未找到ROS 1安装"
        print_info "请先安装ROS 1 (Noetic/Melodic/Kinetic)"
        exit 1
    fi
else
    print_success "ROS版本: $ROS_DISTRO"
fi

# Source工作空间
if [ -f "$HOME/zzx_ws/devel/setup.bash" ]; then
    source $HOME/zzx_ws/devel/setup.bash
    print_success "工作空间已加载"
fi

# 检查rosbridge_server
print_info "检查rosbridge_server..."
if ! rospack list | grep -q rosbridge_server; then
    print_error "rosbridge_server未安装"
    print_info "请运行: sudo apt-get install ros-$ROS_DISTRO-rosbridge-suite"
    exit 1
fi
print_success "rosbridge_server已安装"

# 检查Docker
print_info "检查Docker..."
if ! command -v docker &> /dev/null; then
    print_warning "Docker未安装，将跳过曲面拟合功能"
    DOCKER_AVAILABLE=0
else
    print_success "Docker已安装"
    DOCKER_AVAILABLE=1
fi

# 检查roscore是否运行
print_info "检查roscore..."
if ! rostopic list &>/dev/null; then
    print_info "启动roscore..."
    roscore > /dev/null 2>&1 &
    ROSCORE_PID=$!
    sleep 3
    if ! rostopic list &>/dev/null; then
        print_error "roscore启动失败"
        exit 1
    fi
    print_success "roscore已启动 (PID: $ROSCORE_PID)"
    ROSCORE_STARTED=1
else
    print_success "roscore已在运行"
    ROSCORE_STARTED=0
fi

# 检查端口占用
print_info "检查端口占用..."
if netstat -tuln 2>/dev/null | grep -q ":9090 "; then
    print_warning "端口9090已被占用，尝试清理..."
    pkill -f rosbridge_websocket || true
    sleep 2
fi
if netstat -tuln 2>/dev/null | grep -q ":8000 "; then
    print_warning "端口8000已被占用，尝试清理..."
    pkill -f "http.server 8000" || true
    sleep 2
fi
if netstat -tuln 2>/dev/null | grep -q ":50051 "; then
    print_warning "端口50051已被占用"
    print_info "如需重启Docker服务，请手动停止旧容器"
fi

# 创建日志目录
LOG_DIR="logs"
mkdir -p $LOG_DIR
TIMESTAMP=$(date +%Y%m%d_%H%M%S)

echo ""
echo "=========================================="
echo "  启动服务"
echo "=========================================="
echo ""

# 1. 启动rosbridge
print_info "启动ROS Bridge WebSocket服务器..."
roslaunch rosbridge_server rosbridge_websocket.launch port:=9090 > $LOG_DIR/rosbridge_$TIMESTAMP.log 2>&1 &
ROSBRIDGE_PID=$!
sleep 3

# 检查rosbridge是否启动成功
if ! ps -p $ROSBRIDGE_PID > /dev/null; then
    print_error "rosbridge启动失败"
    cat $LOG_DIR/rosbridge_$TIMESTAMP.log
    if [ $ROSCORE_STARTED -eq 1 ]; then
        kill $ROSCORE_PID 2>/dev/null || true
    fi
    exit 1
fi

if netstat -tuln 2>/dev/null | grep -q ":9090 "; then
    print_success "ROS Bridge已启动 (PID: $ROSBRIDGE_PID, 端口: 9090)"
else
    print_error "ROS Bridge未能监听端口9090"
    kill $ROSBRIDGE_PID 2>/dev/null || true
    if [ $ROSCORE_STARTED -eq 1 ]; then
        kill $ROSCORE_PID 2>/dev/null || true
    fi
    exit 1
fi

# 2. 检查Docker gRPC Planner Service是否运行
print_info "检查Docker gRPC Planner Service..."
if docker ps 2>/dev/null | grep -q "grpc-planner"; then
    print_success "Docker Planner Service已在运行 (端口: 50051)"
    DOCKER_STARTED=1
elif netstat -tuln 2>/dev/null | grep -q ":50051 "; then
    print_success "检测到端口50051已被占用，假定Docker服务运行中"
    DOCKER_STARTED=1
else
    print_warning "Docker Planner Service未运行"
    print_info "请手动启动: sudo docker run -it --name grpc-planner -p 50051:50051 grpc-planner:arm64-v1 /opt/grpc_planner/build/grpc_planner_server"
    DOCKER_STARTED=0
fi

# 3. 启动点云处理器
print_info "启动点云处理器..."
# python3 src/pointcloud_processor.py > $LOG_DIR/pointcloud_processor_$TIMESTAMP.log 2>&1 &
python3 src/pointcloud_processor.py \
    _input_topic:=/cloud_registered \
    _output_topic:=/cloud_in \
    _accumulate_enabled:=true \
    _accumulate_frames:=20 \
    _max_points:=200000 \
    _max_points_limit:=500000 \
    _publish_rate:=2.0 \
    > $LOG_DIR/pointcloud_processor_$TIMESTAMP.log 2>&1 &
PROCESSOR_PID=$!
sleep 3

if ps -p $PROCESSOR_PID > /dev/null; then
    print_success "点云处理器已启动 (PID: $PROCESSOR_PID)"
else
    print_error "点云处理器启动失败"
    cat $LOG_DIR/pointcloud_processor_$TIMESTAMP.log
    kill $ROSBRIDGE_PID 2>/dev/null || true
    if [ $ROSCORE_STARTED -eq 1 ]; then
        kill $ROSCORE_PID 2>/dev/null || true
    fi
    exit 1
fi

# 4. 启动位姿转换器
print_info "启动位姿转换器..."
python3 src/pose_to_web_frame.py \
    _input_topic:=/mavros/local_position/pose \
    _output_topic:=/mavros/local_position/web_pose \
    _output_frame_id:=web_frame \
    > $LOG_DIR/pose_to_web_frame_$TIMESTAMP.log 2>&1 &
POSE_CONVERTER_PID=$!
sleep 2

if ps -p $POSE_CONVERTER_PID > /dev/null; then
    print_success "位姿转换器已启动 (PID: $POSE_CONVERTER_PID)"
else
    print_error "位姿转换器启动失败"
    cat $LOG_DIR/pose_to_web_frame_$TIMESTAMP.log
    kill $PROCESSOR_PID 2>/dev/null || true
    kill $ROSBRIDGE_PID 2>/dev/null || true
    if [ $ROSCORE_STARTED -eq 1 ]; then
        kill $ROSCORE_PID 2>/dev/null || true
    fi
    exit 1
fi

# 5. 启动gRPC Bridge (如果Docker服务已启动)
if [ $DOCKER_STARTED -eq 1 ]; then
    print_info "启动gRPC Planner Bridge..."
    python3 src/grpc_planner_bridge.py > $LOG_DIR/grpc_bridge_$TIMESTAMP.log 2>&1 &
    GRPC_BRIDGE_PID=$!
    sleep 3

    if ps -p $GRPC_BRIDGE_PID > /dev/null; then
        print_success "gRPC Bridge已启动 (PID: $GRPC_BRIDGE_PID)"
    else
        print_error "gRPC Bridge启动失败"
        cat $LOG_DIR/grpc_bridge_$TIMESTAMP.log
        GRPC_BRIDGE_PID=""
    fi
else
    print_warning "跳过gRPC Bridge (Docker服务未启动)"
    GRPC_BRIDGE_PID=""
fi

# 5. 启动Web服务器
print_info "启动Web服务器..."
python3 -m http.server 8000 > $LOG_DIR/webserver_$TIMESTAMP.log 2>&1 &
WEB_PID=$!
sleep 2

# 检查Web服务器是否启动成功
if ! ps -p $WEB_PID > /dev/null; then
    print_error "Web服务器启动失败"
    kill $ROSBRIDGE_PID 2>/dev/null || true
    kill $PROCESSOR_PID 2>/dev/null || true
    kill $POSE_CONVERTER_PID 2>/dev/null || true
    [ -n "$GRPC_BRIDGE_PID" ] && kill $GRPC_BRIDGE_PID 2>/dev/null || true
    if [ $ROSCORE_STARTED -eq 1 ]; then
        kill $ROSCORE_PID 2>/dev/null || true
    fi
    exit 1
fi

if netstat -tuln 2>/dev/null | grep -q ":8000 "; then
    print_success "Web服务器已启动 (PID: $WEB_PID, 端口: 8000)"
else
    print_error "Web服务器未能监听端口8000"
    kill $ROSBRIDGE_PID $WEB_PID $PROCESSOR_PID 2>/dev/null || true
    [ -n "$GRPC_BRIDGE_PID" ] && kill $GRPC_BRIDGE_PID 2>/dev/null || true
    if [ $ROSCORE_STARTED -eq 1 ]; then
        kill $ROSCORE_PID 2>/dev/null || true
    fi
    exit 1
fi

# 保存PID到文件以便后续清理
PID_FILE="$LOG_DIR/service_pids_$TIMESTAMP.txt"
echo "ROSCORE_STARTED=$ROSCORE_STARTED" > $PID_FILE
[ $ROSCORE_STARTED -eq 1 ] && echo "ROSCORE_PID=$ROSCORE_PID" >> $PID_FILE
echo "ROSBRIDGE_PID=$ROSBRIDGE_PID" >> $PID_FILE
echo "PROCESSOR_PID=$PROCESSOR_PID" >> $PID_FILE
echo "POSE_CONVERTER_PID=$POSE_CONVERTER_PID" >> $PID_FILE
echo "WEB_PID=$WEB_PID" >> $PID_FILE
[ -n "$GRPC_BRIDGE_PID" ] && echo "GRPC_BRIDGE_PID=$GRPC_BRIDGE_PID" >> $PID_FILE
echo "DOCKER_STARTED=$DOCKER_STARTED" >> $PID_FILE

echo ""
echo "==========================================="
echo "  所有服务已启动"
echo "==========================================="
echo ""
print_success "✓ ROS Bridge WebSocket: http://localhost:9090"
print_success "✓ 点云处理器: 运行中"
print_success "✓ 位姿转换器: 运行中"
print_success "✓ Web 界面: http://localhost:8000"
if [ $DOCKER_STARTED -eq 1 ]; then
    print_success "✓ gRPC Planner Service: localhost:50051"
    if [ -n "$GRPC_BRIDGE_PID" ] && ps -p $GRPC_BRIDGE_PID > /dev/null; then
        print_success "✓ gRPC Bridge: 运行中"
    else
        print_warning "✗ gRPC Bridge: 未运行，请查看 $LOG_DIR/grpc_bridge_$TIMESTAMP.log"
    fi
fi
echo ""
print_info "日志文件保存在: $LOG_DIR/"
print_info "PID 文件: $PID_FILE"
echo ""
print_warning "按 Ctrl+C 停止所有服务"
echo ""

# 定义清理函数
cleanup() {
    echo ""
    print_warning "正在停止所有服务..."

    # 停止Web服务器
    if [ -n "$WEB_PID" ] && ps -p $WEB_PID > /dev/null 2>&1; then
        print_info "停止Web服务器 (PID: $WEB_PID)..."
        kill $WEB_PID 2>/dev/null || true
    fi

    # 停止gRPC Bridge
    if [ -n "$GRPC_BRIDGE_PID" ] && ps -p $GRPC_BRIDGE_PID > /dev/null 2>&1; then
        print_info "停止gRPC Bridge (PID: $GRPC_BRIDGE_PID)..."
        kill $GRPC_BRIDGE_PID 2>/dev/null || true
    fi

    # 停止位姿转换器
    if [ -n "$POSE_CONVERTER_PID" ] && ps -p $POSE_CONVERTER_PID > /dev/null 2>&1; then
        print_info "停止位姿转换器 (PID: $POSE_CONVERTER_PID)..."
        kill $POSE_CONVERTER_PID 2>/dev/null || true
    fi

    # 停止点云处理器
    if [ -n "$PROCESSOR_PID" ] && ps -p $PROCESSOR_PID > /dev/null 2>&1; then
        print_info "停止点云处理器 (PID: $PROCESSOR_PID)..."
        kill $PROCESSOR_PID 2>/dev/null || true
    fi

    # 停止rosbridge
    if [ -n "$ROSBRIDGE_PID" ] && ps -p $ROSBRIDGE_PID > /dev/null 2>&1; then
        print_info "停止ROS Bridge (PID: $ROSBRIDGE_PID)..."
        kill $ROSBRIDGE_PID 2>/dev/null || true
    fi

    # 停止roscore (仅当由本脚本启动时)
    if [ $ROSCORE_STARTED -eq 1 ] && [ -n "$ROSCORE_PID" ] && ps -p $ROSCORE_PID > /dev/null 2>&1; then
        print_info "停止roscore (PID: $ROSCORE_PID)..."
        kill $ROSCORE_PID 2>/dev/null || true
    fi

    sleep 2
    print_success "所有服务已停止"
    exit 0
}

# 注册信号处理
trap cleanup SIGINT SIGTERM

# 保持脚本运行
print_info "服务运行中... (按 Ctrl+C 停止)"
while true; do
    # 检查关键进程是否还在运行
    if ! ps -p $ROSBRIDGE_PID > /dev/null 2>&1; then
        print_error "ROS Bridge 进程已退出"
        cleanup
    fi

    if ! ps -p $PROCESSOR_PID > /dev/null 2>&1; then
        print_error "点云处理器进程已退出"
        cleanup
    fi

    if ! ps -p $POSE_CONVERTER_PID > /dev/null 2>&1; then
        print_error "位姿转换器进程已退出"
        cleanup
    fi

    if ! ps -p $WEB_PID > /dev/null 2>&1; then
        print_error "Web服务器进程已退出"
        cleanup
    fi

    if [ -n "$GRPC_BRIDGE_PID" ] && ! ps -p $GRPC_BRIDGE_PID > /dev/null 2>&1; then
        print_warning "gRPC Bridge 进程已退出"
    fi

    if [ $DOCKER_STARTED -eq 1 ] && ! netstat -tuln 2>/dev/null | grep -q ":50051 "; then
        print_warning "Docker Planner Service 端口50051不再监听"
    fi

    sleep 5
done
