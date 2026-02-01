#!/bin/bash
# 一键启动脚本 - ROS 2无人机接触作业工具完整测试环境
# 使用test.pcd文件进行测试

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
echo "  ROS 2 无人机接触作业工具 - 一键启动脚本"
echo "=========================================="
echo ""

# 检查ROS环境
print_info "检查ROS环境..."
if [ -z "$ROS_DISTRO" ]; then
    print_info "设置ROS 2环境..."
    if [ -f "/opt/ros/jazzy/setup.bash" ]; then
        source /opt/ros/jazzy/setup.bash
        export PATH=/usr/bin:$PATH
        print_success "ROS 2 Jazzy环境已加载"
    else
        print_error "未找到ROS 2安装"
        exit 1
    fi
else
    print_success "ROS版本: $ROS_DISTRO"
fi

# 检查test.pcd文件
print_info "检查test.pcd文件..."
if [ ! -f "test/test.pcd" ]; then
    print_error "找不到test/test.pcd文件"
    exit 1
fi
PCD_SIZE=$(du -h test/test.pcd | cut -f1)
print_success "找到test.pcd文件 ($PCD_SIZE)"

# 检查符号链接
if [ ! -L "ros2_version/test/test.pcd" ]; then
    print_info "创建符号链接..."
    ln -s ../../test/test.pcd ros2_version/test/test.pcd
    print_success "符号链接已创建"
fi

# 检查rosbridge_server
print_info "检查rosbridge_server..."
if ! ros2 pkg list | grep -q rosbridge_server; then
    print_error "rosbridge_server未安装"
    print_info "请运行: sudo apt-get install ros-jazzy-rosbridge-suite"
    exit 1
fi
print_success "rosbridge_server已安装"

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
/usr/bin/python3.12 /opt/ros/jazzy/lib/rosbridge_server/rosbridge_websocket --port 9090 > $LOG_DIR/rosbridge_$TIMESTAMP.log 2>&1 &
ROSBRIDGE_PID=$!
sleep 3

# 检查rosbridge是否启动成功
if ! ps -p $ROSBRIDGE_PID > /dev/null; then
    print_error "rosbridge启动失败"
    cat $LOG_DIR/rosbridge_$TIMESTAMP.log
    exit 1
fi

if netstat -tuln 2>/dev/null | grep -q ":9090 "; then
    print_success "ROS Bridge已启动 (PID: $ROSBRIDGE_PID, 端口: 9090)"
else
    print_error "ROS Bridge未能监听端口9090"
    exit 1
fi

# 2. 启动PCD发布器
print_info "启动PCD点云发布器..."
print_warning "正在加载394MB的PCD文件，这可能需要10-15秒..."
/usr/bin/python3.12 ros2_version/test/test_pcd_publisher.py > $LOG_DIR/pcd_publisher_$TIMESTAMP.log 2>&1 &
PCD_PID=$!
sleep 15  # 等待PCD文件加载

# 检查PCD发布器是否启动成功
if ! ps -p $PCD_PID > /dev/null; then
    print_error "PCD发布器启动失败"
    cat $LOG_DIR/pcd_publisher_$TIMESTAMP.log
    kill $ROSBRIDGE_PID 2>/dev/null || true
    exit 1
fi

# 检查topics是否发布
print_info "验证ROS topics..."
if ros2 topic list 2>/dev/null | grep -q "/cloud_in"; then
    print_success "PCD发布器已启动 (PID: $PCD_PID)"
    print_success "Topics: /cloud_in, /mavros/local_position/pose"
else
    print_error "点云topic未发布"
    kill $ROSBRIDGE_PID $PCD_PID 2>/dev/null || true
    exit 1
fi

# 3. 启动Web服务器
print_info "启动Web服务器..."
cd ros2_version
python3 -m http.server 8000 > ../$LOG_DIR/webserver_$TIMESTAMP.log 2>&1 &
WEB_PID=$!
cd ..
sleep 2

# 检查Web服务器是否启动成功
if ! ps -p $WEB_PID > /dev/null; then
    print_error "Web服务器启动失败"
    kill $ROSBRIDGE_PID $PCD_PID 2>/dev/null || true
    exit 1
fi

if netstat -tuln 2>/dev/null | grep -q ":8000 "; then
    print_success "Web服务器已启动 (PID: $WEB_PID, 端口: 8000)"
else
    print_error "Web服务器未能监听端口8000"
    kill $ROSBRIDGE_PID $PCD_PID $WEB_PID 2>/dev/null || true
    exit 1
fi

# 保存PID到文件
echo $ROSBRIDGE_PID > $LOG_DIR/rosbridge.pid
echo $PCD_PID > $LOG_DIR/pcd_publisher.pid
echo $WEB_PID > $LOG_DIR/webserver.pid

echo ""
echo "=========================================="
echo "  ✅ 所有服务启动成功！"
echo "=========================================="
echo ""
echo "📊 服务状态:"
echo "  • ROS Bridge:    运行中 (PID: $ROSBRIDGE_PID, 端口: 9090)"
echo "  • PCD发布器:     运行中 (PID: $PCD_PID)"
echo "  • Web服务器:     运行中 (PID: $WEB_PID, 端口: 8000)"
echo ""
echo "📁 日志文件:"
echo "  • ROS Bridge:    $LOG_DIR/rosbridge_$TIMESTAMP.log"
echo "  • PCD发布器:     $LOG_DIR/pcd_publisher_$TIMESTAMP.log"
echo "  • Web服务器:     $LOG_DIR/webserver_$TIMESTAMP.log"
echo ""
echo "🚀 使用方法:"
echo "  1. 在浏览器中打开: ${GREEN}http://localhost:8000${NC}"
echo "  2. 点击'连接'按钮 (ws://localhost:9090)"
echo "  3. 查看点云数据并进行操作"
echo ""
echo "🔍 验证命令:"
echo "  • 查看topics:    ros2 topic list"
echo "  • 查看点云频率:  ros2 topic hz /cloud_in"
echo "  • 查看发布数据:  ros2 topic echo /planning/roi_box"
echo ""
echo "🛑 停止服务:"
echo "  运行: ${YELLOW}./stop.sh${NC}"
echo ""
echo "按 Ctrl+C 可以停止所有服务"
echo "=========================================="
echo ""

# 等待用户中断
trap "echo '' && print_info '正在停止所有服务...' && kill $ROSBRIDGE_PID $PCD_PID $WEB_PID 2>/dev/null || true && print_success '所有服务已停止' && exit 0" INT TERM

# 保持脚本运行
wait
