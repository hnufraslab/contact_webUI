#!/bin/bash
# 停止所有服务的脚本

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

print_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

echo "=========================================="
echo "  停止所有服务"
echo "=========================================="
echo ""

LOG_DIR="logs"

# 从PID文件读取并停止进程
if [ -f "$LOG_DIR/rosbridge.pid" ]; then
    PID=$(cat $LOG_DIR/rosbridge.pid)
    if ps -p $PID > /dev/null 2>&1; then
        print_info "停止ROS Bridge (PID: $PID)..."
        kill $PID 2>/dev/null || true
        print_success "ROS Bridge已停止"
    fi
    rm -f $LOG_DIR/rosbridge.pid
fi

if [ -f "$LOG_DIR/pcd_publisher.pid" ]; then
    PID=$(cat $LOG_DIR/pcd_publisher.pid)
    if ps -p $PID > /dev/null 2>&1; then
        print_info "停止PCD发布器 (PID: $PID)..."
        kill $PID 2>/dev/null || true
        print_success "PCD发布器已停止"
    fi
    rm -f $LOG_DIR/pcd_publisher.pid
fi

if [ -f "$LOG_DIR/webserver.pid" ]; then
    PID=$(cat $LOG_DIR/webserver.pid)
    if ps -p $PID > /dev/null 2>&1; then
        print_info "停止Web服务器 (PID: $PID)..."
        kill $PID 2>/dev/null || true
        print_success "Web服务器已停止"
    fi
    rm -f $LOG_DIR/webserver.pid
fi

# 额外清理：通过进程名查找并停止
print_info "清理残留进程..."
pkill -f rosbridge_websocket 2>/dev/null || true
pkill -f test_pcd_publisher 2>/dev/null || true
pkill -f "http.server 8000" 2>/dev/null || true

sleep 1

echo ""
print_success "所有服务已停止"
echo ""
