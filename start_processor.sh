#!/bin/bash
# 启动点云处理工具

echo "=========================================="
echo "启动点云处理工具"
echo "=========================================="

# 检查ROS环境
if [ -z "$ROS_DISTRO" ]; then
    echo "错误: ROS环境未配置"
    echo "请先运行: source /opt/ros/noetic/setup.bash"
    exit 1
fi

# 检查工作空间
if [ ! -f "$HOME/zzx_ws/devel/setup.bash" ]; then
    echo "警告: 工作空间未编译"
    echo "正在编译工作空间..."
    cd $HOME/zzx_ws
    catkin_make
    if [ $? -ne 0 ]; then
        echo "错误: 编译失败"
        exit 1
    fi
fi

# Source工作空间
source $HOME/zzx_ws/devel/setup.bash

echo ""
echo "配置信息:"
echo "  输入Topic: /cloud_registered"
echo "  输出Topic: /cloud_in"
echo "  ROI Topic: /planning/roi_box"
echo "  最大点数: 500000"
echo "  发布频率: 5 Hz"
echo ""
echo "启动节点..."
echo ""

# 启动节点
roslaunch contact_webui pointcloud_processor.launch

echo ""
echo "点云处理工具已停止"
