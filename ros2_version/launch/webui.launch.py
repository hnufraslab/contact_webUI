from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    ROS 2 Launch 文件
    启动 rosbridge_server 用于 Web 通信
    """
    return LaunchDescription([
        # rosbridge_server 节点
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            output='screen',
            parameters=[{
                'port': 9090,
                'address': '',
                'retry_startup_delay': 5.0,
                'fragment_timeout': 600,
                'delay_between_messages': 0,
                'max_message_size': 10000000,
            }]
        ),

        # rosapi 节点
        Node(
            package='rosapi',
            executable='rosapi_node',
            name='rosapi',
            output='screen'
        ),
    ])
