#!/usr/bin/env python3
"""发布小型测试点云"""

import rclpy
from rclpy.node import Node
import struct
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

class SmallCloudPublisher(Node):
    def __init__(self):
        super().__init__('small_cloud_publisher')
        self.pub = self.create_publisher(PointCloud2, '/cloud_in', 10)
        self.timer = self.create_timer(1.0, self.publish)
        self.get_logger().info('Small cloud publisher started')

    def publish(self):
        # 创建 1000 个点的简单点云
        num_points = 1000
        points = []

        for i in range(num_points):
            x = float(np.random.uniform(-10, 10))
            y = float(np.random.uniform(-10, 10))
            z = float(np.random.uniform(0, 5))
            intensity = float(np.random.uniform(0, 100))
            points.append(struct.pack('ffff', x, y, z, intensity))

        cloud_data = b''.join(points)

        msg = PointCloud2()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.height = 1
        msg.width = num_points
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 16
        msg.row_step = 16 * num_points
        msg.data = cloud_data
        msg.is_dense = True

        self.pub.publish(msg)
        self.get_logger().info(f'Published {num_points} points')

def main():
    rclpy.init()
    node = SmallCloudPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
