#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped, TransformStamped
import tf.transformations as tf_trans
import tf2_ros


class PoseToWebFrame:
    def __init__(self):
        rospy.init_node('pose_to_web_frame', anonymous=True)

        self.input_topic = rospy.get_param('~input_topic', '/mavros/local_position/pose')
        self.output_topic = rospy.get_param('~output_topic', '/mavros/local_position/web_pose')
        self.output_frame_id = rospy.get_param('~output_frame_id', 'web_frame')
        self.position_offset = np.array([
            float(rospy.get_param('~position_offset_x', 0.0)),
            float(rospy.get_param('~position_offset_y', 0.0)),
            float(rospy.get_param('~position_offset_z', 0.0)),
        ], dtype=np.float64)

        self.ros_to_web_rotation = np.array([
            [1, 0, 0],
            [0, 0, 1],
            [0, -1, 0],
        ], dtype=np.float64)
        self.ros_to_web_transform = np.eye(4, dtype=np.float64)
        self.ros_to_web_transform[:3, :3] = self.ros_to_web_rotation

        self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster()
        self.publish_static_transform()

        self.pose_pub = rospy.Publisher(self.output_topic, PoseStamped, queue_size=10)
        self.pose_sub = rospy.Subscriber(self.input_topic, PoseStamped, self.pose_callback, queue_size=10)

        rospy.loginfo('=' * 60)
        rospy.loginfo('位姿转换节点启动')
        rospy.loginfo('=' * 60)
        rospy.loginfo(f'输入Topic: {self.input_topic}')
        rospy.loginfo(f'输出Topic: {self.output_topic}')
        rospy.loginfo(f'输出坐标系: {self.output_frame_id}')
        rospy.loginfo(
            f'位置偏移: x={self.position_offset[0]:.3f}, '
            f'y={self.position_offset[1]:.3f}, z={self.position_offset[2]:.3f}'
        )
        rospy.loginfo('=' * 60)

    def publish_static_transform(self):
        transform = TransformStamped()
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = 'camera_init'
        transform.child_frame_id = self.output_frame_id
        transform.transform.translation.x = 0.0
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = 0.0
        transform.transform.rotation.x = 0.7071068
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = 0.0
        transform.transform.rotation.w = 0.7071068
        self.tf_broadcaster.sendTransform(transform)

    def pose_callback(self, message):
        position_ros = np.array([
            message.pose.position.x,
            message.pose.position.y,
            message.pose.position.z,
        ], dtype=np.float64)
        position_web = self.ros_to_web_rotation.dot(position_ros) + self.position_offset

        quaternion_ros = np.array([
            message.pose.orientation.x,
            message.pose.orientation.y,
            message.pose.orientation.z,
            message.pose.orientation.w,
        ], dtype=np.float64)
        rotation_ros = tf_trans.quaternion_matrix(quaternion_ros)
        rotation_web = self.ros_to_web_transform.dot(rotation_ros).dot(np.linalg.inv(self.ros_to_web_transform))
        quaternion_web = tf_trans.quaternion_from_matrix(rotation_web)

        converted_pose = PoseStamped()
        converted_pose.header.stamp = message.header.stamp
        converted_pose.header.frame_id = self.output_frame_id
        converted_pose.pose.position.x = position_web[0]
        converted_pose.pose.position.y = position_web[1]
        converted_pose.pose.position.z = position_web[2]
        converted_pose.pose.orientation.x = quaternion_web[0]
        converted_pose.pose.orientation.y = quaternion_web[1]
        converted_pose.pose.orientation.z = quaternion_web[2]
        converted_pose.pose.orientation.w = quaternion_web[3]

        self.pose_pub.publish(converted_pose)


if __name__ == '__main__':
    try:
        PoseToWebFrame()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
