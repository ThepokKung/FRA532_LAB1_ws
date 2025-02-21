#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import tf2_ros
from geometry_msgs.msg import TransformStamped

class GroundTruthToTF(Node):
    def __init__(self):
        super().__init__('ground_truth_to_tf')
        self.subscription = self.create_subscription(
            Odometry,
            '/ground_truth/odom',  # Topic ground truth ที่ปลั๊กอินเผยแพร่
            self.odom_callback,
            10
        )
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
    
    def odom_callback(self, msg):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "world"          # World frame ที่อ้างอิงจาก Gazebo
        t.child_frame_id = "base_link"         # Frame ของหุ่นยนต์
        t.transform.translation = msg.pose.pose.position
        t.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = GroundTruthToTF()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
