#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
import math
import time

class FakeVelPublisher(Node):
    def __init__(self):
        super().__init__('fake_vel_node')
        self.pub = self.create_publisher(TwistStamped, '/vel_encoder/data', 10)
        self.timer_period = 0.1  # 10 Hz
        self.timer = self.create_timer(self.timer_period, self.publish_twist)
        self.v = 2.0     # m/s giả lập
        self.w = 1.0     # rad/s giả lập

    def publish_twist(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()  # thời gian hiện tại
        msg.header.frame_id = "base_link"  # frame reference
        msg.twist.linear.x = self.v
        msg.twist.angular.z = self.w
        self.pub.publish(msg)
        self.get_logger().info(f"Publishing fake TwistStamped: linear.x={self.v}, angular.z={self.w}")

def main(args=None):
    rclpy.init(args=args)
    node = FakeVelPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
