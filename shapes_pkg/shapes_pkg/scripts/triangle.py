#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math

class TriangleNode(Node):

    def __init__(self):
        super().__init__('triangle_node')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.state = 'move'
        self.side_count = 0
        self.side_length = 2.0
        self.turn_angle = 120
        self.angular_velocity = math.radians(self.turn_angle) / 1

    def timer_callback(self):
        msg = Twist()
        if self.state == 'move':
            msg.linear.x = self.side_length
            msg.angular.z = 0.0
        elif self.state == 'turn':
            msg.linear.x = 0.0
            msg.angular.z = self.angular_velocity
        self.publisher_.publish(msg)

        if self.state == 'move':
            self.state = 'turn'
        elif self.state == 'turn':
            self.side_count += 1
            self.state = 'move'

        if self.side_count >= 3:
            self.timer.cancel()
            self.get_logger().info("Done")

def main(args=None):
    rclpy.init(args=args)
    node = TriangleNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()