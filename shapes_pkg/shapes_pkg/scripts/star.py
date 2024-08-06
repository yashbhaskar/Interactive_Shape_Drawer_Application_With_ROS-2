#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math


class PubNode(Node):

    def __init__(self):
        super().__init__("PUB")
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 20)
        rate = 1
        self.timer = self.create_timer(rate, self.callback)
        self.state = 'move'
        self.side_count = 0
        self.side_length = 2.0
        self.turn_angle = 144
        self.angular_velocity = math.radians(self.turn_angle) / 1
        self.side_lengths = [self.side_length] * 5

    def callback(self):
        msg = Twist()
        if self.state == 'move':
            msg.linear.x = self.side_lengths[self.side_count]
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

        if self.side_count >= 5:
            self.timer.cancel()
            self.get_logger().info("Done")


def main(args=None):
    rclpy.init(args=args)
    node = PubNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()