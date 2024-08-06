#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist


class PubNode(Node):

    def __init__(self):
        super().__init__("PUB")
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 2)
        rate = 1
        self.timer = self.create_timer(rate, self.callback)
        self.state = 'move'
        self.side_count = 0
        

    def callback(self):
        msg = Twist()
        if self.state == 'move':
            msg.linear.x = 3.0
            msg.angular.z = 0.0
        elif self.state == 'turn':
            msg.linear.x = 0.0
            msg.angular.z = 1.542
        self.publisher_.publish(msg)

        if self.state == 'move':
            self.state = 'turn'
        elif self.state == 'turn':
            self.side_count+=1
            self.get_logger().info(str(self.side_count))
            self.state = 'move'

        if self.side_count >=4 :
            self.timer.cancel()
            self.get_logger().info(str(self.side_count))
            self.get_logger().info("Done")



def main(args=None):
    rclpy.init(args=args)
    node = PubNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()