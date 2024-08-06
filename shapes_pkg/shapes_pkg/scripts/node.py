#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class MoveTurtle(Node):
    def __init__(self):
        super().__init__('move_turtle_node')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.start_time = self.get_clock().now().seconds_nanoseconds()[0]
        self.moving_duration = 5.0  # seconds

    def timer_callback(self):
        current_time = self.get_clock().now().seconds_nanoseconds()[0]
        vel_msg = Twist()

        if current_time - self.start_time < self.moving_duration:
            vel_msg.linear.x = 1.0  # Move forward with 1.0 m/s
            vel_msg.angular.z = 0.0
        else:
            vel_msg.linear.x = 0.0  # Stop
            vel_msg.angular.z = 0.0

        self.publisher_.publish(vel_msg)

def main(args=None):
    rclpy.init(args=args)
    move_turtle = MoveTurtle()
    rclpy.spin(move_turtle)
    move_turtle.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
