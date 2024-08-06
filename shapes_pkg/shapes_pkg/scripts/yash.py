#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

class StarDrawer(Node):

    def __init__(self):
        super().__init__("star_drawer")
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pose_subscriber = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.timer = self.create_timer(0.1, self.control_loop)  # Control loop at 10 Hz
        self.current_pose = Pose()
        self.state = 'move'
        self.side_count = 0
        self.side_length = 2.0
        self.turn_angle = 144
        self.angular_velocity = math.radians(self.turn_angle) / 2  # Adjust as needed
        self.linear_velocity = 2.0  # Adjust for suitable speed
        self.moving_to_position = False

    def pose_callback(self, msg):
        self.current_pose = msg
        self.get_logger().info(f'Pose received: x={msg.x}, y={msg.y}, theta={msg.theta}')

    def control_loop(self):
        msg = Twist()
        if self.state == 'move':
            if not self.moving_to_position:
                self.moving_to_position = True
                self.get_logger().info("Moving forward")

            distance_remaining = self.calculate_distance_to_target()
            if distance_remaining <= 0.1:  # Threshold for reaching the target distance
                self.state = 'turn'
                self.moving_to_position = False
                self.get_logger().info("Reached target distance, now turning")
            else:
                msg.linear.x = self.linear_velocity
                msg.angular.z = 0.0

        elif self.state == 'turn':
            target_angle = self.calculate_target_angle()
            if self.is_at_target_angle(target_angle):
                self.state = 'move'
                self.side_count += 1
                self.get_logger().info(f"Reached target angle, now moving ({self.side_count}/5)")
            else:
                msg.linear.x = 0.0
                msg.angular.z = self.angular_velocity

        if self.side_count >= 5:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.get_logger().info("Star completed")
            self.timer.cancel()

        self.publisher_.publish(msg)

    def calculate_distance_to_target(self):
        return self.side_length

    def calculate_distance_covered(self):
        return abs(self.current_pose.x)

    def calculate_target_angle(self):
        return self.current_pose.theta + math.radians(self.turn_angle)

    def is_at_target_angle(self, target_angle):
        return abs(self.current_pose.theta - target_angle) < 0.1

def main(args=None):
    rclpy.init(args=args)
    node = StarDrawer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
