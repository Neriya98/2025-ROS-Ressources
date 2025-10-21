#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class VelocityScaler(Node):
    def __init__(self):
        super().__init__('velocity_scaler')

        # Subscriber to the raw cmd_vel
        self.sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Publisher for the scaled cmd_vel
        self.pub = self.create_publisher(
            Twist,
            'x3/cmd_vel',
            10
        )

        self.declare_parameter('angular_scale', 10.0)
        self.angular_scale = self.get_parameter('angular_scale').get_parameter_value().double_value

        self.get_logger().info('✅ Velocity scaler node started')

    def cmd_vel_callback(self, msg: Twist):
        # Copy message
        scaled_msg = Twist()
        scaled_msg.linear = msg.linear
        scaled_msg.angular = msg.angular

        # Scale angular.z
        scaled_msg.angular.z = msg.angular.z * self.angular_scale

        # Publish
        self.pub.publish(scaled_msg)

def main(args=None):
    rclpy.init(args=args)
    node = VelocityScaler()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

