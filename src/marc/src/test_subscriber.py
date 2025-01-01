#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import String

class CoordinateSubscriber(Node):
    def __init__(self):
        super().__init__('coordinate_subscriber')
        self.subscription = self.create_subscription(
            Point,
            'click_coords',
            self.listener_callback,
            10)
        self.cmd_subscription = self.create_subscription(
            String,
            'cmdpickplace',
            self.cmd_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.cmd_subscription  # prevent unused variable warning
        self.get_logger().info('Coordinate subscriber node initialized')

    def listener_callback(self, msg):
        self.get_logger().info(f'Received coordinates: x={msg.x}, y={msg.y}, z={msg.z}')

    def cmd_callback(self, msg):
        self.get_logger().info(f'Received command: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    coordinate_subscriber = CoordinateSubscriber()
    
    try:
        rclpy.spin(coordinate_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        coordinate_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
