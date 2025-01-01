#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point

class CoordinatePublisher(Node):
    def __init__(self):
        super().__init__('coordinate_publisher')
        self.publisher = self.create_publisher(Point, 'click_coords', 10)
        self.timer = self.create_timer(1.0, self.publish_coordinates)
        self.get_logger().info('Coordinate publisher node initialized')

    def publish_coordinates(self):
        msg = Point()
        msg.x = 0.15  # Example coordinates
        msg.y = 0.0
        msg.z = 0.03
        self.publisher.publish(msg)
        self.get_logger().info(f'Published coordinates: x={msg.x}, y={msg.y}, z={msg.z}')

def main(args=None):
    rclpy.init(args=args)
    coordinate_publisher = CoordinatePublisher()
    
    try:
        rclpy.spin(coordinate_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        coordinate_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
