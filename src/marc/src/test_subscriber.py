#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
import numpy as np

class CoordSubscriber(Node):
    def __init__(self):
        super().__init__('coord_subscriber')
        
        # Create subscription to click coordinates
        self.coords_subscription = self.create_subscription(
            Point,
            'click_coords', 
            self.coords_callback,
            10)
            
        # Create publisher for joint states
        self.joint_state_publisher = self.create_publisher(
            JointState,
            'joint_states',
            10)
            
        self.get_logger().info('Coord subscriber node initialized')

    def coords_callback(self, msg):
        self.get_logger().info(f'Received coordinates: x={msg.x}, y={msg.y}, z={msg.z}')
        
        # Create and publish joint state message
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = ['joint1', 'joint2', 'joint3']  # Add joint names
        joint_state.position = [0.0, 0.0, 0.0]  # Add joint positions
        joint_state.velocity = []
        joint_state.effort = []
        
        self.joint_state_publisher.publish(joint_state)

def main(args=None):
    rclpy.init(args=args)
    coord_subscriber = CoordSubscriber()
    
    try:
        rclpy.spin(coord_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        coord_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
