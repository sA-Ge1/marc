#!/usr/bin/env python3
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from pymoveit2 import MoveIt2
from pymoveit2.robots import marc as robot
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Point
from queue import Queue, Full
from threading import Thread
import time
from pose_goal import go_to_pose, create_moveit2_interface

QUEUE_SIZE = 5
state = False
def main():
    rclpy.init()
    node = Node("arm_pose_goal")
    coord_queue = Queue(maxsize=QUEUE_SIZE)
    def click_callback(msg):
        x_meters = round(msg.x / 100.0, 2)
        y_meters = round(msg.y / 100.0, 2)
        z_meters = round(msg.z / 100.0, 2)

        if coord_queue.full():
            node.get_logger().warn("Queue is full! Dropping incoming coordinate.")
            state = True
        else:
            coord_queue.put([x_meters, y_meters, z_meters])
            node.get_logger().info(
                f'Added to queue: x={x_meters}, y={y_meters}, z={z_meters}'
            )
    node.create_subscription(
        Point,
        '/key_xyz',
        click_callback,
        10
    )

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()

if __name__ == "__main__":
    main()
