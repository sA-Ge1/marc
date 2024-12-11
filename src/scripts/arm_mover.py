import rclpy
from pymoveit2 import MoveIt2
from rclpy.node import Node
import numpy as np

class ArmController(Node):
    def __init__(self):
        super().__init__('arm_joint_controller')
        
        # Joint configuration
        self.joint_names = [
            "shoulder_joint",
            "upper_arm_joint", 
            "elbow_joint", 
            "wrist_joint", 
            "ee_joint", 
            "ee_rotation_joint_x",
            "ee_rotation_joint_y", 
            "ee_rotation_joint_z"
        ]
        
        # Initialize MoveIt2
        self.moveit2 = MoveIt2(
            node=self,
            joint_names=self.joint_names,
            base_link_name='base_link', 
            end_effector_name='robot_ee'
        )

    def move_joints(self, joint_positions):
        """Move to specified joint positions"""
        self.moveit2.move_to_configuration(joint_positions)

def main(args=None):
    rclpy.init(args=args)
    node = ArmController()
    
    try:
        # Example joint targets
        target_joints = [
            2.0,    # shoulder_joint
            0.0,  # upper_arm_joint 
            0.0,  # elbow_joint
            0.0,    # wrist_joint
            0.0,    # ee_joint
            0.0,    # ee_rotation_joint_x
            0.0,    # ee_rotation_joint_y
            0.0     # ee_rotation_joint_z
        ]
        
        node.move_joints(target_joints)
        rclpy.spin(node)
    
    except Exception as e:
        node.get_logger().error(f'Error: {e}')
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()