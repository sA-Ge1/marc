import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import serial
import time
import math

class ArmController(Node):
    def __init__(self):
        super().__init__('arm_controller_serial')
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10)
        
        # Initialize serial communication (adjust port and baudrate as necessary)
        self.serial_port = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)  # Update port based on your setup
        time.sleep(2)  # Give the serial connection time to initialize
        
        # Last sent positions for each joint, initialized to None
        self.last_sent_positions = {
            'shoulder_joint': None,
            'upper_arm_joint': None,
            'elbow_joint': None,
            'wrist_joint': None
        }
        self.threshold = 0.5 # Degree change threshold

    def joint_state_callback(self, msg):
        joint_data = []
        for name, position in zip(msg.name, msg.position):
            # Convert position (radians) to degrees
            position_deg = math.degrees(position)
            
            # Check if the joint is in the list and if it exceeds the threshold
            if name in self.last_sent_positions:
                last_position = self.last_sent_positions[name]
                if last_position is None or abs(position_deg - last_position) >= self.threshold:
                    # Update last sent position and add to data to send
                    self.last_sent_positions[name] = position_deg
                    joint_data.append(f"{name}:{position_deg:.2f}")

        # Send data only if there are joints with updated positions
        if joint_data:
            data_str = ";".join(joint_data)
            self.serial_port.write((data_str + "\n").encode())
            self.get_logger().info(f"Sent joint data: {data_str}")

def main(args=None):
    rclpy.init(args=args)
    arm_controller = ArmController()
    rclpy.spin(arm_controller)
    arm_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
