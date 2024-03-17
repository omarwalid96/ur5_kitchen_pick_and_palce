#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

import math

class JointStateListener(Node):

    def __init__(self):
        super().__init__('joint_state_listener')
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',  # Adjust the topic name if needed
            self.joint_state_callback,
            10)

        self.subscription = self.create_subscription(
            Bool,
            'save_points',  # Adjust the topic name if needed
            self.save_cb,
            10)
        self.subscription  # prevent unused variable warning
        self.joint_positions_degrees=[]
        self.file_path = 'joint_positions.txt' 

    def joint_state_callback(self, msg):
        if len(msg.position) > 0:
            self.joint_positions_degrees = [math.degrees(pos) for pos in msg.position]
            self.get_logger().info('Joint Positions (degrees): %s' % self.joint_positions_degrees)

    def save_cb(self, msg):
        if msg.data:
            string_value="{'action': 'MoveJs', 'value': {'joint1': "+str(self.joint_positions_degrees[1]) +", 'joint2': "+str(self.joint_positions_degrees[2])+", 'joint3': "+str(self.joint_positions_degrees[4])+", 'joint4': "+str(self.joint_positions_degrees[5])+", 'joint5':"+ str(self.joint_positions_degrees[6])+", 'joint6': "+ str(self.joint_positions_degrees[7])+", 'joint7': "+ str(self.joint_positions_degrees[8])+"}, 'speed': 1.0}"

            with open(self.file_path, 'a') as file:
                file.write(string_value+ '\n')


def main(args=None):
    rclpy.init(args=args)
    joint_state_listener = JointStateListener()
    rclpy.spin(joint_state_listener)
    joint_state_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
