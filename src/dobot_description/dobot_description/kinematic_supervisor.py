#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import math

class KinematicSupervisor(Node):
    def __init__(self):
        super().__init__('kinematic_supervisor')
        
        self.master_joint_names = ['magician_joint_2', 'magician_joint_3']
        self.mimic_joint_names = ['magician_joint_mimic_1', 'magician_joint_mimic_2']
        
        self.joint_indices = {}
        self.initialized = False

        self.publisher_ = self.create_publisher(
            Float64MultiArray,
            '/magician_parallelogram_controller/commands',
            10)
            
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_states_callback,
            10)
            
        self.get_logger().info('Kinematic Supervisor has started.')

    def joint_states_callback(self, msg):
        if not self.initialized:
            try:
                all_needed_joints = self.master_joint_names + self.mimic_joint_names
                for name in all_needed_joints:
                    self.joint_indices[name] = msg.name.index(name)
                self.initialized = True
                self.get_logger().info(f'Supervisor initialized. Found all required joints.')
            except ValueError as e:
                if not self.initialized:
                    self.get_logger().error(f'Could not find joint in /joint_states: {e}. Retrying...')
                return

        pos_joint_2 = msg.position[self.joint_indices['magician_joint_2']]
        pos_joint_3 = msg.position[self.joint_indices['magician_joint_3']]

        target_mimic_1 = -pos_joint_2
        target_mimic_2 = -pos_joint_3

        cmd_msg = Float64MultiArray()
        cmd_msg.data = [target_mimic_1, target_mimic_2]
        self.publisher_.publish(cmd_msg)

def main(args=None):
    rclpy.init(args=args)
    supervisor_node = KinematicSupervisor()
    rclpy.spin(supervisor_node)
    supervisor_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
