#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Float64MultiArray
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout, QLabel,
    QSlider, QPushButton, QGroupBox
)
from PyQt5.QtCore import Qt
import math

class DobotGUI(Node):
    def __init__(self):
        super().__init__('dobot_gui_control_panel')

        self.arm_pub = self.create_publisher(
            JointTrajectory,
            '/magician_arm_controller/joint_trajectory',
            10
        )
        self.gripper_pub = self.create_publisher(
            JointTrajectory,
            '/magician_gripper_controller/joint_trajectory',
            10
        )
        self.joint_positions = [0.0, 0.0, 0.0, 0.0]
        self.get_logger().info('Dobot GUI Control Panel has been started.')

    def send_arm_positions(self):
        msg = JointTrajectory()
        msg.joint_names = [
            'magician_joint_1',
            'magician_joint_2',
            'magician_joint_3',
            'magician_joint_4'
        ]
        point = JointTrajectoryPoint()
        point.positions = self.joint_positions
        # Make the movement faster
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 500 * 1000 * 1000 # 0.5 seconds
        msg.points.append(point)
        self.arm_pub.publish(msg)
        self.get_logger().info(f'Sending arm positions: {self.joint_positions}', throttle_duration_sec=1.0)

    def send_gripper_position(self, pos):
        msg = JointTrajectory()
        msg.joint_names = ['magician_joint_prismatic_l']
        point = JointTrajectoryPoint()
        point.positions = [pos]
        point.time_from_start.sec = 1
        msg.points.append(point)
        self.gripper_pub.publish(msg)
        self.get_logger().info(f"Sending gripper position: {pos}")


class GUI(QWidget):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.setWindowTitle("Dobot Magician Control Panel")
        self.setGeometry(100, 100, 400, 500)

        layout = QVBoxLayout()

        # ===== ARM JOINTS (with correct limits) =====
        arm_box = QGroupBox("Arm Joints (Position)")
        arm_layout = QVBoxLayout()
        
        # URDF Limits (degrees)
        # j1: -125 to 125
        # j2: -5 to 90
        # j3: -50 to 60 (our custom value)
        # j4: -150 to 150
        limits_deg = [(-125, 125), (-5, 90), (-60, 70), (-150, 150)]
        self.arm_sliders = []

        for i in range(4):
            min_deg, max_deg = limits_deg[i]
            # Convert degrees to rad*100 for slider scale
            min_val = int(min_deg * math.pi / 180 * 100)
            max_val = int(max_deg * math.pi / 180 * 100)
            
            label = QLabel(f"Joint {i+1}: 0.00 rad")
            slider = QSlider(Qt.Horizontal)
            slider.setMinimum(min_val)
            slider.setMaximum(max_val)
            slider.setValue(0)
            slider.valueChanged.connect(lambda val, i=i, lbl=label: self.update_arm_positions(i, val, lbl))

            self.arm_sliders.append(slider)
            arm_layout.addWidget(label)
            arm_layout.addWidget(slider)

        arm_box.setLayout(arm_layout)
        layout.addWidget(arm_box)

        # ===== GRIPPER (Position Control) =====
        grip_box = QGroupBox("Gripper")
        grip_layout = QHBoxLayout()

        open_btn = QPushButton("Open")
        close_btn = QPushButton("Close")

        open_btn.clicked.connect(lambda: self.node.send_gripper_position(0.013))
        close_btn.clicked.connect(lambda: self.node.send_gripper_position(0.0))

        grip_layout.addWidget(open_btn)
        grip_layout.addWidget(close_btn)
        grip_box.setLayout(grip_layout)
        layout.addWidget(grip_box)

        self.setLayout(layout)

    def update_arm_positions(self, index, value, label):
        pos = value / 100.0
        label.setText(f"Joint {index+1}: {pos:.2f} rad")
        self.node.joint_positions[index] = pos
        self.node.send_arm_positions()


def main():
    rclpy.init()
    app = QApplication(sys.argv)
    ros_node = DobotGUI()
    gui = GUI(ros_node)
    gui.show()

    while rclpy.ok() and gui.isVisible():
        rclpy.spin_once(ros_node, timeout_sec=0.01)
        app.processEvents()

    ros_node.destroy_node()
    rclpy.shutdown()
    sys.exit()


if __name__ == '__main__':
    main()