from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import xacro


def generate_launch_description():
    pkg_dobot_description = get_package_share_directory('dobot_description')
    
    # Use sim time argument
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # --- Robot Description ---
    # NOTE: We use the standalone URDF, NOT the Gazebo one for RViz visualization
    robot_description_path = os.path.join(
        pkg_dobot_description, 'model', 'magician_standalone.urdf.xacro'
    )
    robot_description_config = Command(['xacro ', robot_description_path])
    
    # --- Nodes ---
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description_config
        }]
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
    )
    
    # --- RViz ---
    rviz_config_file = os.path.join(pkg_dobot_description, 'rviz', 'urdf_full.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation (Gazebo) clock if true'),
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])
