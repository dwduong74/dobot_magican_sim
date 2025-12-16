from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, SetEnvironmentVariable, TimerAction, DeclareLaunchArgument
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro
import os


def generate_launch_description():
    pkg_dobot_description = get_package_share_directory('dobot_description')

    # --- Arguments ---
    use_sim_time = LaunchConfiguration('use_sim_time')

    # --- Gazebo Environment Setup ---
    pkg_share_parent = os.path.dirname(pkg_dobot_description)
    gazebo_model_path = pkg_share_parent
    if os.environ.get('GAZEBO_MODEL_PATH'):
        gazebo_model_path += ':' + os.environ['GAZEBO_MODEL_PATH']
    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=gazebo_model_path
    )

    ros_distro = os.environ.get('ROS_DISTRO', 'humble')
    ros_plugin_path = os.path.join('/opt/ros', ros_distro, 'lib')
    gazebo_plugin_path = ros_plugin_path
    if os.environ.get('GAZEBO_PLUGIN_PATH'):
        gazebo_plugin_path += ':' + os.environ['GAZEBO_PLUGIN_PATH']
    set_gazebo_plugin_path = SetEnvironmentVariable(
        name='GAZEBO_PLUGIN_PATH',
        value=gazebo_plugin_path
    )

    set_gazebo_model_database = SetEnvironmentVariable(
        name='GAZEBO_MODEL_DATABASE_URI',
        value=''
    )

    # --- Gazebo Simulation ---
    world = ""
    gzserver_cmd = ExecuteProcess(
        cmd=['gzserver', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'] + ([world] if world else []),
        output='screen'
    )
    gzclient_cmd = ExecuteProcess(cmd=['gzclient'], output='screen')

    # --- Robot Description ---
    robot_description_path = os.path.join(pkg_dobot_description, 'model', 'magician_standalone_gazebo.urdf.xacro')
    robot_description_config = xacro.process_file(robot_description_path)
    processed_urdf = robot_description_config.toxml()
    temp_urdf_path = "/tmp/dobot_magician.urdf"
    with open(temp_urdf_path, "w") as f:
        f.write(processed_urdf)

    # --- Nodes ---
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': processed_urdf
        }]
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-file', temp_urdf_path, '-entity', 'magician'],
        output='screen'
    )

    spawn_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    spawn_arm_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['magician_arm_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    spawn_gripper_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['magician_gripper_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )
    
    spawn_parallelogram_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['magician_parallelogram_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    kinematic_supervisor_node = Node(
        package='dobot_description',
        executable='kinematic_supervisor.py',
        output='screen'
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

    # --- Event Handlers ---
    delay_controllers_after_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[
                TimerAction(period=5.0, actions=[
                    spawn_joint_state_broadcaster,
                    spawn_arm_controller,
                    spawn_gripper_controller,
                    spawn_parallelogram_controller,
                ])
            ],
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation (Gazebo) clock if true'),
        
        set_gazebo_model_path,
        set_gazebo_plugin_path,
        set_gazebo_model_database,
        
        gzserver_cmd,
        gzclient_cmd,
        
        node_robot_state_publisher,
        spawn_entity,
        delay_controllers_after_spawn,
        
        kinematic_supervisor_node,
        rviz_node,
    ])
