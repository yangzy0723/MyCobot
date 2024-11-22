import os
import xacro

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

  pkg_gazebo_path = get_package_share_directory('mycobot_gazebo')
  pkg_ros_gz_sim_path = get_package_share_directory('ros_gz_sim')

  urdf_model_path = os.path.join(pkg_gazebo_path, 'urdf', 'mycobot_280.urdf.xacro')
  bridge_path = os.path.join(pkg_gazebo_path, 'config', 'ros_gz_bridge.yaml')
  world_path = os.path.join(pkg_gazebo_path, 'worlds', 'empty.world')
  models_path = os.path.join(pkg_gazebo_path, 'worlds')
  controllers_path = os.path.join(pkg_gazebo_path, 'config', 'ros2_controllers.yaml')

  doc = xacro.parse(open(urdf_model_path))
  xacro.process_doc(doc)
  robot_state_publisher = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    parameters=[{'use_sim_time': True, 'robot_description': doc.toxml()}]
  )
  
  set_env_vars_resources = AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', models_path)
  gz_sim = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(os.path.join(pkg_ros_gz_sim_path, 'launch', 'gz_sim.launch.py')),
      launch_arguments=[('gz_args', [' -r -v 4 ', world_path])]
  )
  
  # # Start Gazebo server
  # gazebo_server = IncludeLaunchDescription(
  #   PythonLaunchDescriptionSource(os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
  #   launch_arguments={'gz_args': ['-r -s -v4 ', world], 'on_exit_shutdown': 'true'}.items()
  # )

  # # Start Gazebo client    
  # start_gazebo_client_cmd = IncludeLaunchDescription(
  #   PythonLaunchDescriptionSource(os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
  #   launch_arguments={'gz_args': '-g -v4 '}.items()
  # )

  rviz2 = Node(
    package='rviz2',
    executable='rviz2'
  )  
    
  ros_gz_spawner = Node(
    package='ros_gz_sim',
    executable='create',
    arguments=[
      '-topic', 'robot_description',
      '-x', '0.0',
      '-y', '0.0',
      '-z', '0.05',
      '-R', '0.0',
      '-P', '0.0',
      '-Y', '0.0'
    ]
  )  
    
  ros_gz_bridge = Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    parameters=[{'config_file': bridge_path}]
  )

  ros2_control = Node(
      package="controller_manager",
      executable="ros2_control_node",
      parameters=[controllers_path],
  )

  joint_state_broadcaster = Node(
      package='controller_manager',
      executable='spawner',
      arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager']
  )

  arm_controller = Node(
      package='controller_manager',
      executable='spawner',
      arguments=['arm_controller', '--controller-manager', '/controller_manager']
  )

  gripper_action_controller = Node(
      package='controller_manager',
      executable='spawner',
      arguments=['grip_action_controller', '--controller-manager', '/controller_manager']
  )

  gripper_controller = Node(
      package='controller_manager',
      executable='spawner',
      arguments=['grip_controller', '--controller-manager', '/controller_manager']
  )

  # 用于顺序启动一系列ROS2控制器
  joint_state_broadcaster_handler = RegisterEventHandler(
     event_handler=OnProcessExit(
     target_action=ros_gz_spawner,
     on_exit=[joint_state_broadcaster],))

  arm_controller_handler = RegisterEventHandler(
    event_handler=OnProcessExit(
    target_action=joint_state_broadcaster,
    on_exit=[arm_controller],))

  gripper_action_controller_handler = RegisterEventHandler(
    event_handler=OnProcessExit(
    target_action=arm_controller,
    on_exit=[gripper_action_controller],))
    
  gripper_controller_handler = RegisterEventHandler(
    event_handler=OnProcessExit(
    target_action=gripper_controller,
    on_exit=[gripper_controller],))     
    
  return LaunchDescription([
    joint_state_broadcaster_handler,
    arm_controller_handler,
    gripper_controller_handler,
    gripper_action_controller_handler,

    robot_state_publisher,
    # ros2_control,
    # rviz2,
    gz_sim,
    ros_gz_spawner,
    ros_gz_bridge,
    set_env_vars_resources
  ])
