import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    # get package path
    pkg_description_path = get_package_share_directory('mycobot_description')

    # get file path
    urdf_model_path = os.path.join(pkg_description_path, 'urdf', 'mycobot_280_urdf.xacro')

    # Publish the joint state values for the non-fixed joints in the URDF file.
    # start_joint_state_publisher_cmd = Node(
    #     package='joint_state_publisher',
    #     executable='joint_state_publisher'
    # )

    # Depending on gui parameter, either launch joint_state_publisher or joint_state_publisher_gui
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui'
    )

    # Subscribe to the joint states of the robot, and publish the 3D pose of each link.
    robot_description_content = ParameterValue(Command(['xacro ', urdf_model_path]), value_type=str)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description_content}]
    )
      
    rviz2 = Node(
        package='rviz2',
        executable='rviz2'
    )
  
    # Create the launch description and populate
    return LaunchDescription([
        joint_state_publisher_gui,
        robot_state_publisher,
        rviz2
    ])
