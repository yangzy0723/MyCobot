import os
import xacro

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    pkg_gazebo_path = get_package_share_directory('mycobot_gazebo')
    pkg_moveit_path = get_package_share_directory('mycobot_moveit')

    urdf_file_path = os.path.join(pkg_gazebo_path, 'urdf', 'mycobot_280.urdf.xacro')
    rviz_file_path = os.path.join(pkg_moveit_path, 'rviz', 'mycobot.rviz')

    moveit_controllers_file_path = os.path.join(
        pkg_moveit_path, 'config', 'moveit_controllers.yaml'
    )
    joint_limits_file_path = os.path.join(
        pkg_moveit_path, 'config', 'joint_limits.yaml'
    )
    srdf_file_path = os.path.join(
        pkg_moveit_path, 'config', 'mycobot_280.srdf'
    )
    kinematics_file_path = os.path.join(
        pkg_moveit_path, 'config', 'kinematics.yaml'
    )
    pilz_cartesian_limits_file_path = os.path.join(
        pkg_moveit_path, 'config', 'pilz_cartesian_limits.yaml'
    )
    initial_positions_file_path = os.path.join(
        pkg_moveit_path, 'config', 'initial_positions.yaml'
    )

    moveit_config = (
        MoveItConfigsBuilder(
            "mycobot_280", package_name='mycobot_moveit'
        )
        .joint_limits(
            file_path='config/joint_limits.yaml'
        )
        .robot_description_kinematics(
            file_path='config/kinematics.yaml'
        )
        .pilz_cartesian_limits(
            file_path='config/pilz_cartesian_limits.yaml'
        )
        # .robot_description(
        #     file_path='urdf/mycobot_280.urdf.xacro'
        # )
        .robot_description_semantic(
            file_path='config/mycobot_280.srdf'
        )
        .trajectory_execution(
            file_path='config/moveit_controllers.yaml'
        )
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
        .planning_pipelines(
            # pipelines=["ompl", "pilz_industrial_motion_planner", "stomp"],
            pipelines=["ompl"]
        )
        .to_moveit_configs()
    )
    
    # Start the actual move_group node/action server
    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {'use_sim_time': True},
            {'start_state': {'content': initial_positions_file_path}},
        ],
    )

    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
        arguments=["-d", rviz_file_path],
    )

    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["--frame-id", "world", "--child-frame-id", "base_link"],
    )

    return LaunchDescription([
        move_group,
        rviz2,
        static_tf,
    ])
