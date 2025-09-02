"""
Configure and launch nodes related to MoveIt2 fake controls.
List of nodes to be launched:
 * move_group
 * robot_state_publisher
 * ros2_control_node
 * rviz2
 * controller_manager spawning the controllers

Like most launch-files created for ROS2 and Gazebo,
we use XACRO for the creation of the robot description
(conversion from XACRO to URDF is handled on demand).
"""

from os import path
from typing import List

from py_utils.launch_utils import load_file

from launch.actions import (
    OpaqueFunction,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.substitutions import (
    LaunchConfiguration,
    PythonExpression,
    PathJoinSubstitution,
)

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description() -> LaunchDescription:
    """Creates the launch description for the Moveit2 / Gazebo example."""

    # declare all launch arguments
    declared_arguments = _generate_declared_arguments()
    # run _setup_nodes as opaque function that hands over the context
    # this is required in order to substitute launch arguments directly
    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=_setup_nodes)]
    )


def _setup_nodes(context, *args, **kwargs):
    """Set up nodes to be launched."""

    rviz_config = LaunchConfiguration("rviz_config")
    use_sim_time = LaunchConfiguration("use_sim_time")
    log_level = LaunchConfiguration("log_level")
    enable_realsense_camera = LaunchConfiguration("enable_realsense_camera").perform(
        context
    )
    use_collision_meshes = LaunchConfiguration("use_collision_meshes").perform(context)
    controller_type = LaunchConfiguration("controller_type").perform(context)

    # define package with configuration
    moveit_config_package = "cobot_moveit_config"

    # get URDF model (from xacro) with "fake" controllers
    robot_description_as_string = load_file(
        "cobot_model",
        path.join("urdf", "festo_cobot_model.urdf.xacro"),
        mappings={
            "ros2_controls_plugin": controller_type,
            "enable_realsense_camera": enable_realsense_camera,
            "use_collision_meshes": use_collision_meshes,
        },
    )
    robot_description = {"robot_description": robot_description_as_string}

    # get SRDF model
    robot_description_semantic_srdf = load_file(
        moveit_config_package, path.join("config", "festo_cobot_model.srdf")
    )
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_srdf
    }
    # get kinematics
    robot_description_kinematics_yaml = load_file(
        moveit_config_package, path.join("config", "kinematics.yaml")
    )
    robot_description_kinematics = {
        "robot_description_kinematics": robot_description_kinematics_yaml
    }

    # get joint limits
    joint_limits = {
        "robot_description_planning": load_file(
            moveit_config_package, path.join("config", "joint_limits.yaml")
        )
    }
    # define planning pipeline (loading defaults from OMPL yaml)
    planning_pipeline = {
        "planning_pipelines": ["ompl"],
        "default_planning_pipeline": "ompl",
    }
    planning_pipeline["ompl"] = load_file(
        moveit_config_package, path.join("config", "ompl_planning.yaml")
    )
    # deine params for planning scene
    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "publish_robot_description": True,
        "publish_robot_description_semantic": True,
    }

    # load controller definition for the MoveIt controller manager
    moveit_controllers = (
        "moveit_controllers.yaml"
        if controller_type != "real"
        else "moveit_custom_controllers.yaml"
    )

    moveit_controllers_yaml = load_file(
        moveit_config_package, path.join("config", moveit_controllers)
    )

    # ros2_control using FakeSystem as hardware
    ros2_controllers_path = path.join(
        get_package_share_directory(moveit_config_package),
        "config",
        "ros2_controllers.yaml",
    )

    # load config for occupancy map
    sensors_yaml = (
        load_file(moveit_config_package, path.join("config", "sensor_3d.yaml"))
        if enable_realsense_camera
        else {"sensors": []}  # create empty config if camera is not running
    )

    # configure trajectory execution
    trajectory_execution = {
        "allow_trajectory_execution": True,
        "moveit_manage_controllers": True,
        "execution_duration_monitoring": False,
        "trajectory_execution.allowed_execution_duration_scaling": 1.5,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }

    # Load  ExecuteTaskSolutionCapability so we can execute found solutions in simulation
    # required for the MoveIt Task Contructor demo
    move_group_capabilities = {
        "capabilities": "move_group/ExecuteTaskSolutionCapability"
    }

    # define list of nodes to be launched
    nodes = [
        # robot_state_publisher
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="log",
            arguments=["--ros-args", "--log-level", log_level],
            parameters=[
                robot_description,
                {
                    "publish_frequency": 100.0,
                    "use_sim_time": use_sim_time,
                },
            ],
        ),
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[robot_description, ros2_controllers_path],
            output="both",
        ),
        # move_group
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="log",
            arguments=["--ros-args", "--log-level", log_level],
            parameters=[
                robot_description,
                robot_description_semantic,
                robot_description_kinematics,
                joint_limits,
                planning_pipeline,
                trajectory_execution,
                planning_scene_monitor_parameters,
                moveit_controllers_yaml,
                {"use_sim_time": use_sim_time},
                move_group_capabilities,
                sensors_yaml,
            ],
        ),
        # rviz2
        Node(
            package="rviz2",
            executable="rviz2",
            output="log",
            arguments=[
                "--display-config",
                rviz_config,
                "--ros-args",
                "--log-level",
                log_level,
            ],
            parameters=[
                robot_description,
                robot_description_semantic,
                robot_description_kinematics,
                planning_pipeline,
                joint_limits,
                {"use_sim_time": use_sim_time},
            ],
        ),
        ### start controllers ###
        # arm group controller (with fake controls)
        Node(
            package="controller_manager",
            executable="spawner",
            output="log",
            arguments=[
                "arm_group_controller",
                "--ros-args",
                "--log-level",
                log_level,
            ],
            parameters=[{"use_sim_time": use_sim_time}],
            condition=(
                IfCondition(
                    PythonExpression(
                        [
                            "'",
                            controller_type,
                            "'",
                            " == ",
                            "'fake'",
                        ]
                    )
                )
            ),
        ),
        # cobot arm group controller (for real cobot)
        Node(
            package="controller_manager",
            executable="spawner",
            output="log",
            arguments=[
                "cobot_arm_group_controller",
                "--ros-args",
                "--log-level",
                log_level,
            ],
            parameters=[{"use_sim_time": use_sim_time}],
            condition=(
                IfCondition(
                    PythonExpression(
                        [
                            "'",
                            controller_type,
                            "'",
                            " == ",
                            "'real'",
                        ]
                    )
                )
            ),
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            output="log",
            arguments=[
                "gripper_group_controller",
                "--ros-args",
                "--log-level",
                log_level,
            ],
            parameters=[{"use_sim_time": use_sim_time}],
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            output="log",
            arguments=[
                "vacuum_upper_joint_controller",
                "--ros-args",
                "--log-level",
                log_level,
            ],
            parameters=[{"use_sim_time": use_sim_time}],
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            output="log",
            arguments=[
                "vacuum_lower_joint_controller",
                "--ros-args",
                "--log-level",
                log_level,
            ],
            parameters=[{"use_sim_time": use_sim_time}],
        ),
        # run joint state broadcaster (so we can see the current robot state in rviz)
        Node(
            package="controller_manager",
            executable="spawner",
            output="log",
            arguments=[
                "joint_state_broadcaster",
                "--ros-args",
                "--log-level",
                log_level,
            ],
            parameters=[{"use_sim_time": use_sim_time}],
        ),
    ]

    # add external launch-files
    nodes.append(
        # include realsense node if required
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [
                        FindPackageShare("realsense2_camera"),
                        "launch",
                        "rs_launch.py",
                    ]
                )
            ),
            launch_arguments={
                "pointcloud.enable": "true",
                "clip_distance": "1.3",
            }.items(),
            condition=(
                IfCondition(
                    PythonExpression(
                        [
                            "'",
                            enable_realsense_camera,
                            "'",
                            " == ",
                            "'true'",
                        ]
                    )
                )
            ),
        ),
    )

    return nodes


def _generate_declared_arguments() -> List[DeclareLaunchArgument]:
    """
    Generate list of all launch arguments that are declared for this launch script.

    Returns:
        List of launch arguments
    """

    return [
        DeclareLaunchArgument(
            "rviz_config",
            default_value=path.join(
                get_package_share_directory("demo"),
                "rviz",
                "demo.rviz",
            ),
            description="Path to configuration for RViz2.",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="If true, use simulated clock.",
        ),
        DeclareLaunchArgument(
            "log_level",
            default_value="warn",
            description="The level of logging that is applied to all ROS 2 nodes launched by this script.",
        ),
        DeclareLaunchArgument(
            "enable_realsense_camera",
            default_value="false",
            description="Activate RealSense Node.",
        ),
        DeclareLaunchArgument(
            "use_collision_meshes",
            default_value="true",
            description="Use meshes for collisions. If false, using simple geometric objects.",
        ),
        DeclareLaunchArgument(
            "controller_type",
            default_value="fake",
            description="Define controllers: fake or real.",
        ),
    ]
