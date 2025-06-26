import os
from os import path
from launch_ros.actions import Node
from launch import LaunchDescription
from py_utils.launch_utils import load_file
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # define package with configuration
    moveit_config_package = "cobot_moveit_config"

    # get URDF model (from xacro) with "fake" controllers
    robot_description_as_string = load_file(
        "cobot_model",
        path.join("urdf", "festo_cobot_model.urdf.xacro"),
        mappings={"ros2_controls_plugin": "fake"},
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
    joint_limits_yaml = load_file(
        moveit_config_package, path.join("config", "joint_limits.yaml")
    )
    joint_limits = {"robot_description_planning": joint_limits_yaml}

    # define planning pipeline (loading defaults from OMPL yaml)
    planning_pipeline = {
        "planning_pipelines": ["ompl"],
        "default_planning_pipeline": "ompl",
    }
    planning_pipeline["ompl"] = load_file(
        moveit_config_package, path.join("config", "ompl_planning_conf.yaml")
    )

    node = Node(
        package="moveit_task_constructor_demo",
        executable="pick_place_demo",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            joint_limits,
            planning_pipeline,
            os.path.join(
                get_package_share_directory("demo"), "config", "pick_place_config.yaml"
            ),
        ],
    )

    return LaunchDescription([node])
