#!/usr/bin/env python3
"""
Simple Inverse Kinematic and Forward Kinematic Demo for Cobot with MoveIt2.

Run this demo with an active moveit group (example: rviz_demo_launch.py).

Adapted from:
  https://github.com/moveit/moveit2_tutorials/blob/main/doc/examples/motion_planning_python_api/scripts/motion_planning_python_api_tutorial.py
"""

import time
import rclpy
from rclpy.logging import get_logger

# moveit python library
from moveit.planning import MoveItPy
from moveit.core.robot_state import RobotState
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory

# messages
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import Constraints, PositionConstraint


def plan_and_execute(
    robot,
    planning_component,
    logger,
    sleep_time=0.0,
):
    """Helper function to plan and execute a motion."""
    # plan to goal
    logger.info("Planning trajectory")
    plan_result = planning_component.plan()

    # execute the plan
    if plan_result:
        logger.info("Executing plan")
        robot_trajectory = plan_result.trajectory
        robot.execute(robot_trajectory, controllers=[])
    else:
        logger.error("Planning failed")

    time.sleep(sleep_time)


def main():

    ###################################################################
    # Create config required for MoveIt Python API
    ###################################################################
    rclpy.init()
    logger = get_logger("moveit_py.pose_goal")

    # the Python MoveIt2 API requires lots of configuration
    # we basically need most of the config files required to start the move group here again
    # it is also close to impossible to create a config that is accepted by MoveItPy
    # without the MoveItConfigsBuilder (hence, the config creation differs from the launch files)
    # TODO @rharbach: we need to pass the "real" controller param here somewhere for the urdf
    moveit_config = (
        MoveItConfigsBuilder(robot_name="cobot", package_name="cobot_model")
        .robot_description(
            file_path=get_package_share_directory("cobot_model")
            + "/urdf/festo_cobot_model.urdf.xacro"
        )
        .robot_description_semantic(
            file_path=get_package_share_directory("cobot_moveit_config")
            + "/config/festo_cobot_model.srdf"
        )
        .robot_description_kinematics(
            file_path=get_package_share_directory("cobot_moveit_config")
            + "/config/kinematics.yaml"
        )
        .joint_limits(
            file_path=get_package_share_directory("cobot_moveit_config")
            + "/config/joint_limits.yaml"
        )
        .trajectory_execution(
            file_path=get_package_share_directory("py_demo")
            + "/config/moveit_controllers.yaml"
        )
        .pilz_cartesian_limits(
            file_path=get_package_share_directory("py_demo")
            + "/config/pilz_cartesian_limits.yaml"
        )
        .moveit_cpp(
            file_path=get_package_share_directory("py_demo") + "/config/moveit_cpp.yaml"
        )
        .to_moveit_configs()
        .to_dict()
    )

    cobot = MoveItPy(
        node_name="moveit_py",
        config_dict=moveit_config,
    )
    cobot_arm = cobot.get_planning_component("arm_group")
    logger.info("MoveItPy instance created")

    ###########################################################################
    # Option 1: run IK
    ###########################################################################

    # set plan start state to current state
    cobot_arm.set_start_state_to_current_state()
    # define goal pose
    from geometry_msgs.msg import PoseStamped

    pose_goal = PoseStamped()
    pose_goal.header.frame_id = "base_link"
    pose_goal.pose.orientation.w = 1.0
    pose_goal.pose.position.x = 0.6
    pose_goal.pose.position.y = 0.3
    pose_goal.pose.position.z = 1.0

    # set a tolerance for the goal
    # this is required in order to relax the constraints on the planner
    # if not set, there will be hardly a solution found
    # this might be an issue with our cobot but could also be an issue
    # resulting from the usage of std param values for the planner config
    constraints = Constraints()
    position_constraint = PositionConstraint()
    position_constraint.header.frame_id = "base_link"
    position_constraint.link_name = "TCP"
    sphere = SolidPrimitive()
    sphere.type = SolidPrimitive.SPHERE
    sphere.dimensions = [0.05]  # 5 cm radius
    position_constraint.constraint_region.primitives.append(sphere)
    position_constraint.constraint_region.primitive_poses.append(pose_goal.pose)
    position_constraint.weight = 1.0
    constraints.position_constraints.append(position_constraint)
    # set the goal state with tolerance of 5cm
    cobot_arm.set_goal_state(motion_plan_constraints=[constraints])

    # plan to goal
    plan_and_execute(cobot, cobot_arm, logger, sleep_time=2.0)

    ###########################################################################
    # Option 2: run FK
    ###########################################################################

    # set plan start state to current state
    cobot_arm.set_start_state_to_current_state()
    robot_model = cobot.get_robot_model()
    robot_state = RobotState(robot_model)

    # applying FK is handled using joint constraints
    # note: unlike IK, running FK almost always works as long as joint limits are
    # respected. Refer file cobot_moveit_config/config/joint_limits.yaml
    from moveit.core.kinematic_constraints import construct_joint_constraint

    # move to stable position
    joint_values = {
        "joint_0": 0.155,
        "joint_1": -1.57,
        "joint_2": 0.610865,
        "joint_3": -0.785398,
        "joint_4": 1.57,
        "joint_5": -1.57,
        "joint_6": -0.2,
    }
    robot_state.joint_positions = joint_values
    joint_constraint = construct_joint_constraint(
        robot_state=robot_state,
        joint_model_group=cobot.get_robot_model().get_joint_model_group("arm_group"),
    )
    cobot_arm.set_goal_state(motion_plan_constraints=[joint_constraint])

    # plan to goal
    plan_and_execute(cobot, cobot_arm, logger, sleep_time=3.0)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
