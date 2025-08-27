#!/usr/bin/env python3
"""
Simple Inverse Kinematic and Forward Kinematic Demo for Cobot with MoveIt2.

Run this demo with an active moveit group (example: rviz_demo_launch.py).

Adapted from:
  https://github.com/moveit/moveit2_tutorials/blob/main/doc/examples/motion_planning_python_api/scripts/motion_planning_python_api_tutorial.py
"""

import rclpy
from rclpy.logging import get_logger

from py_utils.planner_utils import (
    plan_and_execute,
    wait_for_joint_states,
    generate_moveit_config,
)
from py_utils.visualization_utils import FootballMarkerPublisher


# moveit python library
from moveit.planning import (
    MoveItPy,
    MultiPipelinePlanRequestParameters,  # not used (enables running different requests in parallel)
    PlanRequestParameters,
)

from moveit.core.robot_state import RobotState


def main():

    ###################################################################
    # Create config required for MoveIt Python API
    ###################################################################
    rclpy.init()
    logger = get_logger("moveit_py.pose_goal")
    # the Python MoveIt2 API requires configuration
    moveit_config = generate_moveit_config()
    # wait until the joint states are published
    wait_for_joint_states(logger)

    cobot = MoveItPy(
        node_name="moveit_py",
        config_dict=moveit_config,
    )
    cobot_arm = cobot.get_planning_component("arm_group")
    logger.info("MoveItPy instance created")
    cobot_arm.set_workspace(-1.0, -0.2, 0.0, 1.0, 1.0, 1.2)

    ###########################################################################
    # Option 1: run IK
    ###########################################################################

    # set plan start state to current state
    cobot_arm.set_start_state_to_current_state()

    # define end effector: plan in world frame orientation
    eef = "gripper_tcp_world"
    """
    Note: we define several TCP frames for the grippers to avoid rotations in code.
          *_tcp: orientation in cobot frame (this frame is rotated around x and y)
          *_tcp_world: all rotations are removed and orientation is aligned with the base_link
          *_tcp_world_tilted_up: compensating the natural 45 deg tilt of Cobot's TCP
                                 => this results in the Cobot approaching neutral oriented
                                    objects from above
          *_tcp_world_tilted_up: same as *_tcp_world_tilted_up but with z upside down (z+ down)
                                 => this results in the Cobot approaching neutral oriented
                                    objects from below

        A neutral orientation is defined as x+: forward, y+: left, z+ up (representing the right hand rule).
        => quaternion: xyzw = (0.0, 0.0, 0.0, 1.0)

        In case the object placed does not have a neutral orientation but is oriented with z+ down
        (=> x+: forward, y+: left, z+ down, quaternion xyzw = (0.0, 1.0, 0.0, 0.0))
        we can use the gripper_tcp_world_tilted_down to compensate the rotation around y.
    """
    # define goal pose
    from geometry_msgs.msg import PoseStamped

    pose_goal = PoseStamped()
    pose_goal.header.frame_id = "base_link"
    pose_goal.pose.orientation.w = 1.0  # neutral orientation
    pose_goal.pose.position.x = 0.6
    pose_goal.pose.position.y = -0.3
    pose_goal.pose.position.z = 1.0
    # set up marker to display pose_goal
    # => subscribe to /football_marker_topic in rviz to visualize the marker
    football_marker = FootballMarkerPublisher()
    football_marker.publish_marker(pose_goal)
    # set goal with preferred TCP frame
    cobot_arm.set_goal_state(pose_stamped_msg=pose_goal, pose_link=eef)
    # reference parameters defined in config/moveit_cpp.yaml
    single_plan_request_parameters = PlanRequestParameters(cobot, "ompl_rrtc")
    # define preferred planner: default is SBLkConfigDefault
    # single_plan_request_parameters.planner_id = "SBLkConfigDefault"
    # speed up simulation
    single_plan_request_parameters.max_velocity_scaling_factor = 1.0
    single_plan_request_parameters.max_acceleration_scaling_factor = 1.0

    # plan to goal
    plan_and_execute(
        cobot,
        cobot_arm,
        logger,
        single_plan_parameters=single_plan_request_parameters,
        sleep_time=3.0,
    )

    # change end effector: plan in tilted world frame
    eef = "gripper_tcp_world_tilted_up"
    cobot_arm.set_goal_state(pose_stamped_msg=pose_goal, pose_link=eef)
    plan_and_execute(
        cobot,
        cobot_arm,
        logger,
        single_plan_parameters=single_plan_request_parameters,
        sleep_time=3.0,
    )

    pose_goal.pose.orientation.w = 0.0
    pose_goal.pose.orientation.y = 1.0  # rotate 180 deg around y => z+ is down
    pose_goal.pose.position.y = -0.4  # move pose goal to the right to see change
    football_marker.publish_marker(pose_goal)
    # use TCP for objects oriented with z+ down
    eef = "gripper_tcp_world_tilted_down"
    cobot_arm.set_goal_state(pose_stamped_msg=pose_goal, pose_link=eef)
    plan_and_execute(
        cobot,
        cobot_arm,
        logger,
        single_plan_parameters=single_plan_request_parameters,
        sleep_time=3.0,
    )

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
