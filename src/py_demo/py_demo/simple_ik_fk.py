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
from moveit.planning import (
    MoveItPy,
    MultiPipelinePlanRequestParameters,  # not used (enables running different requests in parallel)
    PlanRequestParameters,
)

from rclpy.node import Node
from visualization_msgs.msg import Marker
from moveit.core.robot_state import RobotState
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory


class FootballMarkerPublisher(Node):
    """Marker publisher for displaying the target pose."""

    def __init__(self, pose):
        super().__init__("football_marker_publisher")
        self.publisher_ = self.create_publisher(Marker, "/football_marker_topic", 10)
        self.pose_ = pose

    def update_pose(self, pose):
        self.pose_ = pose

    def publish_marker(self):
        marker = Marker()
        marker.id = 0
        marker.header.frame_id = "base_link"
        marker.ns = "football_marker"
        marker.pose = self.pose_.pose
        marker.action = Marker.ADD
        marker.type = Marker.MESH_RESOURCE
        marker.mesh_use_embedded_materials = True
        marker.mesh_resource = "package://demo/meshes/football.dae"
        marker.scale.x, marker.scale.y, marker.scale.z = 0.15, 0.15, 0.15
        marker.color.a = 1.0
        marker.lifetime = rclpy.duration.Duration(seconds=0).to_msg()

        self.publisher_.publish(marker)


def plan_and_execute(
    robot,
    planning_component,
    logger,
    single_plan_parameters=None,
    multi_plan_parameters=None,
    sleep_time=0.0,
):
    """Helper function to plan and execute a motion."""
    # plan to goal
    logger.info("Planning trajectory")
    if multi_plan_parameters is not None:
        plan_result = planning_component.plan(
            multi_plan_parameters=multi_plan_parameters
        )
    elif single_plan_parameters is not None:
        plan_result = planning_component.plan(
            single_plan_parameters=single_plan_parameters
        )
    else:
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

    # the Python MoveIt2 API requires configuration
    # we basically need most of the config files required to start the move group here again
    # it is also close to impossible to create a config that is accepted by MoveItPy
    # without the MoveItConfigsBuilder (hence, the config creation differs from the launch files)
    # TODO @rharbach: we need to pass the "real" controller param here somewhere for the urdf
    moveit_config = (
        MoveItConfigsBuilder(robot_name="cobot")
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
            file_path=get_package_share_directory("cobot_moveit_config")
            + "/config/moveit_controllers.yaml"
        )
        # apparently this file is read based on convention
        # in <robot_name>_moveit_config
        # .planning_pipelines(
        #    file_path=get_package_share_directory("cobot_moveit_config")
        #    + "/config/ompl_planning.yaml"
        # )
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

    # wait until joint states are available
    # otherwise we could get a "RuntimeError: Unable to configure planning scene monitor"
    node = rclpy.create_node("wait_for_joint_states")
    while True:
        joint_states = node.get_parameter_or("joint_states", None)
        if joint_states is not None:
            break
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()

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
    football_marker = FootballMarkerPublisher(pose_goal)
    football_marker.publish_marker()
    # set goal with preferred TCP frame
    cobot_arm.set_goal_state(pose_stamped_msg=pose_goal, pose_link=eef)
    # reference parameters defined in config/moveit_cpp.yaml
    single_plan_request_parameters = PlanRequestParameters(cobot, "ompl_rrtc")
    # define preferred planner
    single_plan_request_parameters.planner_id = "APSConfigDefault"
    single_plan_request_parameters.planning_pipeline = "ompl"
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
    football_marker.update_pose(pose_goal)
    football_marker.publish_marker()
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
