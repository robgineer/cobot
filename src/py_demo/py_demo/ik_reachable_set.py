#!/usr/bin/env python3
"""
Inverse Kinematic search for the reachable set of the Cobot.

This code runs planning requests for points within a bounding box (shuffled randomly).
Points for which an IK run was successful and unsuccessful are displayed within /points_marker_topic.
Green point: IK solution found
Red point: IK solution not found

Stores the points optionally into pickle files for further processing.

Run this search with an active moveit group (example: rviz_demo_launch.py).
"""

import rclpy
import pickle
import numpy as np
from tqdm import tqdm
from itertools import product

from rclpy.logging import get_logger

from py_utils.planner_utils import (
    plan_and_execute,
    wait_for_joint_states,
    generate_moveit_config,
)
from geometry_msgs.msg import PoseStamped

from py_utils.visualization_utils import (
    FootballMarkerPublisher,
    PointsPublisher,
    FootballMarkerArrayPublisher,  # nice visualization but slow
)


from moveit.planning import (
    MoveItPy,
    MultiPipelinePlanRequestParameters,  # not used (enables running different requests in parallel)
    PlanRequestParameters,
)

# define bounding box for IK runs
kXMin, kXMax = 0.3, 0.65
kYMin, kYMax = -0.3, 0.1
kZMin, kZMax = 0.5, 1.0
# define resolution
kDelta = 0.05  # [cm]
# store the results
kStorePoints = True
kVPointsFilenameSuffix = "baseline"


def main():

    ###########################################################################
    # Setup MoveIt2 and Planner
    ###########################################################################

    rclpy.init()
    logger = get_logger("moveit_py.grid_search")

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
    cobot_arm.set_workspace(-1.0, -0.3, 0.0, 0.2, 1.0, 1.4)

    # set plan start state to current state
    cobot_arm.set_start_state_to_current_state()

    # reference parameters defined in config/moveit_cpp.yaml
    single_plan_request_parameters = PlanRequestParameters(cobot, "ompl_rrtc")
    # define preferred planner
    single_plan_request_parameters.planner_id = "APSConfigDefault"
    single_plan_request_parameters.planning_pipeline = "ompl"
    # speed up simulation
    single_plan_request_parameters.max_velocity_scaling_factor = 1.0
    single_plan_request_parameters.max_acceleration_scaling_factor = 1.0
    # display current target pose
    football_marker = FootballMarkerPublisher()
    # football_marker_array = FootballMarkerArrayPublisher() # slow
    # use points publisher to display successful and unsuccessful IK runs
    points_marker = PointsPublisher()

    # define end effector: plan in tilted world frame orientation
    # => compensating the natural tilt of the cobot
    eef = "gripper_tcp_world_tilted_up"

    ###########################################################################
    # Setup search
    # We running the inverse kinematic solver for iteratively for points
    # within a bounding box (kXMin, kXMax, kYMin, kYMax, kZMin, kZMax)
    # with a resolution of "kDelta"
    ###########################################################################

    # create points within bounding box
    x_points = np.arange(kXMin, kXMax, kDelta)
    y_points = np.arange(kYMin, kYMax, kDelta)
    z_points = np.arange(kZMin, kZMax, kDelta)
    # create cartesian product of points in each dimension
    # => this is our search grid
    search_grid = np.array(list(product(x_points, y_points, z_points)))
    # shuffle the points randomly
    # we want to make sure that the IK solutions are not
    # only found because two points are close to each other
    # -> IK receives random points
    np.random.shuffle(search_grid)
    # store all points that have an IK solution
    valid_points = []
    # store all points that have no IK solution
    invalid_points = []

    ###########################################################################
    # Run search
    ###########################################################################

    # define goal pose
    pose_goal = PoseStamped()
    pose_goal.header.frame_id = "base_link"
    pose_goal.pose.orientation.w = 1.0

    for x, y, z in tqdm(search_grid, desc="progress"):
        pose_goal.pose.position.x = float(x)
        pose_goal.pose.position.y = float(y)
        pose_goal.pose.position.z = float(z)
        # football_marker_array.add_marker(pose_goal)
        # display current pose (visu only)
        football_marker.publish_marker(pose_goal)
        # run IK
        cobot_arm.set_goal_state(pose_stamped_msg=pose_goal, pose_link=eef)
        succeeded = plan_and_execute(
            cobot,
            cobot_arm,
            logger,
            single_plan_parameters=single_plan_request_parameters,
            sleep_time=0.0,
            execute=False,  # do not execute (IK run only)
        )
        if succeeded:
            # football_marker_array.add_marker(pose_goal)
            # update valid points with green
            points_marker.add_point(pose_goal)
            # add to list of valid points
            valid_points.append((x, y, z))
        else:
            # football_marker_array.add_marker(pose_goal, "red")
            # update invalid points with red
            points_marker.add_point(pose_goal, color="red")
            # add to list of invalid points
            invalid_points.append((x, y, z))

    if kStorePoints:
        valid_points_filename = "valid_points_" + kVPointsFilenameSuffix + ".pkl"
        with open(valid_points_filename, "wb") as valid_file:
            pickle.dump(np.asarray(valid_points), valid_file)

        invalid_points_filename = "invalid_points_" + kVPointsFilenameSuffix + ".pkl"
        with open(invalid_points_filename, "wb") as valid_file:
            pickle.dump(np.asarray(invalid_points), valid_file)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
