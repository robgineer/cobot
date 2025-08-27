"""
Planner specific utilities.
Used in the py_demo package as well as for the grid search.
"""

import time
import rclpy

from sensor_msgs.msg import JointState
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory


def plan_and_execute(
    robot,
    planning_component,
    logger,
    single_plan_parameters=None,
    multi_plan_parameters=None,
    sleep_time=0.0,
    execute=True,
):
    """Helper function to plan and execute a motion.

    Returns true if a plan was successful.
    """
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
    if plan_result and execute:
        logger.info("Executing plan")
        robot_trajectory = plan_result.trajectory
        robot.execute(robot_trajectory, controllers=[])
        time.sleep(sleep_time)
    else:
        logger.error("Planning failed")

    return plan_result


def wait_for_joint_states(logger):
    """Creates a node, subscribes to /joint_states and waits until
    joint states are available. Without waiting, we could run into a
    'RuntimeError: Unable to configure planning scene monitor'.

    Args:
        logger: the logger instance of the caller
    """
    node = rclpy.create_node("wait_for_joint_states")

    joint_states_msg = {"data": None}

    node.create_subscription(
        JointState,
        "/joint_states",
        lambda msg: joint_states_msg.update({"data": msg}),
        10,
    )

    while joint_states_msg["data"] is None:
        logger.warn("Joint states are not available. Retrying...")
        rclpy.spin_once(node, timeout_sec=0.1)

    logger.info("Joint states received")

    node.destroy_node()


def generate_moveit_config() -> dict:
    """Generates a MoveIt2 configuration using the MoveItConfigsBuilder.

    Unlike the C++ MoveGroup API, the Python MoveIt2 API requires configuration.
    We basically need most of the config files required to start the move group here again.
    It is also close to impossible to create a config that is accepted by MoveItPy
    without the MoveItConfigsBuilder (hence, the config creation differs from the launch files).

    Returns: MoveIt2 config as dict.

    TODO @rharbach: we need to pass the "real" controller param here somewhere for the urdf.
    """

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
        # this defines the planner config
        # TODO @rharbach: move this to the config package
        .moveit_cpp(
            file_path=get_package_share_directory("py_demo") + "/config/moveit_cpp.yaml"
        )
        .to_moveit_configs()
        .to_dict()
    )
    return moveit_config
