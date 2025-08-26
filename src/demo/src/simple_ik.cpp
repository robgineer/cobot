/*********************************************************************
 * Simple Inverse Kinematic Demo for Cobot with MoveIt Visual Tools.
 *
 * Run this demo with an active moveit group (example: rviz_demo_launch.py).
 *
 * Adapted from:
 * https://moveit.picknik.ai/main/doc/examples/move_group_interface/move_group_interface_tutorial.html
 *********************************************************************/

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>

/*
 * Handle the visual part: wait for rviz to subscribe to marker and publish desired pose
 */
void publishFootballMarker(std::shared_ptr<rclcpp::Node> node,
                           moveit_visual_tools::MoveItVisualTools &visual_tools,
                           geometry_msgs::msg::Pose &football_pose,
                           rclcpp::Logger &logger)
{

  // do not start with planning until topic has been subscribed in rviz
  RCLCPP_INFO(logger, "Waiting for rviz to subscribe topic: add MarkerArray and select /visual_tools_topic to continue");
  while (node->get_node_graph_interface()->count_subscribers("visual_tools_topic") == 0)
  {
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }
  RCLCPP_INFO(logger, "Topic subscribed, publishing MarkerArray.");
  visual_tools.deleteAllMarkers();

  // publish football
  visual_tools.publishMesh(football_pose, "package://demo/meshes/football.dae", rviz_visual_tools::BLACK, 0.15);
  visual_tools.trigger();
}

/*
 * Handle planning part: set target pose, plan and execute.
 */
void planAndExecute(std::shared_ptr<rclcpp::Node> node, geometry_msgs::msg::Pose &football_pose, std::string eef)
{
  // get logger
  auto logger = rclcpp::get_logger("simple_ik_node_log");

  // get move group
  const auto kArmGroup = "arm_group";
  moveit::planning_interface::MoveGroupInterface move_group(node, kArmGroup);
  move_group.setPlannerId("APSConfigDefault");

  // set up visual tools and publish marker
  moveit_visual_tools::MoveItVisualTools visual_tools(node, "world", "visual_tools_topic",
                                                      move_group.getRobotModel());
  publishFootballMarker(node, visual_tools, football_pose, logger);

  // set a very small planning time
  // => with the current model and planner configuration, we are able to run fast IKs
  move_group.setPlanningTime(0.5);
  // speed up simulation
  move_group.setMaxAccelerationScalingFactor(1.0);
  move_group.setMaxVelocityScalingFactor(1.0);
  // run inverse kinematic solver
  move_group.setPoseTarget(football_pose, eef);
  moveit::planning_interface::MoveGroupInterface::Plan reach_football_plan;
  bool success = (move_group.plan(reach_football_plan) == moveit::core::MoveItErrorCode::SUCCESS);

  // execute the planned trajectory
  if (success)
  {
    move_group.execute(reach_football_plan);
    RCLCPP_INFO(logger, "Planning succeeded.");
    RCLCPP_INFO(logger, "Planner ID used: %s", move_group.getPlannerId().c_str());
    // wait for the execution to complete: this is required for gazebo
    rclcpp::sleep_for(std::chrono::milliseconds(3000));
  }
  else
  {
    auto text_pose = football_pose;
    text_pose.position.z = 1.5;
    visual_tools.publishText(text_pose, "Planing failed!", rviz_visual_tools::RED, rviz_visual_tools::XLARGE);
    visual_tools.trigger();
    RCLCPP_ERROR(logger, "Planing failed! Cannot execute trajectory.");
  }
}

int main(int argc, char **argv)
{
  // init ROS2 context, create node and run node as thread
  rclcpp::init(argc, argv);
  auto simple_ik_node = rclcpp::Node::make_shared("simple_ik_node");
  rclcpp::spin_some(simple_ik_node);

  // define pose goal
  geometry_msgs::msg::Pose football_pose;
  football_pose.orientation.w = 1.0; // neutral orientation
  football_pose.position.x = 0.6;
  football_pose.position.y = -0.3;
  football_pose.position.z = 1.0;
  /*
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
  */
  // plan for TCP in world frame
  planAndExecute(simple_ik_node, football_pose, "gripper_tcp_world");
  // plan for TCP in world frame tilted with z+ up
  planAndExecute(simple_ik_node, football_pose, "gripper_tcp_world_tilted_up");
  // rotate 180 deg around y => z+ is down
  football_pose.orientation.w = 0.0;
  football_pose.orientation.y = 1.0;
  // move pose goal to the right to see change
  football_pose.position.y = -0.4;
  // plan for TCP in world frame tilted with z+ down
  planAndExecute(simple_ik_node, football_pose, "gripper_tcp_world_tilted_down");
  // exit demo
  rclcpp::shutdown();
  return 0;
}
