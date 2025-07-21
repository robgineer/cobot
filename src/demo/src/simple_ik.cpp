/*********************************************************************
 * Simple Inverse Kinematic Demo for Cobot with MoveIt Visual Tools.
 *
 * Run this demo with an active moveit group (example: simple_ik_launch.py).
 *
 * Adapted from:
 * https://moveit.picknik.ai/main/doc/examples/move_group_interface/move_group_interface_tutorial.html
 *********************************************************************/

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>

int main(int argc, char **argv)
{
  // init ROS2 context, create node, logger and run node as thread
  rclcpp::init(argc, argv);
  auto simple_ik_node = rclcpp::Node::make_shared("simple_ik_node");
  auto logger = rclcpp::get_logger("simple_ik_node_log");

  rclcpp::spin_some(simple_ik_node);

  // get move group
  const auto kArmGroup = "arm_group";
  moveit::planning_interface::MoveGroupInterface move_group(simple_ik_node, kArmGroup);
  move_group.setPlannerId("APSConfigDefault");

  // handle the visual part (do not start with planning until topic has been subscribed in rviz)
  moveit_visual_tools::MoveItVisualTools visual_tools(simple_ik_node, "world", "visual_tools_topic",
                                                      move_group.getRobotModel());
  RCLCPP_INFO(logger, "Waiting for rviz to subscribe topic: add MarkerArray and select /visual_tools_topic to continue");
  while (simple_ik_node->get_node_graph_interface()->count_subscribers("visual_tools_topic") == 0)
  {
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }
  RCLCPP_INFO(logger, "Topic subscribed, publishing MarkerArray.");
  visual_tools.deleteAllMarkers();

  geometry_msgs::msg::Pose football_pose;
  football_pose.orientation.w = 1.0;
  football_pose.position.x = -0.3;
  football_pose.position.y = 0.3;
  football_pose.position.z = 1.0;
  // publish football
  visual_tools.publishMesh(football_pose, "package://demo/meshes/football.dae", rviz_visual_tools::BLACK, 0.15);
  visual_tools.trigger();
  // run inverse kinematic solver
  move_group.setPositionTarget(football_pose.position.x, football_pose.position.y, football_pose.position.z);
  move_group.setPoseTarget(football_pose);
  move_group.setGoalTolerance(0.1);
  moveit::planning_interface::MoveGroupInterface::Plan reach_football_plan;
  bool success = (move_group.plan(reach_football_plan) == moveit::core::MoveItErrorCode::SUCCESS);

  // execute the planned trajectory
  if (success)
  {
    move_group.execute(reach_football_plan);
    RCLCPP_INFO(logger, "Planning succeeded.");
    RCLCPP_INFO(logger, "Planner ID used: %s", move_group.getPlannerId().c_str());
  }
  else
  {
    auto text_pose = football_pose;
    text_pose.position.z = 1.5;
    visual_tools.publishText(text_pose, "Planing failed!", rviz_visual_tools::RED, rviz_visual_tools::XLARGE);
    visual_tools.trigger();
    RCLCPP_ERROR(logger, "Planing failed! Cannot execute trajectory.");
  }

  // exit demo
  rclcpp::shutdown();
  return 0;
}