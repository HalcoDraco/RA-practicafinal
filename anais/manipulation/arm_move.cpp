// src/arm_move.cpp

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/Pose.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tb3_arm_move_example");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  //
  // 1) Create a MoveGroupInterface for the manipulator group
  //
  //    In turtlebot3_manipulation_moveit_config, the group name is usually "arm",
  //    but you can confirm by inspecting:
  //      rosparam get /move_group/planning_groups
  //    or by opening the SRDF under your config package.
  //
  static const std::string PLANNING_GROUP = "arm";
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  // (Optional) If you want to see what controllers are attached:
  // move_group.getJointModelGroup("arm")->getActiveJoints();

  //
  // 2) Read current end‐effector pose (initial pose)
  //
  geometry_msgs::PoseStamped current_pose_stamped = move_group.getCurrentPose();
  ROS_INFO_STREAM_NAMED("initial_pose", 
    "Initial pose: \n"
    << current_pose_stamped.pose.position.x << ", "
    << current_pose_stamped.pose.position.y << ", "
    << current_pose_stamped.pose.position.z);

  //
  // 3) Define your goal pose
  //
  geometry_msgs::Pose target_pose;
  // For example: shift +0.1 m in x, keep the same orientation:
  target_pose.orientation = current_pose_stamped.pose.orientation;
  target_pose.position.x = current_pose_stamped.pose.position.x + 0.1;
  target_pose.position.y = current_pose_stamped.pose.position.y;
  target_pose.position.z = current_pose_stamped.pose.position.z;

  move_group.setPoseTarget(target_pose);

  //
  // 4) Plan to that pose
  //
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if (!success)
  {
    ROS_ERROR("Failed to plan to target pose");
    ros::shutdown();
    return 1;
  }
  ROS_INFO("Planning to goal pose succeeded.");

  //
  // 5) Execute the plan
  //
  moveit::planning_interface::MoveItErrorCode exe_success = move_group.execute(my_plan);
  if (exe_success != moveit::planning_interface::MoveItErrorCode::SUCCESS)
  {
    ROS_ERROR("Failed to execute plan");
    ros::shutdown();
    return 1;
  }
  ROS_INFO("Execution to goal pose succeeded.");

  ros::shutdown();
  return 0;
}
