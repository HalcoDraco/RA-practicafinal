// src/arm_move.cpp

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/Pose.h>
#include <vector>

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

  moveit::planning_interface::MoveGroupInterface move_group("arm");
  moveit::planning_interface::MoveGroupInterface gripper_group("gripper");

  ros::Duration(1.0).sleep(); // Sleep for 1 second to allow the groups to initialize

  move_group.setMaxVelocityScalingFactor(0.1); // Set the velocity scaling factor
  move_group.setMaxAccelerationScalingFactor(0.1); // Set the acceleration scaling factor
  move_group.setGoalPositionTolerance(0.03); // Set position tolerance
  move_group.setGoalOrientationTolerance(0.1); // Set orientation tolerance
  move_group.setPlanningTime(10.0);
  move_group.setNumPlanningAttempts(2);


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
    << current_pose_stamped.pose.position.z << ", "
    << current_pose_stamped.pose.orientation.x << ", "
    << current_pose_stamped.pose.orientation.y << ", "
    << current_pose_stamped.pose.orientation.z << ", "
    << current_pose_stamped.pose.orientation.w);

  //
  // 3) Define your goal pose
  //
  geometry_msgs::Pose target_pose;
  // For example: shift +0.1 m in x, keep the same orientation:
  //up -0.0367087, 0.00111089, 0.545741, 0.00855517, -0.666897, 0.00955762, 0.74504
  //down 0.178152, 0.00787757, 0.10994, -0.00708296, 0.464333, 0.0135079, 0.885529
  target_pose.position.x = -0.037;
  target_pose.position.y = 0.0;
  target_pose.position.z = 0.45;
  target_pose.orientation.x = 0.0;
  target_pose.orientation.y = -0.667;
  target_pose.orientation.z = 0.0;
  target_pose.orientation.w = 0.745;

  move_group.setPoseTarget(target_pose);

  //
  // 4) Plan to that pose
  //
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
  if (!success)
  {
    ROS_ERROR("Failed to plan to target pose");
    // ros::shutdown();
    // return 1;
  } else {
    ROS_INFO("Planning to goal pose succeeded.");
  }

  //
  // 5) Execute the plan
  //
  moveit::core::MoveItErrorCode exe_success = move_group.execute(my_plan);
  if (exe_success != moveit::core::MoveItErrorCode::SUCCESS)
  {
    // Print the error code for debugging
    ROS_ERROR_STREAM("Execution to goal pose failed with error code: " << exe_success);
    // ros::shutdown();
    // return 1;
  } else {
    ROS_INFO("Execution to goal pose succeeded.");
  }

  // Read current gripper joint values
  std::vector<double> gripper_joint_values = gripper_group.getCurrentJointValues();
  ROS_INFO_STREAM_NAMED("gripper_joint_values", 
    "Current gripper joint values: "
    << gripper_joint_values[0] << ", "
    << gripper_joint_values[1]);
  // Set gripper joint values to close the gripper
  gripper_joint_values[0] = 0.15; // Close the gripper
  gripper_joint_values[1] = 0.15; // Close the gripper
  gripper_group.setJointValueTarget(gripper_joint_values);
  // Plan to close the gripper
  moveit::planning_interface::MoveGroupInterface::Plan gripper_plan;
  bool gripper_success = (gripper_group.plan(gripper_plan) == moveit::core::MoveItErrorCode::SUCCESS);
  if (!gripper_success)
  {
    ROS_ERROR("Failed to plan to close gripper");
    // ros::shutdown();
    // return 1;
  } else {
    ROS_INFO("Planning to close gripper succeeded.");
  }
  // Execute the gripper plan
  moveit::core::MoveItErrorCode gripper_exe_success = gripper_group.execute(gripper_plan);
  if (gripper_exe_success != moveit::core::MoveItErrorCode::SUCCESS)
  {
    // Print the error code for debugging
    ROS_ERROR_STREAM("Execution to close gripper failed with error code: " << gripper_exe_success);
    // ros::shutdown();
    // return 1;
  } else {
    ROS_INFO("Execution to close gripper succeeded.");
  }


  while (ros::ok())
  {
    geometry_msgs::PoseStamped current_pose_stamped = move_group.getCurrentPose();
    ROS_INFO_STREAM_NAMED("current_pose", 
      "Current pose: \n"
      << current_pose_stamped.pose.position.x << ", "
      << current_pose_stamped.pose.position.y << ", "
      << current_pose_stamped.pose.position.z << ", "
      << current_pose_stamped.pose.orientation.x << ", "
      << current_pose_stamped.pose.orientation.y << ", "
      << current_pose_stamped.pose.orientation.z << ", "
      << current_pose_stamped.pose.orientation.w);
    ros::Duration(1.5).sleep(); // Sleep for 1.5 seconds to avoid flooding the console
  }

  ros::shutdown();
  return 0;
}
