#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/String.h>
#include <vector>

class RoboticArmController
{
public:
	RoboticArmController()
	: nh_(),
		move_group_("arm"),
		gripper_group_("gripper")
	{
		orquestator_comm_pub_ = nh_.advertise<std_msgs::String>("/orquestator_manipulation", 1000);
		orquestator_comm_sub_ = nh_.subscribe(
			"/orquestator_manipulation", 1000,
			&RoboticArmController::communicationCallback,
			this
		);

		// 2) (Re)configure MoveIt! parameters for the arm group:
		move_group_.setGoalPositionTolerance(0.03);
		move_group_.setGoalOrientationTolerance(0.1);
		move_group_.setPlanningTime(10.0);
		move_group_.setNumPlanningAttempts(2);

		// 3) Let the planning groups “warm up” before first use:
		ros::Duration(1.0).sleep();
	}

	// ─── Constants ───────────────────────────────────────────────────────────────
	// Poses are defined with x, y, z coordinates and orientation in quaternion format (x, y, z, w).
	const float HOME_POSE_[7] = {
		-0.021,  // x
		0.0,     // y
		0.378,   // z
		0.0,     // orientation.x
		-0.541,  // orientation.y
		0.0,     // orientation.z
		0.841    // orientation.w
	};

	const float PICK_POSE_[7] = {
		-0.037,  // x
		0.0,     // y
		0.45,    // z
		0.0,     // orientation.x
		-0.667,  // orientation.y
		0.0,     // orientation.z
		0.745    // orientation.w
	};

	const float PLACE_POSE_[7] = {
		0.178,   // x
		0.0,     // y
		0.11,    // z
		0.0,  // orientation.x
		0.464,   // orientation.y
		0.014,   // orientation.z
		0.886    // orientation.w
	};

	const float CLOSED_GRIPPER_POSE_[2] = {
		0.0,
		0.0
	};

	const float OPEN_GRIPPER_POSE_[2] = {
		0.0079,
		0.0079
	};

	// ─── Member Variables ──────────────────────────────────────────────────────
	ros::NodeHandle nh_;
	moveit::planning_interface::MoveGroupInterface move_group_;
	moveit::planning_interface::MoveGroupInterface gripper_group_;
	ros::Publisher orquestator_comm_pub_;
	ros::Subscriber orquestator_comm_sub_;
	std_msgs::String orquestator_communication_msg_;

	// ─── Core Methods ───────────────────────────────────────────────────────────

	/// Moves the arm to a pose (given as [x,y,z, qx,qy,qz,qw]).
	/// Returns 0 on success, 1 on planning failure, 2 on execution failure.
	int moveArm(const float pose[7], int log_level = 3)
	{
		if (log_level > 2) ROS_INFO("Moving arm");

		geometry_msgs::Pose target;
		target.position.x = pose[0];
		target.position.y = pose[1];
		target.position.z = pose[2];
		target.orientation.x = pose[3];
		target.orientation.y = pose[4];
		target.orientation.z = pose[5];
		target.orientation.w = pose[6];

		move_group_.setPoseTarget(target);

		if (log_level > 2)
			ROS_INFO("Arm target pose set");

		moveit::planning_interface::MoveGroupInterface::Plan plan;
		auto plan_success = move_group_.plan(plan);
		if (plan_success != moveit::core::MoveItErrorCode::SUCCESS)
		{
			if (log_level > 0)
			ROS_ERROR_STREAM("Arm planning failed [" << plan_success << "]");
			return 1;
		}

		if (log_level > 2)
			ROS_INFO("Arm plan created successfully");

		auto exec_success = move_group_.execute(plan);
		if (exec_success != moveit::core::MoveItErrorCode::SUCCESS)
		{
			if (log_level > 0)
			ROS_ERROR_STREAM("Arm execution failed [" << exec_success << "]");
			return 2;
		}

		if (log_level > 1)
			ROS_INFO("Arm reached target pose successfully.");
			
		return 0;
	}

	/// Moves the gripper to a joint-value target (2 joints: left & right finger).
	/// Returns 0 on success, 1 on planning failure, 2 on execution failure.
	int moveGripper(const float target_joints[2], int log_level = 1)
	{
		std::vector<double> joint_vals{ target_joints[0], target_joints[1] };
		gripper_group_.setJointValueTarget(joint_vals);

		moveit::planning_interface::MoveGroupInterface::Plan plan;
		auto plan_success = gripper_group_.plan(plan);
		if (plan_success != moveit::core::MoveItErrorCode::SUCCESS)
		{
			if (log_level > 0)
			ROS_ERROR_STREAM("Gripper planning failed [" << plan_success << "]");
			return 1;
		}

		auto exec_success = gripper_group_.execute(plan);
		if (exec_success != moveit::core::MoveItErrorCode::SUCCESS)
		{
			if (log_level > 0)
			ROS_ERROR_STREAM("Gripper execution failed [" << exec_success << "]");
			return 2;
		}

		if (log_level > 1)
			ROS_INFO("Gripper reached target configuration successfully.");
			
		return 0;
	}

	/// High-level “pick ball” sequence:
	///  1) Move arm to PICK_POSE
	///  2) Open gripper
	///  3) wait 5 s
	///  4) Close gripper
	///  5) Move arm to HOME_POSE
	bool pickBallAction()
	{
		if (moveArm(PICK_POSE_) != 0)     return false;
		if (moveGripper(OPEN_GRIPPER_POSE_) != 0) return false;
		ros::Duration(5.0).sleep();
		if (moveGripper(CLOSED_GRIPPER_POSE_) != 0) return false;
		if (moveArm(HOME_POSE_) != 0)    return false;
		return true;
	}

	/// High-level “place ball” sequence:
	///  1) Move arm to PLACE_POSE
	///  2) Open gripper
	///  3) Move arm to HOME_POSE
	///  4) Close gripper
	bool placeBallAction()
	{
		if (moveArm(PLACE_POSE_) != 0)    return false;
		if (moveGripper(OPEN_GRIPPER_POSE_) != 0) return false;
		if (moveArm(HOME_POSE_) != 0)    return false;
		if (moveGripper(CLOSED_GRIPPER_POSE_) != 0) return false;
		return true;
	}

	/// ROS callback for “/orquestator_manipulation”
	void communicationCallback(const std_msgs::String::ConstPtr& msg)
	{
		if (msg->data == "pick")
		{
			// 1) Reset to “none” so we don't immediately re-trigger
			orquestator_communication_msg_.data = "none";
			orquestator_comm_pub_.publish(orquestator_communication_msg_);

			ROS_INFO("Initiating pick action...");
			bool ok = pickBallAction();
			if (ok)
			ROS_INFO("Pick action succeeded.");
			else
			ROS_ERROR("Pick action failed.");

			orquestator_communication_msg_.data = "done_pick";
			orquestator_comm_pub_.publish(orquestator_communication_msg_);
		}
		else if (msg->data == "place")
		{
			orquestator_communication_msg_.data = "none";
			orquestator_comm_pub_.publish(orquestator_communication_msg_);

			ROS_INFO("Initiating place action...");
			bool ok = placeBallAction();
			if (ok)
			ROS_INFO("Place action succeeded.");
			else
			ROS_ERROR("Place action failed.");

			orquestator_communication_msg_.data = "done_place";
			orquestator_comm_pub_.publish(orquestator_communication_msg_);
		}
	// else: ignore “none” or any unrecognized command
	}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "RoboticArmMovement");

	ros::AsyncSpinner spinner(1);
	spinner.start();

	RoboticArmController controller;

	ros::waitForShutdown();

	return 0;	
}
