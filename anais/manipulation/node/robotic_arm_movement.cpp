#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/String.h>
#include <vector>

///// Constants /////

const int DEFAULT_LOG_LEVEL = 3; // Default log level
const int DEFAULT_MOVE_TRIES = 8; // Default number of move attempts

// Poses are defined with x, y, z coordinates and orientation in quaternion format (x, y, z, w).
const float HOME_POSE[7] = {
	-0.021,  // x
	0.0,     // y
	0.378,   // z
	0.0,     // orientation.x
	-0.541,  // orientation.y
	0.0,     // orientation.z
	0.841    // orientation.w
};

const float PICK_POSE[7] = {
	-0.037,  // x
	0.0,     // y
	0.45,    // z
	0.0,     // orientation.x
	-0.667,  // orientation.y
	0.0,     // orientation.z
	0.745    // orientation.w
};

const float PLACE_POSE[7] = {
	0.178,   // x
	0.0,     // y
	0.11,    // z
	0.0,  // orientation.x
	0.464,   // orientation.y
	0.014,   // orientation.z
	0.886    // orientation.w
};

const float OPEN_GRIPPER_POSE[2] = {
	0.015,
	0.015
};

const float CLOSED_GRIPPER_POSE[2] = {
	0.0066,
	0.0066
};

///// Global variables /////

// Possible values for the orquestator_communication_msg.data:
// "none" - No action required
// "pick" - Activate pick function
// "place" - Activate place function
// "done_pick" - Notify that the pick action is done (no action required)
// "done_place" - Notify that the place action is done (no action required)
std_msgs::String orquestator_communication_msg;
ros::Publisher orquestator_communication_publisher;

///// Function declarations /////

bool move_n_tries(moveit::planning_interface::MoveGroupInterface &move_group, int n_tries = DEFAULT_MOVE_TRIES, int log_level = DEFAULT_LOG_LEVEL)
{
	int attempts = 0;
	while (attempts < n_tries) {
		if (log_level > 2) {
			ROS_INFO("Attempt %d of %d to move", attempts + 1, n_tries);
		}

		move_group.setStartStateToCurrentState(); // Ensure the start state is updated before planning

		moveit::core::MoveItErrorCode success = move_group.move();

		if (success == moveit::core::MoveItErrorCode::SUCCESS) {
			if (log_level > 2) {
				ROS_INFO("Move succeeded on attempt %d", attempts + 1);
			}
			return true; // Success
		} else {
			if (log_level > 2) {
				ROS_ERROR_STREAM("Move failed with error code: " << success.val);
			}
			attempts++;
			ros::Duration(1.0).sleep(); // Wait before retrying
		}
	}

	return false;
}

int move_arm(const float pose[7], moveit::planning_interface::MoveGroupInterface &arm_group, int log_level = DEFAULT_LOG_LEVEL)
{
	// Set the target pose for the arm
	// log indicates the log level:
	// 		0 - Silent
	// 		1 - Error
	// 		2 - Info
	// Returns 0 on success, 1 on planning failure, 2 on execution failure

	if (log_level > 2) {
		ROS_INFO("Entered move_arm");
	}

	arm_group.setGoalPositionTolerance(0.03);
	arm_group.setGoalOrientationTolerance(0.1);
	arm_group.setPlanningTime(10.0);
	arm_group.setNumPlanningAttempts(2);

	if (log_level > 2) {
		ROS_INFO("Planning group created");
	}

	geometry_msgs::Pose target_pose;
	target_pose.position.x = pose[0];
	target_pose.position.y = pose[1];
	target_pose.position.z = pose[2];
	target_pose.orientation.x = pose[3];
	target_pose.orientation.y = pose[4];
	target_pose.orientation.z = pose[5];
	target_pose.orientation.w = pose[6];

	arm_group.setPoseTarget(target_pose);

	if (log_level > 2) {
		ROS_INFO("Target pose set to: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
			target_pose.position.x,
			target_pose.position.y,
			target_pose.position.z,
			target_pose.orientation.x,
			target_pose.orientation.y,
			target_pose.orientation.z,
			target_pose.orientation.w
		);
	}

	int success = move_n_tries(arm_group, DEFAULT_MOVE_TRIES, log_level);

	if (success) {
		if (log_level > 1) {
			ROS_INFO("Moving to arm pose succeeded.");
		}
		return 0; // Success
	} else {
		if (log_level > 0) {
			ROS_ERROR("Moving to arm pose failed after %d attempts.", DEFAULT_MOVE_TRIES);
		}
		return 1; // Failure
	}
}

int move_gripper(const float gripper_pose[2], moveit::planning_interface::MoveGroupInterface &gripper_group, int log_level = DEFAULT_LOG_LEVEL) 
{
	// Set the target pose for the gripper
	// log indicates the log level:
	// 		0 - Silent
	// 		1 - Error
	// 		2 - Info
	// Returns 0 on success, 1 on planning failure, 2 on execution failure

	if (log_level > 2) {
		ROS_INFO("Entered move_gripper");
	}

	std::vector<double> gripper_joint_values = {
		gripper_pose[0],
		gripper_pose[1]
	};

	gripper_group.setJointValueTarget(gripper_joint_values);

	if (log_level > 2) {
		ROS_INFO("Gripper joint values set to: [%.4f, %.4f]",
			gripper_joint_values[0],
			gripper_joint_values[1]
		);
	}

	int success = move_n_tries(gripper_group, DEFAULT_MOVE_TRIES, log_level);
	if (success) {
		if (log_level > 1) {
			ROS_INFO("Moving to gripper pose succeeded.");
		}
		return 0; // Success
	} else {
		if (log_level > 0) {
			ROS_ERROR("Moving to gripper pose failed after %d attempts.", DEFAULT_MOVE_TRIES);
		}
		return 1; // Failure
	}
}

void pick_ball_action() {

	moveit::planning_interface::MoveGroupInterface arm_group("arm");
	moveit::planning_interface::MoveGroupInterface gripper_group("gripper");

	// Move to pick pose
	move_arm(PICK_POSE, arm_group);
	ros::Duration(1.0).sleep();
	
	// Open the gripper
	move_gripper(OPEN_GRIPPER_POSE, gripper_group);

	// Wait some seconds
	ros::Duration(4.0).sleep();

	// Close the gripper
	move_gripper(CLOSED_GRIPPER_POSE, gripper_group);
	ros::Duration(1.0).sleep();

	// Move to home pose
	move_arm(HOME_POSE, arm_group);
	ros::Duration(1.0).sleep();

}

void place_ball_action() {

	moveit::planning_interface::MoveGroupInterface arm_group("arm");
	moveit::planning_interface::MoveGroupInterface gripper_group("gripper");

	// Move to place pose
	move_arm(PLACE_POSE, arm_group);
	ros::Duration(1.0).sleep();

	// Open the gripper
	move_gripper(OPEN_GRIPPER_POSE, gripper_group);
	ros::Duration(1.0).sleep();

	// Move to home pose
	move_arm(HOME_POSE, arm_group);
	ros::Duration(1.0).sleep();

	// Close the gripper
	move_gripper(CLOSED_GRIPPER_POSE, gripper_group);
	ros::Duration(1.0).sleep();
}

void comunicationCallback(const std_msgs::String::ConstPtr &msg)
{
	if (msg->data == "pick") {
		orquestator_communication_msg.data = "none"; // Reset the message to avoid repeated actions
		orquestator_communication_publisher.publish(orquestator_communication_msg); // Publish the reset message

		ROS_INFO("Initiating pick action...");
		pick_ball_action();
		ROS_INFO("Pick action completed");

		orquestator_communication_msg.data = "done_pick";
		orquestator_communication_publisher.publish(orquestator_communication_msg); // Notify that the pick action is done

	} else if (msg->data == "place") {
		orquestator_communication_msg.data = "none"; // Reset the message to avoid repeated actions
		orquestator_communication_publisher.publish(orquestator_communication_msg); // Publish the reset message

		ROS_INFO("Initiating place action...");
		place_ball_action();
		ROS_INFO("Place action completed");
		

		orquestator_communication_msg.data = "done_place";
		orquestator_communication_publisher.publish(orquestator_communication_msg); // Notify that the place action is done
	}
}

///// Main function /////

int main(int argc, char **argv)
{
	ros::init(argc, argv, "RoboticArmMovement");
	ros::NodeHandle nh;
	
	ros::AsyncSpinner spinner(2);
	spinner.start();

	orquestator_communication_publisher = nh.advertise<std_msgs::String>("/orquestator_manipulation", 1000);
	ros::Subscriber orquestator_communication_subscriber = nh.subscribe("/orquestator_manipulation", 1000, comunicationCallback);

	ros::waitForShutdown(); // Wait for shutdown signal
	ros::shutdown(); // Shutdown the ROS node

	return 0;
}
