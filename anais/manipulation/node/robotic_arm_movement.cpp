#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/String.h>
#include <vector>

///// Constants /////

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

const float CLOSED_GRIPPER_POSE[2] = {
	0.0,
	0.0
};

const float OPEN_GRIPPER_POSE[2] = {
	0.0079,
	0.0079
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

int move_arm(const float pose[7], int log_level = 1) {
	// Set the target pose for the arm
	// log indicates the log level:
	// 		0 - Silent
	// 		1 - Error
	// 		2 - Info
	// Returns 0 on success, 1 on planning failure, 2 on execution failure

	if (log_level > 2) {
		ROS_INFO("Entered move_arm");
	}

	moveit::planning_interface::MoveGroupInterface move_group("arm");

	move_group.setGoalPositionTolerance(0.03);
	move_group.setGoalOrientationTolerance(0.1);
	move_group.setPlanningTime(10.0);
	move_group.setNumPlanningAttempts(2);

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

	move_group.setPoseTarget(target_pose);

	moveit::planning_interface::MoveGroupInterface::Plan my_plan;
	moveit::core::MoveItErrorCode plan_success = move_group.plan(my_plan);
	if (plan_success != moveit::core::MoveItErrorCode::SUCCESS) {
		if (log_level > 0) {
			ROS_ERROR_STREAM("Planning to target pose failed with error code: " << plan_success);
		}
		return 1;
	}
	moveit::core::MoveItErrorCode exe_success = move_group.execute(my_plan);
	if (exe_success != moveit::core::MoveItErrorCode::SUCCESS) {
		if (log_level > 0) {
			ROS_ERROR_STREAM("Execution to target pose failed with error code: " << exe_success);
		}
		return 2;
	}
	if (log_level > 1) {
		ROS_INFO("Execution to target pose succeeded.");
	}
	return 0;
}

int move_gripper(const float gripper_pose[2], int log_level = 1) {
	// Set the target pose for the gripper
	// log indicates the log level:
	// 		0 - Silent
	// 		1 - Error
	// 		2 - Info
	// Returns 0 on success, 1 on planning failure, 2 on execution failure

	moveit::planning_interface::MoveGroupInterface gripper_group("gripper");

	std::vector<double> gripper_joint_values = {
		gripper_pose[0],
		gripper_pose[1]
	};

	gripper_group.setJointValueTarget(gripper_joint_values);

	moveit::planning_interface::MoveGroupInterface::Plan gripper_plan;
	moveit::core::MoveItErrorCode gripper_plan_success = gripper_group.plan(gripper_plan);
	if (gripper_plan_success != moveit::core::MoveItErrorCode::SUCCESS) {
		if (log_level > 0) {
			ROS_ERROR_STREAM("Planning to gripper pose failed with error code: " << gripper_plan_success);
		}
		return 1;
	}
	moveit::core::MoveItErrorCode gripper_exe_success = gripper_group.execute(gripper_plan);
	if (gripper_exe_success != moveit::core::MoveItErrorCode::SUCCESS) {
		if (log_level > 0) {
			ROS_ERROR_STREAM("Execution to gripper pose failed with error code: " << gripper_exe_success);
		}
		return 2;
	}
	if (log_level > 1) {
		ROS_INFO("Execution to gripper pose succeeded.");
	}
	return 0;
}

bool pick_ball_action() {
	// Move to pick pose
	int result = move_arm(PICK_POSE, 3);
	if (result != 0) {
		return false; // Return false if there was an error
	}
	
	// Open the gripper
	result = move_gripper(OPEN_GRIPPER_POSE, 3);
	if (result != 0) {
		return false; // Return false if there was an error
	}

	// Wait 5 seconds
	ros::Duration(5.0).sleep();

	// Close the gripper
	result = move_gripper(CLOSED_GRIPPER_POSE);
	if (result != 0) {
		return false; // Return false if there was an error
	}

	// Move to home pose
	result = move_arm(HOME_POSE);
	if (result != 0) {
		return false; // Return false if there was an error
	}

	return true; // Return true if the action was successful
}

bool place_ball_action() {
	// Move to place pose
	int result = move_arm(PLACE_POSE);
	if (result != 0) {
		return false; // Return false if there was an error
	}

	// Open the gripper
	result = move_gripper(OPEN_GRIPPER_POSE);
	if (result != 0) {
		return false; // Return false if there was an error
	}

	// Move to home pose
	result = move_arm(HOME_POSE);
	if (result != 0) {
		return false; // Return false if there was an error
	}

	// Close the gripper
	result = move_gripper(CLOSED_GRIPPER_POSE);
	if (result != 0) {
		return false; // Return false if there was an error
	}

	return true; // Return true if the action was successful
}

void comunicationCallback(const std_msgs::String::ConstPtr &msg)
{
	if (msg->data == "pick") {
		orquestator_communication_msg.data = "none"; // Reset the message to avoid repeated actions
		orquestator_communication_publisher.publish(orquestator_communication_msg); // Publish the reset message

		ROS_INFO("Initiating pick action...");
		bool result = pick_ball_action();
		if (result) {
			ROS_INFO("Pick action completed successfully.");
		} else {
			ROS_ERROR("Pick action failed.");
		}

		orquestator_communication_msg.data = "done_pick";
		orquestator_communication_publisher.publish(orquestator_communication_msg); // Notify that the pick action is done

	} else if (msg->data == "place") {
		orquestator_communication_msg.data = "none"; // Reset the message to avoid repeated actions
		orquestator_communication_publisher.publish(orquestator_communication_msg); // Publish the reset message

		ROS_INFO("Initiating place action...");
		bool result = place_ball_action();
		if (result) {
			ROS_INFO("Place action completed successfully.");
		} else {
			ROS_ERROR("Place action failed.");
		}

		orquestator_communication_msg.data = "done_place";
		orquestator_communication_publisher.publish(orquestator_communication_msg); // Notify that the place action is done
	}
}

///// Main function /////

int main(int argc, char **argv)
{
	ros::init(argc, argv, "RoboticArmMovement");
	ros::NodeHandle nh;
	
	ros::AsyncSpinner spinner(1);
	spinner.start();

	ros::Duration(1.0).sleep(); // Sleep for 1 second to allow the groups to initialize

	orquestator_communication_publisher = nh.advertise<std_msgs::String>("/orquestator_manipulation", 1000);
	ros::Subscriber orquestator_communication_subscriber = nh.subscribe("/orquestator_manipulation", 1000, comunicationCallback);

	// orquestator_communication_msg.data = "pick"; // Initialize the message to "none"
	// comunicationCallback(orquestator_communication_msg); // Initial call to set the message

	ros::Rate loop_rate(1); // 1 Hz

	while (ros::ok()) {
		// orquestator_communication_publisher.publish(orquestator_communication_msg);
		// ros::spinOnce(); // Process incoming messages
		loop_rate.sleep();
	}

	ros::waitForShutdown(); // Wait for shutdown signal
	ros::shutdown(); // Shutdown the ROS node

	return 0;
}
