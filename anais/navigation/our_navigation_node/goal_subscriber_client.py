#!/usr/bin/env python3

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
# Import GoalStatus from actionlib_msgs.msg to correctly access goal states
from actionlib_msgs.msg import GoalStatus as AS_GoalStatus # Renamed to avoid conflict if GoalStatus already used

class GoalCommander:
    def __init__(self):
        rospy.init_node('goal_commander', anonymous=True)

        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.move_base_client.wait_for_server()
        rospy.loginfo("move_base action server connected.")

        # Subscriber for incoming goals (on /move_command as per your setup)
        rospy.Subscriber("/move_command", PoseStamped, self.goal_callback)

        # Subscriber for move status commands (e.g., "stop")
        rospy.Subscriber("/move_status", String, self.move_status_callback)

        # Publisher for sending robot status (e.g., "arrived")
        self.status_publisher = rospy.Publisher("/move_status", String, queue_size=10)

        rospy.loginfo("Goal Commander node ready.")

    def goal_callback(self, msg):
        rospy.loginfo("Received new goal:")
        rospy.loginfo("  Position: x=%f, y=%f, z=%f", msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
        rospy.loginfo("  Orientation: x=%f, y=%f, z=%f, w=%f", msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w)

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = msg.header.frame_id
        goal.target_pose.pose = msg.pose

        rospy.loginfo("Sending goal to move_base...")
        # Send the goal and register a 'done_cb'
        self.move_base_client.send_goal(goal, done_cb=self.goal_done_callback)

    def goal_done_callback(self, state, result):
        """
        Callback function called by actionlib when the goal is finished.
        :param state: The terminal status code of the goal (integer value).
        :param result: The result message.
        """
        # The 'state' parameter here is an integer status code.
        # We compare it to the status codes defined in actionlib_msgs.msg.GoalStatus.
        if state == AS_GoalStatus.SUCCEEDED:
            rospy.loginfo("Goal reached successfully!")
            self.status_publisher.publish(String("arrived"))
        elif state == AS_GoalStatus.PREEMPTED:
            rospy.logwarn("Goal was preempted (cancelled).")
        elif state == AS_GoalStatus.ABORTED:
            # For ABORTED, get_goal_status_text() can provide more details.
            rospy.logerr("Goal aborted! Reason: %s", self.move_base_client.get_goal_status_text())
        else:
            # For other possible states (e.g., RECALLED, REJECTED, PENDING, ACTIVE),
            # though done_cb typically fires for terminal states.
            rospy.loginfo("Goal finished with state: %s (%d)", self.move_base_client.get_state_text(state), state)


    def move_status_callback(self, msg):
        command = msg.data.lower()
        rospy.loginfo("Received move_status command: %s", command)

        if command == "stop":
            rospy.loginfo("Cancelling all active goals for move_base...")
            self.move_base_client.cancel_all_goals()
            rospy.loginfo("Goals cancelled.")
        elif command == "arrived":
            rospy.loginfo("Received 'arrived' confirmation.")
        else:
            rospy.logwarn("Unknown command received on /move_status: %s", msg.data)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        commander = GoalCommander()
        commander.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Goal Commander node terminated.")