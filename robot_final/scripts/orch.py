#!/usr/bin/env python3
"""
Orchestrator node for the UPC mobile‑manipulation project (ROS Noetic)
====================================================================

Responsibilities
----------------
• Maintains a simple finite‑state machine (FSM) that coordinates
  movement, perception and manipulation.
• Publishes **pick / place / none** commands to the manipulation node.
• Publishes **geometry_msgs/PoseStamped** goals to the movement node.
• Sends "stop" string to movement node via /move_status.
• Listens for **detected_color**, **arrived**, and manipulation feedback (**done_pick / done_place**).
• Enables or disables vision while travelling to a container to save compute.

Message conventions
-------------------
/vision_active         – std_msgs/Bool   (true ⇒ enabled)
/detected_color        – std_msgs/String  «yellow | blue | green»
/move_command          – geometry_msgs/PoseStamped
/move_status           – std_msgs/String  «arrived»
/move_ctrl             – std_msgs/String  «stop»
/orchestrator_manip    – std_msgs/String  «pick | place | done_pick | done_place | none»
"""

import random
import threading
from typing import Dict, List

import rospy
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion

# ---------------------------  DATA  -----------------------------------------

HOUSES: Dict[str, PoseStamped] = {
    
    "house1": PoseStamped(header=rospy.Header(frame_id="map"), pose=Pose(position=Point(-3.632812, 1.359375, 0.0), orientation=Quaternion(0.0, 0.0, 0.687581, 0.726107))),
    "house2": PoseStamped(header=rospy.Header(frame_id="map"), pose=Pose(position=Point(-5.257812, 0.640625, 0.0), orientation=Quaternion(0.0, 0.0, 0.999908, 0.013509))),
    "house3": PoseStamped(header=rospy.Header(frame_id="map"), pose=Pose(position=Point(-0.539999, 0.959999, 0.0), orientation=Quaternion(0.0, 0.0, 0.700468, 0.713683)))
}

CONTAINERS: Dict[str, PoseStamped] = {
    "yellow": PoseStamped(header=rospy.Header(frame_id="map"), pose=Pose(position=Point(-0.570313, -0.234375, 0.0), orientation=Quaternion(0.0, 0.0, -0.716857, 0.697221))),
    "blue":   PoseStamped(header=rospy.Header(frame_id="map"), pose=Pose(position=Point(-2.601562, -0.109374, 0.0), orientation=Quaternion(0.0, 0.0, -0.722982, 0.690866))),
    "green":  PoseStamped(header=rospy.Header(frame_id="map"), pose=Pose(position=Point(-4.664062, 0.078125, 0.0), orientation=Quaternion(0.0, 0.0, -0.707106, 0.707106)))
}

# ----------------------------------------------------------------------------
# FSM states
class State:
    IDLE = "IDLE"
    MOVING_TO_HOUSE = "MOVING_TO_HOUSE"
    PICKING = "PICKING"
    MOVING_TO_CONTAINER = "MOVING_TO_CONTAINER"
    PLACING = "PLACING"

# -------------------------  ORCHESTRATOR  -----------------------------------
class Orchestrator:
    def __init__(self):
        self.state = State.IDLE
        self.lock = threading.RLock()
        self.current_color: str | None = None
        self.last_house = None

        # Publishers
        self.pub_move  = rospy.Publisher("/move_command", PoseStamped, queue_size=10)
        self.pub_move_ctrl = rospy.Publisher("/move_ctrl", String, queue_size=10)
        self.pub_manip = rospy.Publisher("/orchestrator_manip", String, queue_size=10)
        self.pub_vis   = rospy.Publisher("/vision_active", Bool, queue_size=10)

        # Subscribers
        rospy.Subscriber("/detected_color", String, self.cb_detected_color)
        rospy.Subscriber("/move_status", String, self.cb_move_status)
        rospy.Subscriber("/orchestrator_manip", String, self.cb_manip_status)

        rospy.sleep(2)  # Wait for publishers to be ready
        rospy.loginfo("[Orch] Orchestrator initialized")

        self.enable_vision(True)
        self.dispatch_new_house()

    def enable_vision(self, flag: bool):
        self.pub_vis.publish(Bool(data=flag))

    def send_move_goal(self, pose_stamped: PoseStamped):
        pose_stamped.header.stamp = rospy.Time.now()
        self.pub_move.publish(pose_stamped)
        rospy.loginfo("[Orch] Sent move goal")

    def cancel_motion(self):
        self.pub_move_ctrl.publish(String(data="stop"))
        rospy.loginfo("[Orch] Sent STOP to movement node")

    def send_manip_cmd(self, cmd: str):
        self.pub_manip.publish(String(data=cmd))
        rospy.loginfo("[Orch] Sent manip cmd → %s", cmd)

    def dispatch_new_house(self):
        with self.lock:
            # Ensure we don't send the same house twice in a row
            if self.last_house is not None:
                available_houses = [h for h in HOUSES if h != self.last_house]
            else:
                available_houses = list(HOUSES.keys())
            
            house = random.choice(available_houses)
            self.last_house = house
            self.state = State.MOVING_TO_HOUSE
            self.send_move_goal(HOUSES[house])
            rospy.loginfo("[FSM] → MOVING_TO_HOUSE (%s)", house)

    def cb_detected_color(self, msg: String):
        with self.lock:
            if self.state == State.MOVING_TO_HOUSE:
                color = msg.data.lower()
                if color in CONTAINERS:
                    rospy.loginfo("[Vision] Detected %s", color)
                    self.current_color = color
                    self.cancel_motion()
                    self.enable_vision(False)
                    rospy.sleep(2)
                    self.send_manip_cmd("pick")
                    self.state = State.PICKING
                    rospy.loginfo("[FSM] → PICKING")

    def cb_move_status(self, msg: String):
        with self.lock:
            if msg.data != "arrived":
                return

            if self.state == State.MOVING_TO_HOUSE:
                rospy.loginfo("[Move] Arrived at house; picking next one")
                self.dispatch_new_house()

            elif self.state == State.MOVING_TO_CONTAINER:
                rospy.loginfo("[Move] Arrived at container; issuing place")
                self.send_manip_cmd("place")
                self.state = State.PLACING
                rospy.loginfo("[FSM] → PLACING")

    def cb_manip_status(self, msg: String):
        with self.lock:
            if msg.data == "done_pick" and self.state == State.PICKING:
                pose = CONTAINERS[self.current_color]
                self.send_move_goal(pose)
                self.state = State.MOVING_TO_CONTAINER
                rospy.loginfo("[FSM] → MOVING_TO_CONTAINER  (%s)", self.current_color)

            elif msg.data == "done_place" and self.state == State.PLACING:
                self.current_color = None
                self.state = State.MOVING_TO_HOUSE #?
                self.enable_vision(True)
                self.dispatch_new_house()
                rospy.loginfo("[FSM] Cycle complete → back to houses")

# -----------------------------  MAIN  ---------------------------------------
def main():
    rospy.init_node("orchestrator_node")
    Orchestrator()
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
