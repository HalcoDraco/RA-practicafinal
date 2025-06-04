#!/usr/bin/env python3
"""
Orchestrator node for the UPC mobile‑manipulation project (ROS Noetic)
====================================================================

Responsibilities
----------------
• Maintains a simple finite‑state machine (FSM) that coordinates
  movement, perception and manipulation.
• Publishes **pick / place / none** commands to the manipulation node.
• Publishes **goal / stop** strings to the movement node.
• Listens for **detected_color**, **move_status** and manipulation
  feedback (**done_pick / done_place**).
• (Optionally) enables or disables vision while travelling to
  a container to save compute.

Message conventions (all *std_msgs/String* unless stated otherwise)
------------------------------------------------------------------
/vision_active         – std_msgs/Bool   (true ⇒ enabled)
/detected_color        – std_msgs/String  «yellow | blue | green»
/move_command          – std_msgs/String  «goal:x,y,yaw | stop»
/move_status           – std_msgs/String  «arrived»
/orchestrator_manip    – std_msgs/String  «pick | place | done_pick | done_place | none»

> Change topics or message types here easily if your existing
> nodes differ; keep the high‑level logic intact.
"""

import random
import threading
from typing import Dict, List

import rospy
from std_msgs.msg import String, Bool

# ---------------------------  DATA  -----------------------------------------

# Hard‑coded map coordinates for demo purposes. Replace with parameters or YAML
# if you prefer.
HOUSES: List[Dict[str, float]] = [
    {"name": "house1", "x": 1.0, "y": 2.0, "yaw": 0.0},
    {"name": "house2", "x": 3.5, "y": -0.5, "yaw": 1.57},
    {"name": "house3", "x": -2.0, "y": 1.2, "yaw": 3.14},
]

CONTAINERS: Dict[str, Dict[str, float]] = {
    "yellow": {"x": 5.0, "y": 1.0, "yaw": 0.0},
    "blue":   {"x": 5.0, "y": -1.0, "yaw": 0.0},
    "green":  {"x": 5.0, "y":  0.0, "yaw": 0.0},
}

# ----------------------------------------------------------------------------
# FSM states
class State:
    IDLE = "IDLE"                     # waiting for a house assignment
    MOVING_TO_HOUSE = "MOVING_TO_HOUSE"
    PICKING = "PICKING"
    MOVING_TO_CONTAINER = "MOVING_TO_CONTAINER"
    PLACING = "PLACING"


# -------------------------  ORCHESTRATOR  -----------------------------------
class Orchestrator:
    def __init__(self):
        self.state = State.IDLE
        self.lock = threading.RLock()  # protect shared state
        self.current_color: str | None = None

        # Publishers
        self.pub_move  = rospy.Publisher("/move_command", String, queue_size=10)
        self.pub_manip = rospy.Publisher("/orchestrator_manip", String, queue_size=10)
        self.pub_vis   = rospy.Publisher("/vision_active", Bool,   queue_size=10)

        # Subscribers
        rospy.Subscriber("/detected_color", String, self.cb_detected_color)
        rospy.Subscriber("/move_status",     String, self.cb_move_status)
        rospy.Subscriber("/orchestrator_manip", String, self.cb_manip_status)

        # Start by enabling vision and choosing an initial house
        self.enable_vision(True)
        self.dispatch_new_house()

    # --------------- helper IO ---------------------------------------------
    def enable_vision(self, flag: bool):
        self.pub_vis.publish(Bool(data=flag))

    def send_move_goal(self, pose: Dict[str, float]):
        s = f"goal:{pose['x']},{pose['y']},{pose['yaw']}"
        self.pub_move.publish(String(data=s))
        rospy.loginfo("[Orch] Sent move goal → %s", s)

    def cancel_motion(self):
        self.pub_move.publish(String(data="stop"))
        rospy.loginfo("[Orch] Sent STOP to movement node")

    def send_manip_cmd(self, cmd: str):
        self.pub_manip.publish(String(data=cmd))
        rospy.loginfo("[Orch] Sent manip cmd → %s", cmd)

    # --------------- FSM transitions ---------------------------------------
    def dispatch_new_house(self):
        with self.lock:
            house = random.choice(HOUSES)
            self.state = State.MOVING_TO_HOUSE
            self.send_move_goal(house)
            rospy.loginfo("[FSM] → MOVING_TO_HOUSE  (%s)", house["name"])

    # --------------- callbacks ---------------------------------------------
    def cb_detected_color(self, msg: String):
        with self.lock:
            if self.state == State.MOVING_TO_HOUSE:
                color = msg.data.lower()
                if color in CONTAINERS:
                    rospy.loginfo("[Vision] Detected %s", color)
                    self.current_color = color

                    # Pause nav & vision, start pick
                    self.cancel_motion()
                    self.enable_vision(False)
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
                # Head to the appropriate container
                pose = CONTAINERS[self.current_color]
                self.state = State.MOVING_TO_CONTAINER
                self.send_move_goal(pose)
                rospy.loginfo("[FSM] → MOVING_TO_CONTAINER  (%s)", self.current_color)

            elif msg.data == "done_place" and self.state == State.PLACING:
                # Reset and continue with houses
                self.current_color = None
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
