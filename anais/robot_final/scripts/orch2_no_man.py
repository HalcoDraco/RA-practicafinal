#!/usr/bin/env python3
"""
Orchestrator node – *scan‑then‑move* strategy (ROS Noetic)
=========================================================

This version avoids running navigation and vision at the same time:

1. **MOVING_TO_HOUSE**  · Vision **OFF** · Robot drives to a house.
2. **SCANNING_AT_HOUSE** · Vision **ON** for *SCAN_DURATION* seconds.
   * If a coloured ball is detected → Vision **OFF** → **MOVING_TO_CONTAINER**.
   * If nothing is detected before timeout → Vision **OFF** → next house.
3. **MOVING_TO_CONTAINER** · Vision **OFF** · When the container goal is reached, go back to step 1.

Only navigation and perception are handled; the manipulator is ignored.

Topics used
-----------
/vision_active  – std_msgs/Bool (true ⇒ enabled)
/detected_color – std_msgs/String («yellow | blue | green»)
/move_command   – geometry_msgs/PoseStamped (goals)
/move_ctrl      – std_msgs/String («stop»)
/move_status    – std_msgs/String («arrived»)
"""

import random
import threading
from typing import Dict

import rospy
from std_msgs.msg import String, Bool, Header
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion

# ----------------------  PARAMETERS  ---------------------------------------
SCAN_DURATION = 8.0  # seconds that vision stays active at each house

# ---------------------------  DATA  ----------------------------------------
HOUSES: Dict[str, PoseStamped] = {
    "house1": PoseStamped(header=Header(frame_id="map"), pose=Pose(position=Point(-3.632812, 1.359375, 0.0), orientation=Quaternion(0.0, 0.0, 0.687581, 0.726107))),
    "house2": PoseStamped(header=Header(frame_id="map"), pose=Pose(position=Point(-5.257812, 0.640625, 0.0), orientation=Quaternion(0.0, 0.0, 0.999908, 0.013509))),
    "house3": PoseStamped(header=rospy.Header(frame_id="map"), pose=Pose(position=Point(-0.539999, 0.959999, 0.0), orientation=Quaternion(0.0, 0.0, 0.700468, 0.713683)))
}

CONTAINERS: Dict[str, PoseStamped] = {
    "yellow": PoseStamped(header=Header(frame_id="map"), pose=Pose(position=Point(-0.570313, -0.234375, 0.0), orientation=Quaternion(0.0, 0.0, -0.716857, 0.697221))),
    "blue":   PoseStamped(header=Header(frame_id="map"), pose=Pose(position=Point(-2.601562, -0.109374, 0.0), orientation=Quaternion(0.0, 0.0, -0.722982, 0.690866))),
    "green":  PoseStamped(header=Header(frame_id="map"), pose=Pose(position=Point(-4.664062, 0.078125, 0.0), orientation=Quaternion(0.0, 0.0, -0.707106, 0.707106)))
}

# ---------------------------  FSM  -----------------------------------------
class State:
    MOVING_TO_HOUSE      = "MOVING_TO_HOUSE"
    SCANNING_AT_HOUSE    = "SCANNING_AT_HOUSE"
    MOVING_TO_CONTAINER  = "MOVING_TO_CONTAINER"

# ----------------------  ORCHESTRATOR  -------------------------------------
class Orchestrator:
    def __init__(self):
        self.state: str | None = None
        self.lock = threading.RLock()
        self.current_color: str | None = None
        self.last_house: str | None = None
        self.scan_deadline = rospy.Time()  # deadline for the current scan

        # --- Publishers / Subscribers ---
        self.pub_move_goal = rospy.Publisher("/move_command", PoseStamped, queue_size=10)
        self.pub_move_ctrl = rospy.Publisher("/move_ctrl", String, queue_size=10)
        self.pub_vision    = rospy.Publisher("/vision_active", Bool,   queue_size=10)

        rospy.Subscriber("/detected_color", String, self.cb_detected_color)
        rospy.Subscriber("/move_status",     String, self.cb_move_status)

        # Periodic timer to watch scan timeout
        rospy.Timer(rospy.Duration(0.5), self.timer_scan_watchdog)

        rospy.sleep(2.0)  # allow pubs to register
        rospy.loginfo("[Orch] Orchestrator initialized")
        self.goto_next_house()

    # ---------- Helper I/O ----------
    def set_vision(self, flag: bool):
        self.pub_vision.publish(Bool(data=flag))
        rospy.loginfo("[Vision] %s", "ON" if flag else "OFF")

    def send_goal(self, pose: PoseStamped):
        pose.header.stamp = rospy.Time.now()
        self.pub_move_goal.publish(pose)
        p = pose.pose.position
        rospy.loginfo("[Nav] Goal → (%.2f, %.2f)", p.x, p.y)

    def stop_robot(self):
        self.pub_move_ctrl.publish(String(data="stop"))
        rospy.loginfo("[Nav] STOP sent")

    # ---------- FSM helpers ----------
    def choose_new_house(self) -> str:
        keys = list(HOUSES.keys())
        if self.last_house and len(keys) > 1:
            keys.remove(self.last_house)
        return random.choice(keys)

    def goto_next_house(self):
        with self.lock:
            house = self.choose_new_house()
            self.last_house = house
            self.current_color = None
            self.set_vision(False)  # vision off while driving
            self.state = State.MOVING_TO_HOUSE
            self.send_goal(HOUSES[house])
            rospy.loginfo("[FSM] → MOVING_TO_HOUSE (%s)", house)

    def start_scanning(self):
        self.set_vision(True)
        self.scan_deadline = rospy.Time.now() + rospy.Duration(SCAN_DURATION)
        self.state = State.SCANNING_AT_HOUSE
        rospy.loginfo("[FSM] → SCANNING_AT_HOUSE (%.1fs)", SCAN_DURATION)

    # ---------- Callbacks ----------
    def cb_move_status(self, msg: String):
        with self.lock:
            if msg.data != "arrived":
                return

            if self.state == State.MOVING_TO_HOUSE:
                self.start_scanning()

            elif self.state == State.MOVING_TO_CONTAINER:
                rospy.loginfo("[Nav] Arrived at container; back to houses")
                self.goto_next_house()

    def cb_detected_color(self, msg: String):
        with self.lock:
            if self.state != State.SCANNING_AT_HOUSE:
                return  # ignore detections outside the scanning state

            color = msg.data.lower()
            if color in CONTAINERS:
                rospy.loginfo("[Vision] Detected color %s", color)
                self.current_color = color
                self.set_vision(False)
                self.state = State.MOVING_TO_CONTAINER
                self.send_goal(CONTAINERS[color])
                rospy.loginfo("[FSM] → MOVING_TO_CONTAINER (%s)", color)

    # ---------- Timer watchdog ----------
    def timer_scan_watchdog(self, _):
        with self.lock:
            if self.state == State.SCANNING_AT_HOUSE and rospy.Time.now() > self.scan_deadline:
                rospy.loginfo("[Vision] Scan timeout – no ball detected; continuing")
                self.goto_next_house()

# -----------------------------  MAIN  --------------------------------------

def main():
    rospy.init_node("orchestrator_node")
    Orchestrator()
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
