cd ~/catkin_ws/src

catkin_create_pkg robotic_arm_movement_node \
  roscpp \
  moveit_core \
  moveit_ros_planning_interface \
  std_msgs \
  actionlib