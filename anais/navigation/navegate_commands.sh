roscore
roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch
roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=PATH DEL MAPA
roslaunch turtlebot3_navigation amcl.launch initial_pose_x:=0.41 initial_pose_y:=-0.23 initial_pose_a:=0.0
roslaunch send_goal send_goal_from_topic.launch