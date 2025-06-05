# Download from Drive the map package and copy it to the catkin workspace

cd ~/catkin_ws/src
catkin_create_pkg send_goal rospy move_base_msgs actionlib
mkdir ~/catkin_ws/src/send_goal/scripts
touch ~/catkin_ws/src/send_goal/scripts/goal_subscriber_client.py
# Copiar el contenido del archivo goal_subscriber_client.py
chmod +x ~/catkin_ws/src/send_goal/scripts/goal_subscriber_client.py
mkdir ~/catkin_ws/src/send_goal/launch
touch ~/catkin_ws/src/send_goal/launch/send_goal_from_topic.launch
# Copiar el contenido del archivo send_goal_from_topic.launch
cm
