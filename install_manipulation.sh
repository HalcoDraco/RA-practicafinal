cd ~/catkin_ws/src/
git clone -b noetic https://github.com/ROBOTIS-GIT/turtlebot3_manipulation.git
git clone -b noetic https://github.com/ROBOTIS-GIT/turtlebot3_manipulation_simulations.git
git clone -b noetic https://github.com/ROBOTIS-GIT/open_manipulator_dependencies.git
sudo apt install ros-noetic-ros-control* ros-noetic-control* ros-noetic-moveit* ros-noetic-dwa-local-planner
cd ~/catkin_ws && catkin_make
