cd ~/catkin_ws/src/
git clone -b noetic https://github.com/ROBOTIS-GIT/turtlebot3_manipulation.git
git clone -b noetic https://github.com/ROBOTIS-GIT/turtlebot3_manipulation_simulations.git
git clone -b noetic https://github.com/ROBOTIS-GIT/open_manipulator_dependencies.git
sudo apt install ros-noetic-ros-control* ros-noetic-control* ros-noetic-moveit* ros-noetic-dwa-local-planner
cd ~/catkin_ws && catkin_make


# if sudo apt install ros-noetic-ros-control* ros-noetic-control* ros-noetic-moveit* ros-noetic-dwa-local-planner
# fails, try the following:
sudo rm -rf /var/lib/apt/lists/*
sudo apt clean
sudo apt update --allow-insecure-repositories
sudo apt install -y --allow-unauthenticated ros-noetic-ros-control* ros-noetic-control* ros-noetic-moveit* ros-noetic-dwa-local-planner
# catkin make (cm)


# si el brazo no se yergue, ejecutar lo siguiente dentro del robot (con ssh):
export OPENCR_PORT=/dev/ttyACM0
export OPENCR_MODEL=om_with_tb3_noetic
rm -rf ./opencr_update.tar.bz2
wget https://github.com/ROBOTIS-GIT/OpenCR-Binaries/raw/master/turtlebot3/ROS1/latest/opencr_update.tar.bz2
tar -xvf opencr_update.tar.bz2
cd ./opencr_update
./update.sh $OPENCR_PORT $OPENCR_MODEL.opencr

rostopic pub /orquestator_manipulation std_msgs/String "data: 'pick'" --once
