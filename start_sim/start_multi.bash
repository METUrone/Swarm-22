source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default &&
source ~/turtle_ws/devel/setup.bash &&
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd):$(pwd)/Tools/sitl_gazebo &&
roslaunch px4 multi_uav_mavros_sitl.launch
