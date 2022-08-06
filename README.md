# Aim

This project aims to provide an OOP based heterogeneous swarm system independent of the vehicle types and motion dynamics. Currently we are using the system with crazyflies.

## Installation

First of all, you need to install [ROS noetic](http://wiki.ros.org/noetic/Installation/Ubuntu) and [crazyswarm](https://crazyswarm.readthedocs.io/en/latest/installation.html). Choose the crazyswarm version for physical robot and simulation. <br /><br />
Build workspace.
```
mkdir -p ~/workspace_name/src
cd ~/workspace_name
catkin_make
```

Clone the ROS package.
```
cd ~/workspace_name/src
git clone https://github.com/METUrone/Swarm-22.git
cd ~/workspace_name
catkin_make
```


## Quick Code Usage

Source the workspace.

```
source ~/workspace_name/devel/setup.bash
```

Since crazyswarm searches for .yaml file containing the crazyflie information in the parent path, python script should be run in the scripts directory.
  
```
cd ~/workspace_name/src/Swarm-22/scripts
```

There are two options to start the script. If `--sim` is added after the script name, code will run on simulation mod. If not, crazyflies should start the mission in real-life.

```
rosrun swarm main.py --sim
```

Important Points:

- The crazyflies.yaml file in crazyswarm workspace (`~/crazyswarm/ros_ws/src/crazyswarm/launch/crazyflies.yaml` ) and the one in the swarm package should contain the same information if the script in running in real-life.

- Crazyflies positions in real-life must be as close as possible to the positions which are determined in crazyflies.yaml files.


