#!/usr/bin/env python3

from threading import Thread
from Swarm import Swarm 
import time
import sys
import rospy
from TurtleBot import TurtleBot



if __name__ == "__main__":


    vehicle = "Iris"
    swarm = Swarm(3, vehicle)
    #swarm.form_via_potential_field(4)
    swarm.return_starting_pose()