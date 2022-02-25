#!/usr/bin/env python3

from threading import Thread
from Swarm import Swarm 
import time
import sys
import rospy
from TurtleBot import TurtleBot




if __name__ == "__main__":


    vehicle1 = "Iris"
    vehicle2 = "Crazyflie"
    swarm = Swarm(3, vehicle2)
    swarm.form_via_potential_field(4)
    #swarm.return_starting_pose()