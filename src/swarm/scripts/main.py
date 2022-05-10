#!/usr/bin/env python3

from Swarm import Swarm 
from TurtleBot import TurtleBot
import time

import numpy as np
from utils import *




if __name__ == "__main__":
    clear_log()
    uav_count = 6
    radius = 1.5
    vehicle1 = "Iris"
    vehicle2 = "Crazyflie"
    obstancles1 = [[5,0,5],[5,1.5,5],[5,-1.5,5],[5,1.25,5],[5,-1.25,5]]
    obstancles2 = [[0,5,5],[0.5,5,5],[-0.5,5,5],[0.25,5.25,5],[-0.25,5,5]]
    obstancles3 = [[3,3,2]]


    swarm = Swarm(uav_count, vehicle2)
    #swarm.star_formation()
    """swarm.timeHelper.sleep(0.5)"""
    #swarm.form_via_potential_field(radius)
    """
    #swarm.go(np.array([radius,0,0]))
    #swarm.go(np.array([0,-radius,0]))
    #swarm.go(np.array([-radius,0,0]))
    #swarm.go(np.array([0,radius,0]))
    #swarm.rotate()
    swarm.timeHelper.sleep(2)
    swarm.form_3d(2, "prism")
    #swarm.swarm_square(2)
    #swarm.form_pyramid()
    #swarm.add_agent_to_formation()
    #swarm.timeHelper.sleep(2)

    #for i in range(uav_count):
    #    swarm.omit_agent()
    
    #swarm.swarm_square(2)
    #swarm.rotate()"""
    #swarm = Swarm(uav_count, vehicle2)
    swarm1, swarm2 = swarm.split_formation()
    swarm1.go(np.array([1,0,0]))
    swarm2.go(np.array([1,0,0])) 
    
    """ swarm.return_starting_pose() """
    """ time.sleep(5) """
    """ swarm.form_via_potential_field(3) """
    """ swarm.go_pid(np.array([5,0,0]), obstancles3) """
    """ swarm.star_formation() """
    """ swarm.form_3d(3, 5) """

    """ swarm.timeHelper.sleep(10) """
    #swarm1, swarm2 = swarm.split_formation()
    #swarm.omit_agent()
    #swarm.timeHelper.sleep(4)
    swarm.log_to_csv()
    swarm.land()
    #swarm.return_starting_pose()
    
    swarm.timeHelper.sleep(4)
    
