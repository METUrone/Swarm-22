#!/usr/bin/env python3

from Swarm import Swarm 
from TurtleBot import TurtleBot
from cluster import Cluster

import numpy as np
import os

def clear_log():
    for i in range(50):
        try:
            os.remove('./agent{}_logs.csv'.format(i))
        except FileNotFoundError:
            return

if __name__ == "__main__":

    clear_log()
    uav_count = 10
    radius = 3
    vehicle1 = "Iris"
    vehicle2 = "Crazyflie"
    swarm = Swarm(6, vehicle2)
    # swarm.form_3d(1, "prism")
    #swarm.form_via_potential_field(2)

    swarm.form_3d(1,3)

    swarm.timeHelper.sleep(5)

    #swarm.star_formation()
    # swarm.timeHelper.sleep(0.5)
    # swarm.form_via_potential_field(radius)
    # swarm.timeHelper.sleep(1)
    # swarm.add_agent_to_formation()
    # swarm.timeHelper.sleep(1)
    # swarm.add_agent_to_formation()
    # swarm.omit_agent()
    # swarm.omit_agent()
    # swarm.omit_agent()
    # swarm.omit_agent()
    # swarm.omit_agent()
    # swarm.omit_agent()
    # swarm.omit_agent()
    #swarm.go(np.array([radius,0,0]))
    #swarm.go(np.array([0,-radius,0]))
    #swarm.go(np.array([-radius,0,0]))
    #swarm.go(np.array([0,radius,0]))
    #swarm.rotate()
    swarm.timeHelper.sleep(2)
    swarm.land()
    #swarm.form_3d(2, "prism")
    #swarm.swarm_square(2)
    #swarm.form_pyramid()
    #swarm.add_agent_to_formation()
    #swarm.timeHelper.sleep(2)

    #for i in range(uav_count):
    #    swarm.omit_agent()
    
    #swarm.swarm_square(2)
    #swarm.rotate()