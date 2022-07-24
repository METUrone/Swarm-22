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
    # cluster = Cluster(3, 3)
    # cluster.initailize_positions([[-4.0, -4.0 ,1.0], [-4.0, 4.0, 1.0], [4.0, 4.0, 1.0]])
    # cluster.deliver_via_potential_field([[4.0, 4.0 ,1.0], [4.0, -4.0, 1.0], [-4.0, -4.0, 1.0]])
    
    
    
    
    clear_log()
    uav_count = 6
    radius = 2
    vehicle1 = "Iris"
    vehicle2 = "Crazyflie"
    swarm = Swarm(uav_count, vehicle2)
    
    swarm.takeoff(1)
    swarm.form_3d(1, 3)
    swarm.land_prism(1)
    swarm.timeHelper.sleep(10)

    swarm.log_to_csv()
    
    swarm.timeHelper.sleep(4)
    
    