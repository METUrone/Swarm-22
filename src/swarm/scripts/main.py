#!/usr/bin/env python3

from Iris import Iris 
from Swarm import Swarm 
import time


if __name__ == "__main__":
    
    num_of_agents = 3
    vehicle = "Iris"

    swarm = Swarm(num_of_agents, vehicle)
    
    swarm.form_via_pose(num_of_agents)
    time.sleep(3)
    swarm.swarm_square()
    time.sleep(15)
    swarm.return_starting_pose()
