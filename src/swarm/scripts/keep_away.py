#!/usr/bin/env python3

from Iris import Iris 
from Swarm import Swarm 
import time


if __name__ == "__main__":
    
    num_of_agents = 9
    vehicle = "Iris"

    swarm = Swarm(num_of_agents, vehicle)
    
    swarm.move_with_swarm(1, 1000000, (-10, -10), 0.1)
