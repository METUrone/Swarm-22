#!/usr/bin/env python3

from Iris import Iris 
from Swarm import Swarm 
import time


if __name__ == "__main__":
    
    num_of_agents = 6
    vehicle = "Iris"

    swarm = Swarm(num_of_agents, vehicle)
    
    swarm.return_starting_pose()

    swarm.form_via_pose(num_of_agents)
    
    if input("başlangıç noktasına dönmek için q ya basınız: ") == "q":
        swarm.return_starting_pose()