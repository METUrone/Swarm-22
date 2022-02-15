#!/usr/bin/env python3

from Swarm import Swarm 
import time
import sys
import rospy

if __name__ == "__main__":
    
    num_of_agents = 6
    vehicle = "Iris"

    swarm = Swarm(num_of_agents, vehicle)

        
    #swarm.form_via_pose(4)
    swarm.sort_coordinates(swarm.formation_coordinates(num_of_agents))
    #swarm.form_via_potential_field(num_of_agents)
    #swarm.single_potential_field(3, 2)
    print("square")
    time.sleep(10)
    #swarm.swarm_square()
    #time.sleep(15)
    swarm.return_starting_pose()

    print("shutdown")