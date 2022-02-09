#!/usr/bin/env python3

from Swarm import Swarm 
import time
from TurtleBot import TurtleBot


if __name__ == "__main__":
    
    #num_of_agents = 3
    #vehicle = "Iris"
#
    #swarm = Swarm(num_of_agents, vehicle)
    #
    #swarm.form_via_pose(4)
    ##swarm.form_via_potential_field(3)
    ##swarm.single_potential_field(3, 2)
    #print("square")
    #time.sleep(10)
    #swarm.swarm_square()
    ##time.sleep(15)
    ##swarm.return_starting_pose()
    turtle = TurtleBot(0)
    turtle.velocity(linear_x = 0)