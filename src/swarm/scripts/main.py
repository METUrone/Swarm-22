#!/usr/bin/env python3

from Swarm import Swarm 
from TurtleBot import TurtleBot

import numpy as np


if __name__ == "__main__":


    vehicle1 = "Iris"
    vehicle2 = "Crazyflie"
    swarm = Swarm(5, vehicle2)
    swarm.form_via_potential_field(2.5)
    swarm.add_agent_to_formation()
    #swarm.omit_agent()
    swarm.go(np.array([1,2,0]))
    swarm.land()
    swarm.timeHelper.sleep(4)
    #swarm.return_starting_pose()