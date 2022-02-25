#!/usr/bin/env python3

from Swarm import Swarm 
from TurtleBot import TurtleBot

import numpy as np


if __name__ == "__main__":


    vehicle1 = "Iris"
    vehicle2 = "Crazyflie"
    swarm = Swarm(6, vehicle2)
    swarm.form_via_potential_field(3)
    #swarm.omit_agent()
    swarm.land()
    swarm.timeHelper.sleep(4)
    #swarm.return_starting_pose()