#!/usr/bin/env python3

from Swarm import Swarm 
from TurtleBot import TurtleBot

import numpy as np


if __name__ == "__main__":


    vehicle1 = "Iris"
    vehicle2 = "Crazyflie"
    swarm = Swarm(10, vehicle2)
    swarm.form_via_potential_field(3)
    swarm.form_3d(3, 5)
    swarm.timeHelper.sleep(10)
    #swarm1, swarm2 = swarm.split_formation()
    #swarm.omit_agent()
    swarm.land()
    swarm.timeHelper.sleep(4)
    #swarm.return_starting_pose()