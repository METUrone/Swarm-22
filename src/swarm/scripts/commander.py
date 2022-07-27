#!/usr/bin/env python3

from Swarm import Swarm 
import rospy

import numpy as np
import os


from pycrazyswarm import Crazyswarm
from utils import *

from custom_msg.msg import general_parameters #import custom_msg which is in the workspace
from swarm.srv import SwarmCmd,SwarmCmdResponse

velocities = []
cmdType = ""
cmdValue = 0


def srv_handler(req):
    global velocities, cmdType, cmdValue
    res = SwarmCmdResponse()

    velocities = req.vels
    cmdValue = req.cmdValue
    cmdType = req.cmdType

    return res


def main():
    s = rospy.Service('/swarm/cmd', SwarmCmd, srv_handler)
    num_of_drones = len(velocities)

    crazyswarm = Crazyswarm()
    agents = crazyswarm.allcfs.crazyflies
    agentsById = crazyswarm.allcfs.crazyfliesById
    timeHelper = crazyswarm.timeHelper
    
    while not rospy.is_shutdown():
        # if cmdType == "velocities":
        #     for i in range(agents):
        #         agents[i].cmdVelocityWorld(np.array([velocities[i][0], velocities[i][1], velocities[i][2] ]))
        # elif cmdType == "takeoff":
        #     for agent in agents:
        #         agent.takeoff(targetHeight=cmdValue, duration=1)
        if not cmdType == "":
            print(cmdType)
            

if __name__ == "__main__":
    main()