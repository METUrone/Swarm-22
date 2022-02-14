#!/usr/bin/env python3

from Swarm import Swarm 
import time
import sys
import rospy

if __name__ == "__main__":
    
    num_of_agents = 5
    vehicle = "Iris"

    swarm = Swarm(num_of_agents, vehicle)

    try:

        if sys.argv[1]=="1":
        
            #swarm.form_via_pose(4)
            swarm.form_via_potential_field(4)
            #swarm.single_potential_field(3, 2)
            print("square")
            time.sleep(10)
            #swarm.swarm_square()
            #time.sleep(15)
        else:
            swarm.return_starting_pose()

    except rospy.exceptions.ROSInterruptException:
        print("shutdown")