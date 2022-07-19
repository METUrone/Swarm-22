#!/usr/bin/env python3

from Swarm import Swarm 
from TurtleBot import TurtleBot
from cluster import Cluster

import numpy as np
import os

def clear_log():
    for i in range(50):
        try:
            os.remove('./agent{}_logs.csv'.format(i))
        except FileNotFoundError:
            return



if __name__ == "__main__":
    # cluster = Cluster(3, 3)
    # cluster.initailize_positions([[-4.0, -4.0 ,1.0], [-4.0, 4.0, 1.0], [4.0, 4.0, 1.0]])
    # cluster.deliver_via_potential_field([[4.0, 4.0 ,1.0], [4.0, -4.0, 1.0], [-4.0, -4.0, 1.0]])
    
    
    
    
    clear_log()
    uav_count = 10
    radius = 2
    vehicle1 = "Iris"
    vehicle2 = "Crazyflie"
    swarm = Swarm(uav_count, vehicle2)
    #swarm.star_formation()
    swarm.timeHelper.sleep(5)
    print("take offf")
    
    # swarm.form_3d(radius, "prism")
    # #swarm.go(np.array([1,0,0]))
    # swarm.rotate(120, step=20, duration=5)
    # swarm.land()
    swarm.form_via_potential_field(radius)
    swarm.timeHelper.sleep(1)
    swarm.go([3, 3, 0])
    swarm.obstacle_creator(5)
    swarm.form_polygon(2, 5, 1, [-3, -3, 0])
    swarm.form_polygon(2, 5, 1, [3, 3, 0])


    # swarm.timeHelper.sleep(1)
    # swarm.go([3, 3, 0])
    # swarm.obstacle_creator(7)
    # swarm.form_polygon(2, 3, 1, [-3, -3, 0])
    # swarm.form_polygon(2, 3, 1, [3, 3, 0])
    #swarm.go(np.array([radius,0,0]))
    #swarm.go(np.array([0,-radius,0]))
    #swarm.go(np.array([-radius,0,0]))
    #swarm.go(np.array([0,radius,0]))
    #swarm.rotate()

    # for i in range(5):
    #     swarm.go(np.array([-0.1,0,0]))

    # swarm.hover(1.0)

    
    #swarm.form_3d(2, "prism")
    #swarm.swarm_square(2)
    #swarm.form_pyramid()
    #swarm.add_agent_to_formation()

    #swarm.hover(1.0)

    #swarm.omit_agent()

    #swarm.hover(1.0)
    #swarm.timeHelper.sleep(2)

    #for i in range(uav_count):
    #    swarm.omit_agent()
    
    #swarm.swarm_square(2)
    #swarm.rotate()

    #swarm1, swarm2 = swarm.split_formation()
    #swarm1.go(np.array([1,0,0]))
    #swarm2.go(np.array([1,0,0]))
    #swarm.omit_agent()
    #swarm.timeHelper.sleep(4)
    swarm.log_to_csv()
    
    swarm.timeHelper.sleep(4)
    #swarm.return_starting_pose()
    #swarm.return_starting_pose()
    