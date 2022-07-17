#!/usr/bin/env python3

from Swarm import Swarm 
from TurtleBot import TurtleBot

import numpy as np
import os

def clear_log():
    for i in range(50):
        try:
            os.remove('./agent{}_logs.csv'.format(i))
        except FileNotFoundError:
            return



if __name__ == "__main__":
    clear_log()
    uav_count = int(input("Enter UAV number: "))
    distance_between_agents = 3
    vehicle1 = "Iris"
    vehicle2 = "Crazyflie"
    swarm = Swarm(uav_count, vehicle2)

    mission_numbers = """
    1) 3D fromation
    2) Add remove agent
    2) Obstacle avoidance
    3) Cargo
    4) Fire fighting
    """
    
    print(mission_numbers)
    mission = int(input("Enter mission number: "))
    

    if mission == 1:
        is_mission_running = True
        mission1_options = """
        1: PYRAMID
        2: PRISM
        3: LAND
        4: GO
        5: ROTATE
        """

        while is_mission_running:
            sub_mission = int(input(mission1_options))

            if sub_mission == 1:
                d = float(input("Enter distance between agents:"))
                swarm.form_3d(d, "prism")

            elif sub_mission == 2:
                d = float(input("Enter distance between agents:"))
                edge = int(input("Enter edge number:"))
                swarm.form_3d(d, edge)

            elif sub_mission == 3:
                swarm.land()
                swarm.timeHelper.sleep(3)
                is_mission_running = False
            
            elif sub_mission == 4:
                vector = str(input("Direction vector: "))
                vector = vector.split(" ")
                swarm.go(np.array([float(vector[0]), float(vector[1]), float(vector[2])]))

            elif sub_mission == 5:
                angle = float(input("Enter angle in terms of degree: "))
                swarm.rotate() 


    elif mission == 2:
        mission2_options = """
        1: LAND
        2: ADD AGENT
        3: REMOVE AGENT
        4: GO
        5: ROTATE
        """

        is_mission_running = True

        

        while is_mission_running:

            sub_mission = int(input(mission2_options))

            if sub_mission == 1:
                swarm.land()
                swarm.timeHelper.sleep(5)
                break

            if sub_mission == 2:
                swarm.add_agent_to_formation()

            if sub_mission == 3:
                swarm.omit_agent()

            if sub_mission == 4:
                vector = str(input("Direction vector: "))
                vector = vector.split(" ")
                swarm.go(np.array([float(vector[0]), float(vector[1]), float(vector[2])]))

            if sub_mission == 5:
                pass
    
    elif mission == 3:
        sub_missions = """
        1: Add obstacle
        2: Remove last obstacle
        3: Enter displacement vector
        4: Start mission
        """

        obstacles = []
        vector = []

        is_mission_running = True

        while is_mission_running:

            sub_mission = int(input(sub_missions))

            if sub_mission == 1:

                pose = str(input("Position of the obstacle: "))
                pose = pose.split(" ")
                obstacle = np.array([float(pose[0]), float(pose[1]), float(pose[2])])
                obstacles.append(obstacle)

                print("Obstacles: " + str(obstacles))

            if sub_mission == 2:

                obstacles.pop()
                print(obstacles)

            if sub_mission == 3:

                vector = str(input("Position of the obstacle: "))
                vector = vector.split(" ")
                vector = np.array([float(vector[0]), float(vector[1]), float(vector[2])])

                swarm.obstacle_creator_without_drones(obstacles)

            if sub_mission == 4:
                swarm.go(vector)

    elif mission == 4:
        pass
    elif mission == 5:
        pass
