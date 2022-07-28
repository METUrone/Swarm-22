#!/usr/bin/env python3

"""
eksikler:

land fonk
3d formasyon land
3d formasyon bazen hata veriyo
formasyondan 0. ajan çikarsa sim duruyo ama log devam ediyo ?
"""

from Swarm import Swarm 
from TurtleBot import TurtleBot
from utils import *
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
    

    mod = int(input("1) Plan mission\n2) Execute mission"))
    mission = ""

    if mod == 1:
        clear_mission()
        uav_count = int(input("Enter UAV number: "))
        write_mission(uav_count)

    elif mod == 2:
        mission = read_mission()
        uav_count = int(mission[0][0])
        del mission[0]

    distance_between_agents = 3
    vehicle1 = "Iris"
    vehicle2 = "Crazyflie"
    swarm = Swarm(uav_count, vehicle2)

    swarm.timeHelper.sleep(2)

        

    mission_numbers = """
    1) 3D fromation
    2) Add remove agent
    2) Obstacle avoidance
    3) Cargo
    4) Fire fighting
    """
    
    print(mission_numbers)

    if mod == 1:
        mission_num = int(input("Enter mission number: "))

        write_mission(mission_num)
    elif mod == 2:
        mission_num = int(mission[0][0])
        print(mission_num)
        del mission[0]
    

    if mission_num == 1:
        is_mission_running = True
        mission1_options = """
        1: PYRAMID
        2: PRISM
        3: LAND
        4: GO
        5: ROTATE
        6: SLEEP
        """

        while is_mission_running:
            
            if mod == 1:
                sub_mission = int(input(mission1_options))
                write_mission(sub_mission)
            elif mod == 2:
                sub_mission = int(mission[0][0])
                print(sub_mission)
                del mission[0]

            if sub_mission == 1:
                
                if mod == 1:
                    d = float(input("Enter distance between agents:"))
                    write_mission(d)
                elif mod == 2:
                    d = float(mission[0][0])
                    del mission[0]

                swarm.form_3d(d, "prism")

            elif sub_mission == 2:
                
                if mod == 1:
                    d = float(input("Enter distance between agents:"))
                    write_mission(d)
                elif mod == 2:
                    d = float(mission[0][0])
                    del mission[0]

                
                if mod == 1:
                    edge = int(input("Enter edge number:"))
                    write_mission(edge)
                elif mod == 2:
                    edge = int(mission[0][0])
                    del mission[0]
                
                swarm.form_3d(d, edge)

            elif sub_mission == 3:
                swarm.hover(2)
                swarm.land_swarm()
                swarm.timeHelper.sleep(2)
                is_mission_running = False
            
            elif sub_mission == 4:
                
                if mod == 1:
                    vector = str(input("Direction vector: "))
                    write_mission(vector)
                    vector = vector.split(" ")
                elif mod == 2:
                    vector = [mission[0][0], mission[0][1], mission[0][2]]
                    del mission[0]

                
                swarm.go(np.array([float(vector[0]), float(vector[1]), float(vector[2])]))

            elif sub_mission == 5:
                
                if mod == 1:
                    angle = float(input("Enter angle in terms of degree: "))
                    write_mission(angle)
                elif mod == 2:
                    angle = float(mission[0][0])
                    del mission[0]

                swarm.rotate(angle) 

            elif sub_mission == 6:
                if mod == 1:
                    duration = float(input("Sleep duration: "))
                    swarm.hover(duration)
                    write_mission(duration)
                elif mod == 2:
                    duration = float(mission[0][0])
                    swarm.hover(duration)
                    del mission[0]

            else:
                print("INVALID ARGUMENT!!!")
            
            swarm.hover(0.01)
        


    elif mission_num == 2:

        if mod == 1:
            d = float(input("Enter distance between agents:"))
            write_mission(d)
        if mod == 2:
            d = float(mission[0][0])
            del mission[0]


        swarm.form_via_potential_field(d, dimension=2 , z=0.5)

        mission2_options = """
        1: LAND
        2: ADD AGENT
        3: REMOVE AGENT
        4: GO
        5: ROTATE
        """

        is_mission_running = True

        

        while is_mission_running:

            if mod == 1:
                sub_mission = int(input(mission2_options))
                write_mission(sub_mission)
            elif mod == 2:
                sub_mission = int(mission[0][0])
                del mission[0]

            if sub_mission == 1:
                swarm.land()
                swarm.hover(5)
                break

            if sub_mission == 2:
                swarm.add_agent_to_formation(dimension=2)

            if sub_mission == 3:
                agent_ids = "Select id:\n"
                for i in range(len(swarm.agents)):
                    agent_ids += str(i)
                    agent_ids += str(swarm.agents[i].position())
                    agent_ids += "\n"

                if mod == 1:
                    id = int(input(agent_ids))
                    write_mission(id)
                elif mod == 2:
                    id = int(mission[0][0])
                    del mission[0]


                swarm.omit_agent_by_id(id)

            if sub_mission == 4:
                if mod == 1:
                    vector = str(input("Direction vector: "))
                    write_mission(vector)
                    vector = vector.split(" ")
                elif mod == 2:
                    vector = [mission[0][0], mission[0][1], mission[0][2]]
                    del mission[0]

                
                swarm.go(np.array([float(vector[0]), float(vector[1]), float(vector[2])]))

            elif sub_mission == 5:
                
                if mod == 1:
                    angle = float(input("Enter angle in terms of degree: "))
                    write_mission(angle)
                elif mod == 2:
                    angle = float(mission[0][0])
                    del mission[0]

                swarm.rotate(angle) 

            else:
                print("INVALID ARGUMENT!!!")
            
            swarm.hover(0.01)
    
    elif mission_num == 3:
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
            
            swarm.hover(0.01)

    elif mission_num == 4:
        pass
    elif mission_num == 5:
        pass

    swarm.log_to_csv()