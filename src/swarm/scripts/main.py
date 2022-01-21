#!/usr/bin/env python3

from Iris import Iris 
from threading import Thread
import math


drones = []

def add_drone(id):
    global drones
    drones.append(Iris(id))

def swarm_square(num_of_drones):
    for i in range(num_of_drones):
        Thread(target = drones[i].draw_square, args=(5,)).start()

def distance_of_drones(drone_a, drone_b): # in meters

    x_distance = (drone_a.gps_pose_getter().latitude-drone_b.gps_pose_getter().latitude)*111139 
    y_distance = 40075*math.cos(drone_a.gps_pose_getter().longitude-drone_b.gps_pose_getter().longitude)/360

    print(x_distance)
    print(y_distance)

    
    

if __name__ == "__main__":
    
    num_of_drones = 4
    simulation_origin_latitude = 47.3977419
    simulation_origin_longitude = 8.5455935

    for i in range(num_of_drones):
        add_drone(i)

    print("-----------\n")

    #distance_of_drones(drones[1], drones[0])
    swarm_square(num_of_drones)
    