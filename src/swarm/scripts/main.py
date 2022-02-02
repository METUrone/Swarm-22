#!/usr/bin/env python3

from Iris import Iris 
from threading import Thread
import math
import time


drones = [] 

def add_drone(id):
    global drones
    drones.append(Iris(id))

def swarm_square(num_of_drones):
    for i in range(num_of_drones):
        Thread(target = drones[i].draw_square, args=(5,)).start()

def distance_of_drones(drone_a, drone_b): # in meters

    lat_a = drone_a.gps_pose_getter().latitude
    lat_b = drone_b.gps_pose_getter().latitude
    x_distance = 111139 * (lat_a - lat_b)
    long_a = drone_a.gps_pose_getter().longitude
    long_b = drone_b.gps_pose_getter().longitude
    R = 63678.137

    dlat = lat_b * math.pi / 180 - lat_a * math.pi/180 
    dlon = long_b * math.pi / 180 - long_a * math.pi/180 
    a = math.sin(dlat/2) * math.sin(dlat/2) + math.cos(lat_a*math.pi / 180) * math.cos(lat_b * math.pi / 180) * math.sin(dlon/2) * math.sin(dlon/2)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    d = R * c
    abs_distance = d * 100

    y_distance = math.sqrt(abs_distance**2-x_distance**2)
    #print(abs_distance)
    #print(x_distance) 
    #print(y_distance)
    return x_distance, y_distance, abs_distance

def distance_of_drones(drone_a, lat_b, long_b): # in meters

    lat_a = drone_a.gps_pose_getter().latitude
    x_distance = 111139 * (lat_a - lat_b)
    long_a = drone_a.gps_pose_getter().longitude
    R = 63678.137

    dlat = lat_b * math.pi / 180 - lat_a * math.pi/180 
    dlon = long_b * math.pi / 180 - long_a * math.pi/180 
    a = math.sin(dlat/2) * math.sin(dlat/2) + math.cos(lat_a*math.pi / 180) * math.cos(lat_b * math.pi / 180) * math.sin(dlon/2) * math.sin(dlon/2)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    d = R * c
    abs_distance = d * 100

    y_distance = math.sqrt(abs_distance**2-x_distance**2)

    x_coefficient = 1
    y_coefficient = 1

    if(lat_a < lat_b):
        x_coefficient = -1
    if(long_a < long_b):
        y_coefficient = -1

    return x_distance*x_coefficient, y_distance*y_coefficient, abs_distance

def prin_drones_pose(drones):
    j = 0
    for i in drones:
        print(j)
        i.get_global_pose()
        j += 1
    print("------------")
    time.sleep(1)
    
def swarm_go_starting_point(drones):
    for i in drones:
        i.move_local(0,0,5)

if __name__ == "__main__":
    
    num_of_drones = 3
    simulation_origin_latitude = 47.3977419
    simulation_origin_longitude = 8.5455935

    for i in range(num_of_drones):
        add_drone(i)
        pose = distance_of_drones(drones[i], simulation_origin_latitude, simulation_origin_longitude)
        print(pose[0], pose[1])
        drones[i].set_starting_pose(pose[0], pose[1]) #Agent must be in the starting position (locally 0, 0) height does not matter
        drones[i].get_global_pose()

    print("-----------\n")

    #swarm_square(num_of_drones)
    
    #swarm_go_starting_point(drones)
    drones[1].velocity_command(0)
    time.sleep(5)
    
    
    