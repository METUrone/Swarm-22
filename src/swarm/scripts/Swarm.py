#!/usr/bin/env python3

import math
import time
from Iris import Iris
from threading import Thread
from copy import deepcopy

class Swarm:
    def __init__(self,num_of_drones,vehicle):
        self.agents = []
        self.vehicle = vehicle

        if vehicle == "Iris":
            for i in range(num_of_drones):
                self.add_drone(i)

        self.num_of_agents = len(self.agents)

        simulation_origin_latitude = 47.3977419
        simulation_origin_longitude = 8.5455935

        for i in range(len(self.agents)):
            pose = self.distance_to_pose(drone_a=self.agents[i], lat_b=simulation_origin_latitude, long_b=simulation_origin_longitude)
            print(self.agents[i].id)
            self.agents[i].set_starting_pose(pose[0], pose[1]) #Agent must be in the starting position (locally 0, 0) height does not matter
            self.agents[i].get_global_pose()

    def formation_coordinates(self, radius):
        vector = [0,0]
        vectors = []
        for i in range(self.num_of_agents):
            vectors.append(deepcopy(vector))

        
        vectors[0][0]=0
        vectors[0][1]=radius

        angle = (360/self.num_of_agents)*0.0174532925 #radians 
        
        for i in range(1,self.num_of_agents):
            vectors[i][0] = vectors[i-1][0]*math.cos(angle) - vectors[i-1][1]*math.sin(angle)
            vectors[i][1] = vectors[i-1][1]*math.cos(angle) + vectors[i-1][0]*math.sin(angle)

        return vectors

    def form_via_pose(self, radius):
        coordinates = self.formation_coordinates(radius)
        for i in range(len(self.agents)):
            Thread(target=self.agents[i].move_global, args=(coordinates[i][0], coordinates[i][1], 5)).start()

    def single_potential_field(self, radius, id):
        coordinates = self.formation_coordinates(radius)
        attractive_constant = 2
        repulsive_constant = 0.002
        repulsive_force_x = 0
        repulsive_force_y = 0

        for _ in range(100):

            attractive_force_x = (coordinates[id][0] - self.agents[id].get_global_pose().pose.pose.position.x)*attractive_constant
            attractive_force_y = (coordinates[id][1] - self.agents[id].get_global_pose().pose.pose.position.y)*attractive_constant

            for i in range(len(self.agents)):
                if i == id:
                    continue
                a = self.agents[id].get_global_pose().pose.pose.position.y - self.agents[i].get_global_pose().pose.pose.position.y
                if a < 0:
                    repulsive_force_y += (1/(a**2))*repulsive_constant*-1
                else:
                    repulsive_force_y += (1/(a**2))*repulsive_constant

                b = self.agents[id].get_global_pose().pose.pose.position.x - self.agents[i].get_global_pose().pose.pose.position.x
                if b < 0:
                    repulsive_force_x += (1/(b**2))*repulsive_constant*-1
                else:
                    repulsive_force_x += (1/(b**2))*repulsive_constant

            vel_x = attractive_force_x + repulsive_force_x
            vel_y = attractive_force_y + repulsive_force_y
            self.agents[id].velocity_command(vel_x, vel_y)
            time.sleep(0.1)

        self.agents[id].move_global(coordinates[id][0], coordinates[id][1], 5)

    def form_via_potential_field(self, radius):
        for i in range(len(self.agents)):
            Thread(target=self.single_potential_field, args = (radius,i)).start()


    def add_drone(self,id):
   
        self.agents.append(Iris(id))

    def swarm_square(self):
        for i in range(len(self.agents)):
            Thread(target = self.agents[i].draw_square, args=(5,)).start()

    def distance_of_drones(self,drone_a, drone_b): # in meters

        lat_b = drone_b.gps_pose_getter().latitude
        long_b = drone_b.gps_pose_getter().longitude

        return self.distance_to_pose(drone_a, lat_b, long_b)

    def distance_to_pose(self, drone_a, lat_b, long_b): # in meters

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

    def print_drones_pose(self):
        j = 0
        for i in self.agents:
            print(j)
            i.get_global_pose()
            j += 1
        print("------------")
        time.sleep(1)

    def return_starting_pose(self):
        for i in self.agents:
            Thread(target=i.move_local, args=(0,0,5)).start()

