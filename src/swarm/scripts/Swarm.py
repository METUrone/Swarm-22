#!/usr/bin/env python3

import math
import time
from Iris import Iris
from threading import Thread
from copy import deepcopy
import numpy as np

class Swarm:
    def __init__(self,num_of_drones,vehicle):
        self.agents = []
        self.vehicle = vehicle
        self.F = [0]*num_of_drones
        self.available_targets = [True] * num_of_drones
        
        if vehicle == "Iris":
            for i in range(num_of_drones):
                self.add_drone(i)

        self.num_of_agents = len(self.agents)

        simulation_origin_latitude = 47.3977419
        simulation_origin_longitude = 8.5455935

        for i in range(len(self.agents)):
            pose = self.distance_of_drones(drone_a=self.agents[i], lat_b=simulation_origin_latitude, long_b=simulation_origin_longitude)
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

    def sort_coordinates(self, coordinates):
        sorted_coordinates = [[]] * self.num_of_agents
        for i in range(self.num_of_agents):
            nearest_length = 0xFFFFFFFFFF
            nearest = None
            for j in range(self.num_of_agents):
                if(self.available_targets[j]):
                    length = math.sqrt((coordinates[j][0] - self.agents[self.num_of_agents-1-i].global_pose.pose.pose.position.x)*(coordinates[j][0] - self.agents[self.num_of_agents-1-i].global_pose.pose.pose.position.x) + (coordinates[j][1] - self.agents[self.num_of_agents-1-i].global_pose.pose.pose.position.y)*(coordinates[j][1] - self.agents[self.num_of_agents-1-i].global_pose.pose.pose.position.y))
                    if length < nearest_length:
                        nearest_length = length
                        nearest = j
            sorted_coordinates[self.num_of_agents-1-i] = coordinates[nearest]
            self.available_targets[nearest] = False
        return sorted_coordinates

    def form_via_pose(self, radius):
        coordinates = self.formation_coordinates(radius)
        coordinates = self.sort_coordinates(coordinates)
        while True:
            self.potential_field(2, 100000000, coordinates, 5, 1)
            if self.steady_state():
                for i in range(self.num_of_agents):
                    self.agents[i].velocity_command(0,0,0,0,0,0)
                    self.agents[i].move_local(self.agents[i].odomery_pose.pose.pose.position.x, self.agents[i].odomery_pose.pose.pose.position.y, 5)
                return
            for i in range(self.num_of_agents):
                force = self.F[i]
                self.agents[i].velocity_command(force[0], force[1], 0, 0, 0, 0)

    

    def add_drone(self,id):
   
        self.agents.append(Iris(id))

    def swarm_square(self):
        for i in range(len(self.agents)):
            Thread(target = self.agents[i].draw_square, args=(5,)).start()

    def distance_of_drones(self,drone_a, drone_b): # in meters

        lat_b = drone_b.gps_pose_getter().latitude
        long_b = drone_b.gps_pose_getter().longitude

        return self.distance_of_drones(drone_a, lat_b, long_b)

    def distance_of_drones(self, drone_a, lat_b, long_b): # in meters

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

    def potential_field(self, repulsive_treshold=10, repulsive_effect=10, target=False, attractive_treshold = 1, attractive_effect=5):        
        for i in range(self.num_of_agents):
            force = np.array([0.0, 0.0])
            for j in range(self.num_of_agents):
                distance_tuple = self.distance_of_drones(self.agents[i], self.agents[j].gps_pose_getter().latitude, self.agents[j].gps_pose_getter().longitude)
                distance = math.sqrt(distance_tuple[0]*distance_tuple[0] + distance_tuple[1]*distance_tuple[1])
                if distance <= repulsive_treshold and distance != 0:
                    direction = [distance_tuple[0], distance_tuple[1]]
                    coefficient = -repulsive_effect*((1/repulsive_treshold)-(1/distance))*(1/(distance*distance))
                    force += np.dot(direction, coefficient)
                else:
                    continue

            if target != False:
                direction = [target[i][0] - self.agents[i].global_pose.pose.pose.position.x, target[i][1] - self.agents[i].global_pose.pose.pose.position.y]
                distance = self.distance_of_drones(self.agents[i], target[i][0], target[i][1])[2]
                if distance >= attractive_treshold:
                    force += np.dot(direction, attractive_effect)
                else:
                    force += np.dot(direction, (attractive_effect*attractive_treshold/distance))

            for k in range(2):
                if (force[k] < 0.1 and force[k] > -0.1):
                    force[k] = 0
                        

            force = force.tolist()            
            self.F[i] = force

    def steady_state(self):
        steady = True
        for i in range(self.num_of_agents):
            if self.F[i][0] != 0 or self.F[i][1] != 0:
                steady = False
        return steady

    def move_with_swarm(self, repulsive_treshold = 10, repulsive_effect = 10, target = False, attractive_effect = 5):
        while True:
            self.potential_field(repulsive_treshold, repulsive_effect, [target], repulsive_treshold, attractive_effect)
            if self.steady_state():
                for i in range(self.num_of_agents):
                    self.agents[i].velocity_command(0,0,0,0,0,0)
                    self.agents[i].move_local(self.agents[i].odomery_pose.pose.pose.position.x, self.agents[i].odomery_pose.pose.pose.position.y, 5)
                return
            for i in range(self.num_of_agents):
                force = self.F[i]
                self.agents[i].velocity_command(force[0], force[1], 0, 0, 0, 0)


    def keep_away(self):
        while True:
            self.potential_field(2, 5)
            if self.steady_state():
              break
            for i in range(self.num_of_agents):
                force = self.F[i]
                Thread(target=self.agents[i].velocity_command, args=(force[0], force[1], 0, 0, 0, 0)).start()