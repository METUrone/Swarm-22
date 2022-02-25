#!/usr/bin/env python3
import numpy as np
import math
import time
from Iris import Iris
from threading import Thread
from copy import deepcopy

from TurtleBot import TurtleBot
from pycrazyswarm import Crazyswarm


class Swarm:
    def __init__(self,num_of_drones,vehicle):
        self.agents = []
        self.vehicle = vehicle

        if vehicle == "Iris":
            for i in range(num_of_drones):
                self.add_drone(i)
        elif vehicle == "TurtleBot":
            for i in range(num_of_drones):
                self.add_turtlebot(i)
        elif vehicle == "Crazyflie":
            self.crazyswarm = Crazyswarm()
            self.agents = self.crazyswarm.allcfs.crazyflies
            self.timeHelper = self.crazyswarm.timeHelper

            for i in range(len(self.agents)):
                self.take_off(i)
                print(self.agents[i].position()[0])


            
            #self.agents[0].cmdVelocityWorld(np.array([1.0, 0.0, 0.0]), yawRate=0)
            #self.timeHelper.sleep(2)

        self.num_of_agents = len(self.agents)
        #self.crazyswarm

        simulation_origin_latitude = 47.3977419
        simulation_origin_longitude = 8.5455935

        if self.vehicle == "Iris":
            for i in range(len(self.agents)):
                pose = self.distance_to_pose(drone_a=self.agents[i], lat_b=simulation_origin_latitude, long_b=simulation_origin_longitude)
                print(self.agents[i].id)
                self.agents[i].set_starting_pose(pose[0], pose[1]) #Agent must be in the starting position (locally 0, 0) height does not matter
                self.agents[i].position()

    def is_goal_reached(self, id, goal,error_radius=0.1):# 10 cm default error radius, goal is a numpy array
        
        pose = self.agents[id].position()

        distance =( (pose[0]-goal[0])**2 + (pose[1]-goal[1])**2 + (pose[2]-goal[2])**2 )**(1/2)
        if distance <= error_radius:
            return True
        else:
            return False
    
    def is_formed(self, goals):
        reached =0
        for i in range(self.num_of_agents):
             
            if self.is_goal_reached(i, goals[i]):
                reached += 1
            else:
                print("{} distance".format(i))

        if reached == self.num_of_agents:
            return True
        else:
            print(reached)
            return False


    def formation_coordinates(self, radius, height = 1):
        vectors = np.zeros(shape=(self.num_of_agents,3)) # [id][0: for x, 1: for y, 2: for z]

        vectors[0][0]=0
        vectors[0][1]=radius

        angle = (360/self.num_of_agents)*0.0174532925 #radians 
        
        for i in range(1,self.num_of_agents):
            vectors[i][0] = vectors[i-1][0]*math.cos(angle) - vectors[i-1][1]*math.sin(angle)
            vectors[i][1] = vectors[i-1][1]*math.cos(angle) + vectors[i-1][0]*math.sin(angle)
            vectors[i][2] = height

        return vectors

    def attractive_force(self, id, target_pose, attractive_constant = 5):
        speed_limit = 2.1 # must be float

        attractive_force_x = (target_pose[0] - self.agents[id].position()[0])*attractive_constant
        attractive_force_y = (target_pose[1] - self.agents[id].position()[1])*attractive_constant

        return min(max(float(attractive_force_x),-1*speed_limit),speed_limit), min(max(float(attractive_force_y),-1*speed_limit),speed_limit)

    def repulsive_force(self,id, repulsive_constant =-0.03,repulsive_threshold = 3): # repuslsive constant must be negative
        repulsive_force_x = 0
        repulsive_force_y = 0
        speed_limit = 1.1 # must be float

        for i in range(len(self.agents)):
                if i == id:
                    continue
                y_distance = self.agents[id].position()[1] - self.agents[i].position()[1]
                x_distance = self.agents[id].position()[0] - self.agents[i].position()[0]

                d = ((y_distance**2) + (x_distance**2))**(1/2)

                if d < repulsive_threshold and y_distance != 0 and x_distance != 0:
                    repulsive_force_y += (1/(y_distance**2))*(1/repulsive_threshold - 1/y_distance)*repulsive_constant
                    repulsive_force_x += (1/(x_distance**2))*(1/repulsive_threshold - 1/x_distance)*repulsive_constant

        return min(max(float(repulsive_force_x),-1*speed_limit),speed_limit), min(max(float(repulsive_force_y),-1*speed_limit),speed_limit)


    def single_potential_field(self, radius, id):
        coordinates = self.formation_coordinates(radius)

        #for _ in range(4000):
        #print("id: {}  pose: {}".format(id, self.agents[id].position()))

        attractive_force_x, attractive_force_y = self.attractive_force(target_pose=coordinates[id], id=id)
        repulsive_force_x, repulsive_force_y = self.repulsive_force(id=id)

        vel_x = attractive_force_x + repulsive_force_x
        vel_y = attractive_force_y + repulsive_force_y

        if self.vehicle == "Crazyflie":
            self.agents[id].cmdVelocityWorld(np.array([vel_x, vel_y, 0]), yawRate=0)
            self.timeHelper.sleep(0.001)
        else:           
            self.agents[id].velocity_command(linear_x=vel_x, linear_y=vel_y)
               
            time.sleep(0.001)

        if self.vehicle == "Iris":
            self.agents[id].move_global(coordinates[id][0], coordinates[id][1], 5) # makes more stable

    def form_via_potential_field(self, radius): # uses potential field algorithm to form
        print(self.formation_coordinates(radius))

        coordinates = self.formation_coordinates(radius)
        if self.vehicle == "Crazyflie":
            while not self.is_formed(coordinates):
            #for i in range(4000):
                for i in range(len(self.agents)):
                    #Thread(target=self.single_potential_field, args = (radius,i)).start()
                    self.single_potential_field(radius, i)
                    print(self.is_formed(coordinates))
                    
                
            
        elif self.vehicle == "Iris":
            for i in range(len(self.agents)):
                    #Thread(target=self.single_potential_field, args = (radius,i)).start()
                    self.single_potential_field(radius, i)
                    print("call")

    
    def form_via_pose(self, radius): # uses position commands to form but collisions may occur
        coordinates = self.formation_coordinates(radius)
        for i in range(len(self.agents)):
            Thread(target=self.agents[i].move_global, args=(coordinates[i][0], coordinates[i][1], 5)).start()



    def add_drone(self,id):
   
        self.agents.append(Iris(id))

    def add_turtlebot(self,id):
   
        self.agents.append(TurtleBot(id))

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
            i.position()
            j += 1
        print("------------")
        time.sleep(1)

    def return_starting_pose(self):
        for i in self.agents:
            Thread(target=i.move_local, args=(0,0,5)).start()

    def take_off(self, id, height=1):
        if self.vehicle == "TurtleBot":
            print("TURTLES CAN NOT FLY !!!")
            return
        elif self.vehicle == "Crazyflie":
            self.agents[id].takeoff(targetHeight=height, duration=3)
            self.timeHelper.sleep(3)

