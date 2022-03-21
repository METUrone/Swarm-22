#!/usr/bin/env python3
import numpy as np
import math
import time
from Iris import Iris
from threading import Thread
from copy import deepcopy
from munkres import Munkres

from TurtleBot import TurtleBot
from pycrazyswarm import Crazyswarm


class Swarm:
    def __init__(self,num_of_drones,vehicle, first_time = True, crazyswarm_class=None):
        self.agents = []
        self.vehicle = vehicle

        if vehicle == "Crazyflie":
            if first_time:
                self.crazyswarm = Crazyswarm()
                self.agents = self.crazyswarm.allcfs.crazyflies[0:num_of_drones]
                self.timeHelper = self.crazyswarm.timeHelper

                for i in range(len(self.agents)):
                    self.takeoff(i)
                    print(self.agents[i].position()[0])
            else:
                self.timeHelper = crazyswarm_class.timeHelper
            
            self.timeHelper.sleep(1)
        
        elif vehicle == "Iris":
            
            for i in range(num_of_drones):
                self.add_drone(i)

            for i in range(len(self.agents)):
                pose = self.distance_to_pose(drone_a=self.agents[i], lat_b=simulation_origin_latitude, long_b=simulation_origin_longitude)
                print(self.agents[i].id)
                self.agents[i].set_starting_pose(pose[0], pose[1]) #Agent must be in the starting position (locally 0, 0) height does not matter
                self.agents[i].position()

        elif vehicle == "TurtleBot":

            for i in range(num_of_drones):
                self.add_turtlebot(i)

        self.num_of_agents = num_of_drones

        simulation_origin_latitude = 47.3977419
        simulation_origin_longitude = 8.5455935

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

        if reached == self.num_of_agents:
            return True
        else:
            return False


    def formation_coordinates(self,radius, num_of_edges, height = 1, displacement=np.array([0,0,0]) ,rotation_angle=0):
        vectors = np.zeros(shape=(num_of_edges,3)) # [id][0: for x, 1: for y, 2: for z]

        vectors[0][0]=radius
        vectors[0][1]=0
        vectors[0][2]=height

        angle = (360/num_of_edges)*0.0174532925 #radians 

        for i in range(1,num_of_edges):
            vectors[i][0] = radius*math.cos(i*angle + rotation_angle) + displacement[0]
            vectors[i][1] = radius*math.sin(i*angle + rotation_angle) + displacement[1]
            vectors[i][2] = height + displacement[2]

        return vectors

    def sort_coordinates(self, coordinates):
        sorted_coordinates = [[0, 0]]*self.num_of_agents
        cost_matrix = [[math.sqrt((target[0] - drone.position()[0])**2 + (target[1] - drone.position()[1])**2) for target in coordinates] for drone in self.agents]
        
        assigner = Munkres()
        assigner.fit(cost_matrix)
        assignments = assigner.assign()
        
        for assignment in assignments:
            sorted_coordinates[assignment[0]] = coordinates[assignment[1]]

        return sorted_coordinates

    def attractive_force(self, id, target_pose, attractive_constant = 2):
        speed_limit = 0.9 # must be float

        attractive_force_x = (target_pose[0] - self.agents[id].position()[0])*attractive_constant
        attractive_force_y = (target_pose[1] - self.agents[id].position()[1])*attractive_constant
        attractive_force_z = (target_pose[2] - self.agents[id].position()[2])*attractive_constant

        return min(max(float(attractive_force_x),-1*speed_limit),speed_limit), min(max(float(attractive_force_y),-1*speed_limit),speed_limit), min(max(float(attractive_force_z),-1*speed_limit),speed_limit)

    def repulsive_force(self,id, repulsive_constant =-0.09,repulsive_threshold = 1.5): # repuslsive constant must be negative
        repulsive_force_x = 0
        repulsive_force_y = 0
        repulsive_force_z = 0
        speed_limit = 0.8 # must be float

        for i in range(len(self.agents)):
                if i == id:
                    continue
                z_distance = self.agents[id].position()[2] - self.agents[i].position()[2]
                y_distance = self.agents[id].position()[1] - self.agents[i].position()[1]
                x_distance = self.agents[id].position()[0] - self.agents[i].position()[0]

                d = ((y_distance**2) + (x_distance**2))**(1/2)

                if d < repulsive_threshold and y_distance != 0 and x_distance != 0:
                    repulsive_force_z += (1/(z_distance**2))*(1/repulsive_threshold - 1/z_distance)*repulsive_constant
                    repulsive_force_y += (1/(y_distance**2))*(1/repulsive_threshold - 1/y_distance)*repulsive_constant
                    repulsive_force_x += (1/(x_distance**2))*(1/repulsive_threshold - 1/x_distance)*repulsive_constant

        return min(max(float(repulsive_force_x),-1*speed_limit),speed_limit), min(max(float(repulsive_force_y),-1*speed_limit),speed_limit), min(max(float(repulsive_force_z),-1*speed_limit),speed_limit)


    def single_potential_field(self, id, coordinates):

        #for _ in range(4000):
        #print("id: {}  pose: {}".format(id, self.agents[id].position()))

        attractive_force_x, attractive_force_y, attractive_force_z = self.attractive_force(target_pose=coordinates[id], id=id)
        repulsive_force_x, repulsive_force_y, repulsive_force_z = self.repulsive_force(id=id)

        vel_x = attractive_force_x + repulsive_force_x
        vel_y = attractive_force_y + repulsive_force_y
        vel_z = attractive_force_z + repulsive_force_z

        if self.vehicle == "Crazyflie":
            self.agents[id].cmdVelocityWorld(np.array([vel_x, vel_y, vel_z]), yawRate=0)
            self.timeHelper.sleep(0.001)
        else:           
            self.agents[id].velocity_command(linear_x=vel_x, linear_y=vel_y)
               
            time.sleep(0.001)

        if self.vehicle == "Iris":
            self.agents[id].move_global(coordinates[id][0], coordinates[id][1], 5) # makes more stable

    def form_via_potential_field(self, radius): # uses potential field algorithm to form
        print(self.formation_coordinates(radius, self.num_of_agents))
        self.radius = radius

        coordinates = self.formation_coordinates(radius, self.num_of_agents)
        coordinates = self.sort_coordinates(coordinates)
        if self.vehicle == "Crazyflie":
            while not self.is_formed(coordinates):
            #for i in range(4000):
                for i in range(len(self.agents)):
                    #Thread(target=self.single_potential_field, args = (radius,i)).start()
                    self.single_potential_field(i, coordinates)                    
                
            
        elif self.vehicle == "Iris":
            for i in range(len(self.agents)):
                    #Thread(target=self.single_potential_field, args = (radius,i)).start()
                    self.single_potential_field(i, coordinates)
                    print("call")

            self.stop_all()
            self.timeHelper.sleep(4)  
                  
    def form_polygon(self, radius, num_of_edges, height = 1, displacement=[0,0,0]): # uses potential field algorithm to form
        

        coordinates = self.sort_coordinates(self.formation_coordinates(radius, num_of_edges, height=height, displacement=displacement))

        print(coordinates)

        if self.vehicle == "Crazyflie":
            while not self.is_formed(coordinates):
            #for i in range(4000):
                for i in range(len(self.agents)):
                    
                    self.single_potential_field( i, coordinates)
                    
            self.stop_all()
            self.timeHelper.sleep(4)           

    def form_coordinates(self, coordinates): # uses potential field algorithm to form
        print(coordinates)
        coordinates = self.sort_coordinates(coordinates)

        if self.vehicle == "Crazyflie":
            while not self.is_formed(coordinates):
            #for i in range(4000):
                for i in range(len(self.agents)):
                    
                    self.single_potential_field(i, coordinates)
                    
            self.stop_all()
            self.timeHelper.sleep(4)         

    
    def form_via_pose(self, radius): # uses position commands to form but collisions may occur
        coordinates = self.formation_coordinates(radius, self.num_of_agents)
        for i in range(len(self.agents)):
            Thread(target=self.agents[i].move_global, args=(coordinates[i][0], coordinates[i][1], 5)).start()

    def form_3d(self, radius, num_edges):
        if num_edges == "prism":
            swarm1 = Swarm(1, self.vehicle, False, self.crazyswarm)
            swarm1.agents = self.crazyswarm.allcfs.crazyflies[0 : 1]
            swarm1.stop_all()
            swarm2 = Swarm(4, self.vehicle, False, self.crazyswarm)
            swarm2.agents = self.crazyswarm.allcfs.crazyflies[1 : 5]
            swarm2.stop_all()
            swarm1.form_polygon(0, 1, 1.5)
            swarm2.form_polygon(2, 4, 0.5)   

        elif num_edges == "cylinder": # Here circle function can be used.
            pass
        else:
            swarm1 = Swarm(num_edges, self.vehicle, False, self.crazyswarm)
            swarm1.agents = self.crazyswarm.allcfs.crazyflies[0 : num_edges]
            swarm1.stop_all()
            swarm2 = Swarm(num_edges, self.vehicle, False, self.crazyswarm)
            swarm2.agents = self.crazyswarm.allcfs.crazyflies[num_edges : 2*num_edges]
            swarm2.stop_all()
            #swarm1.go((0, 0, 0.5))
            #swarm2.go((0, 0, -0.5))
            swarm1.form_polygon(2, num_edges, 1.5)
            swarm2.form_polygon(2, num_edges, 0.5)

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

    def takeoff(self, id, height=1):
        if self.vehicle == "TurtleBot":
            print("TURTLES CAN NOT FLY !!!")
            return
        elif self.vehicle == "Crazyflie":
            self.agents[id].takeoff(targetHeight=height, duration=1)
        

    def land(self):
        if self.vehicle == "Crazyflie":
            for i in range(len(self.agents)):
                self.agents[i].land(0.03, 2)

    def stop_all(self):
        if self.vehicle == "Iris" or self.vehicle == "TurtleBot":
            for i in range(len(self.agents)):
                self.agents[i].velocity_command()
        if self.vehicle == "Crazyflie":
            for i in range(len(self.agents)):
                self.agents[i].cmdVelocityWorld(np.array([0.0, 0.0, 0.0]), yawRate=0)

    def add_agent_to_formation(self):
        self.agents.append(self.crazyswarm.allcfs.crazyflies[len(self.agents)])
        self.num_of_agents += 1

        self.takeoff(len(self.agents)-1)
        self.timeHelper.sleep(1)
        self.form_polygon(self.radius, len(self.agents))
        self.timeHelper.sleep(2)

    def omit_agent(self):
        self.agents[len(self.agents)-1].land(0.03, 2)
        self.num_of_agents -= 1
        self.agents.pop()
        self.timeHelper.sleep(1)
        self.form_via_potential_field(self.radius)
        self.timeHelper.sleep(2)

    def go(self, vector):
        coordinates = np.zeros(shape=(len(self.agents),3))

        for i in range(len(self.agents)):
            coordinates[i] = self.agents[i].position()

        for i in range(len(self.agents)):
            try:
                coordinates[i][0] += vector[0]
                coordinates[i][1] += vector[1]
                coordinates[i][2] += vector[2]
            except IndexError:
                print(coordinates[i][2])
                print(vector[i][2])

        self.form_coordinates(coordinates)
        self.timeHelper.sleep(2)

    def circle(self, id, totalTime, radius, kPosition): ## DOES NOT WORK CURRENTLY
        startTime = self.timeHelper.time()
        pos = self.agents[id].position()
        startPos = self.agents[id].initialPosition + np.array([0, 0, 1])
        center_circle = startPos - np.array([radius, 0, 0])
        while True:
            time = self.timeHelper.time() - startTime
            omega = 2 * np.pi / totalTime
            vx = -radius * omega * np.sin(omega * time)  
            vy = radius * omega * np.cos(omega * time)
            desiredPos = center_circle + radius * np.array(
                [np.cos(omega * time), np.sin(omega * time), 0])
            errorX = desiredPos - self.agents[id].position() 
            self.agents[id].cmdVelocityWorld(np.array([vx, vy, 0] + kPosition * errorX), yawRate=0)
            self.timeHelper.sleepForRate(30)
            if time >= totalTime:
                return

    def star_formation(self, radius = 4, angle=0, height = 1):
        pentagon1 = self.formation_coordinates(radius,5,height,angle)
        pentagon2 = self.formation_coordinates(radius/2,5,1,36+angle)
        pentagons = []

        for i in pentagon1:
            pentagons.append(i)
        for i in pentagon2:
            pentagons.append(i)

        self.form_coordinates(pentagons)
        self.timeHelper.sleep(2)
