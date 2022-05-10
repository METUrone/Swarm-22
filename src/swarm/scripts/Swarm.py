#!/usr/bin/env python3
from tokenize import group
import numpy as np
import math
import time
from Iris import Iris
from threading import Thread
from copy import deepcopy
import copy
from munkres import Munkres
from datetime import datetime
import csv

from TurtleBot import TurtleBot
from pycrazyswarm import Crazyswarm
from utils import *



class Swarm:
    def __init__(self,num_of_drones,vehicle, first_time = True, crazyswarm_class=None):
        self.agents = []
        self.vehicle = vehicle
        self.radius = 2
        self.num_of_edges = 3

        self.log = {
    
            "speed": np.array_str(np.array([0,0,0])),
            "time" : datetime.now().strftime("%H:%M:%S:%f"),
            "position" : np.array_str(np.array([0,0,0])),
            "id" : str(0)
        }

        self.logs = {}
        simulation_origin_latitude = 47.3977419
        simulation_origin_longitude = 8.5455935

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

        
    
    def log_to_csv(self):
        for i in self.logs:

            try:

                with open("./agent{}_logs.csv".format(i),"w") as f:
                    writer_csv = csv.writer(f)
                    header = self.log.keys()
                    header_str = []

                    for j in header:
                        j = '{0:''<10}'.format(j)
                        header_str.append(j)

                    #print(header_str)
                    writer_csv.writerow(header_str)
                    for j in self.logs[i]:

                        writer_csv.writerow(j.values())
                    print("LOGGING COMPLETED !!!")
            except LookupError:
                print("FILE COULD NOT OPENED")

    def add_log(self, speed, time, position, id):
        self.log["speed"] = speed
        self.log["time"] = time
        self.log["position"] = position
        self.log["id"] = id

        try:
            self.logs[str(id)].append(copy.copy(self.log))
        except KeyError:
            self.logs[str(id)] = []
            self.logs[str(id)].append(copy.copy(self.log))


    def is_goal_reached(self, id, goal,error_radius=0.1):# 10 cm default error radius, goal is a numpy array
        
        pose = self.agents[id].position()

        distance =( (pose[0]-goal[0])**2 + (pose[1]-goal[1])**2 + (pose[2]-goal[2])**2 )**(1/2)
        if distance <= error_radius:
            return True
        else:
            #print(str(id) + "   " + str(distance))
            return False
    
    def is_formed(self, goals, error_radius=0.1):
        reached =0
        for i in range(self.num_of_agents):
             
            if self.is_goal_reached(i, goals[i], error_radius):
                reached += 1

        if reached == self.num_of_agents:
            return True
        else:
            return False

    def split_formation(self):
        swarm1 = Swarm(self.num_of_agents//2, self.vehicle, False, self.crazyswarm)
        swarm1.agents = self.crazyswarm.allcfs.crazyflies[0 : self.num_of_agents//2]
        swarm2 = Swarm(self.num_of_agents-self.num_of_agents//2, self.vehicle, False, self.crazyswarm)
        swarm2.agents = self.crazyswarm.allcfs.crazyflies[self.num_of_agents//2 : ]

        swarm1.go((-2,-2,1))
        swarm2.go((2,2,1))

        swarm1.form_polygon(3, 3, 1, np.array([-2,0,0]))
        swarm2.form_polygon(3, 3, 1, np.array([2,0,0]))

        return swarm1, swarm2

    def sort_coordinates(self, coordinates):
        sorted_coordinates = [[0, 0]]*self.num_of_agents
        cost_matrix = [[math.sqrt((target[0] - drone.position()[0])**2 + (target[1] - drone.position()[1])**2 + (target[2] - drone.position()[2])**2) for target in coordinates] for drone in self.agents]
        
        assigner = Munkres()
        assigner.fit(cost_matrix)
        assignments = assigner.assign()
        
        for assignment in assignments:
            sorted_coordinates[assignment[0]] = coordinates[assignment[1]]

        return sorted_coordinates

    def attractive_force(self, id, target_pose, attractive_constant = 2):
        speed_limit = 0.9 # must be float

        d_x = target_pose[0] - self.agents[id].position()[0]
        d_y = target_pose[1] - self.agents[id].position()[1]
        d_z = target_pose[2] - self.agents[id].position()[2]

        attractive_force_x = (target_pose[0] - self.agents[id].position()[0])*attractive_constant * math.copysign(1, d_x)
        attractive_force_y = (target_pose[1] - self.agents[id].position()[1])*attractive_constant * math.copysign(1, d_y)
        attractive_force_z = (target_pose[2] - self.agents[id].position()[2])*attractive_constant * math.copysign(1, d_z)

        return min(max(float(attractive_force_x),-1*speed_limit),speed_limit), min(max(float(attractive_force_y),-1*speed_limit),speed_limit), min(max(float(attractive_force_z),-1*speed_limit),speed_limit)

    def repulsive_force(self,id, repulsive_constant =-0.0,repulsive_threshold = 1.5, obstacles = []): # repuslsive constant must be negative

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

                d = ((y_distance**2) + (x_distance**2) + (z_distance**2))**(1/2)

                if d < repulsive_threshold and y_distance != 0 and x_distance != 0 and z_distance != 0:
                    repulsive_force_z += (1/(z_distance**2))*(1/repulsive_threshold - 1/z_distance)*repulsive_constant * math.copysign(1, z_distance)
                    if z_distance < 0.5:
                        repulsive_force_y += (1/(y_distance**2))*(1/repulsive_threshold - 1/y_distance)*repulsive_constant * math.copysign(1, y_distance)
                        repulsive_force_x += (1/(x_distance**2))*(1/repulsive_threshold - 1/x_distance)*repulsive_constant * math.copysign(1, x_distance) 
                    else:
                        repulsive_force_y = 0
                        repulsive_force_x = 0

        for obstacle in obstacles:

            z_distance = self.agents[id].position()[2] - obstacle[2]
            y_distance = self.agents[id].position()[1] - obstacle[1]
            x_distance = self.agents[id].position()[0] - obstacle[0]

            d = ((y_distance**2) + (x_distance**2) + (z_distance**2))**(1/2)

            if id == 1:
                print(str(id) + " " + str(d))

            if d < repulsive_threshold and y_distance != 0 and x_distance != 0 and z_distance != 0:
                repulsive_force_z += (1/(z_distance**2))*(1/repulsive_threshold - 1/z_distance)*repulsive_constant * math.copysign(1, z_distance)
                if z_distance < 0.5:
                    repulsive_force_y += (1/(y_distance**2))*(1/repulsive_threshold - 1/y_distance)*repulsive_constant*20 * math.copysign(1, y_distance)
                    repulsive_force_x += (1/(x_distance**2))*(1/repulsive_threshold - 1/x_distance)*repulsive_constant*20 * math.copysign(1, x_distance)
                else:
                    repulsive_force_y = 0
                    repulsive_force_x = 0

        return min(max(float(repulsive_force_x),-1*speed_limit),speed_limit), min(max(float(repulsive_force_y),-1*speed_limit),speed_limit), min(max(float(repulsive_force_z),-1*speed_limit),speed_limit)

    def attractive_force_pid(self, id, target_pose, attractive_constant = 2):

        speed_limit = 0.9 # must be float
        proportional_gain = 0.5
        integral_gain = 1
        d_x = target_pose[0] - self.agents[id].position()[0]
        d_y = target_pose[1] - self.agents[id].position()[1]

        attractive_force_x_p = ((d_x)*attractive_constant)*proportional_gain
        attractive_force_y_p = ((d_y)*attractive_constant)*proportional_gain

        attractive_force_x_i = ( (d_x)**2  *  attractive_constant/2)*integral_gain * math.copysign(1, d_x)
        attractive_force_y_i = ( (d_y)**2  *  attractive_constant/2)*integral_gain * math.copysign(1, d_y) 

        attractive_force_x = attractive_force_x_i + attractive_force_x_p
        attractive_force_y = attractive_force_y_i + attractive_force_y_p

        return min(max(float(attractive_force_x),-1*speed_limit),speed_limit), min(max(float(attractive_force_y),-1*speed_limit),speed_limit)

    def repulsive_force_pid(self,id, repulsive_constant =-0.09,repulsive_threshold = 1.5, obstacles = []): # repuslsive constant must be negative -0.2
        repulsive_force_x = 0
        repulsive_force_y = 0
        speed_limit = 0.8 # must be float

        proportional_gain = 0.5
        integral_gain = 0.5

        for i in range(len(self.agents)):
                if i == id:
                    continue
                y_distance = self.agents[id].position()[1] - self.agents[i].position()[1]
                x_distance = self.agents[id].position()[0] - self.agents[i].position()[0]

                d = ((y_distance**2) + (x_distance**2))**(1/2)

                if d < repulsive_threshold and y_distance != 0 and x_distance != 0:

                    repulsive_y_p = (1/(y_distance**2))*(1/repulsive_threshold - 1/y_distance)*repulsive_constant*proportional_gain* math.copysign(1, y_distance)
                    repulsive_x_p = (1/(x_distance**2))*(1/repulsive_threshold - 1/x_distance)*repulsive_constant*proportional_gain* math.copysign(1, x_distance)

                    repulsive_y_i = (1/2*repulsive_constant* ((1/y_distance - 1/repulsive_threshold) ** 2)) * integral_gain* math.copysign(1, y_distance)
                    repulsive_x_i = (1/2*repulsive_constant* ((1/x_distance - 1/repulsive_threshold) ** 2)) * integral_gain* math.copysign(1, x_distance)

                    repulsive_force_y += repulsive_y_i + repulsive_y_p
                    repulsive_force_x += repulsive_x_i + repulsive_x_p

        for obstacle in obstacles:

                if i == id:
                    continue
                y_distance = self.agents[id].position()[1] - obstacle[1]
                x_distance = self.agents[id].position()[0] - obstacle[0]

                d = ((y_distance**2) + (x_distance**2))**(1/2)

                if d < repulsive_threshold and y_distance != 0 and x_distance != 0:

                    repulsive_y_p = (1/(y_distance**2))*(1/repulsive_threshold - 1/y_distance)*repulsive_constant*proportional_gain* math.copysign(1, y_distance)
                    repulsive_x_p = (1/(x_distance**2))*(1/repulsive_threshold - 1/x_distance)*repulsive_constant*proportional_gain* math.copysign(1, x_distance)

                    repulsive_y_i = (1/2*repulsive_constant* ((1/y_distance - 1/repulsive_threshold) ** 2)) * integral_gain* math.copysign(1, y_distance)
                    repulsive_x_i = (1/2*repulsive_constant* ((1/x_distance - 1/repulsive_threshold) ** 2)) * integral_gain* math.copysign(1, x_distance)

                    repulsive_force_y += repulsive_y_i + repulsive_y_p
                    repulsive_force_x += repulsive_x_i + repulsive_x_p


        return min(max(float(repulsive_force_x),-1*speed_limit),speed_limit), min(max(float(repulsive_force_y),-1*speed_limit),speed_limit)


    def single_potential_field(self, id, coordinates, obstacles = []):
        #print("id: {}  pose: {}".format(id, self.agents[id].position()))

        attractive_force_x, attractive_force_y, attractive_force_z = self.attractive_force(target_pose=coordinates[id], id=id)
        

        repulsive_force_x, repulsive_force_y, repulsive_force_z = self.repulsive_force(id=id, obstacles = obstacles)

        vel_x = attractive_force_x + repulsive_force_x
        vel_y = attractive_force_y + repulsive_force_y
        vel_z = attractive_force_z + repulsive_force_z

        turtleBot_constant = 0.1
        rotational_constant = 0.3


        if self.vehicle == "Crazyflie":
            self.agents[id].cmdVelocityWorld(np.array([vel_x, vel_y, vel_z]), yawRate=0)
            self.timeHelper.sleep(0.001)
            self.add_log("{}, {}, {}, ".format(vel_x, vel_y, 0), datetime.now(),"{}, {}, {}, ".format(self.agents[id].position()[0],self.agents[id].position()[1],self.agents[id].position()[2]),id)
        elif self.vehicle == "TurtleBot": 
            
            angle_of_agent = self.agents[id].orientation() 
            vector_angle = angle_of_vector(vel_x, vel_y)
            dA = vector_angle - angle_of_agent   
            linear_vel = vel_x*vel_y* math.cos(dA)      
            #print("vel:  {}".format(np.array([linear_vel * turtleBot_constant, 0, 0])))
            self.agents[id].cmdVelocityWorld(np.array([linear_vel * turtleBot_constant, 0, 0]), yawRate=dA * rotational_constant)
            self.add_log(np.array([linear_vel * turtleBot_constant, 0, 0]), datetime.now(),self.agents[id].position(),id)
            time.sleep(0.01)
            #print("velY: {} velX: {}\ncurrA: {} targetA: {}".format(vel_y,vel_x,angle_of_agent, angle_of_vector))

        elif self.vehicle == "Iris":
            self.agents[id].cmdVelocityWorld(np.array([vel_x, vel_y, vel_z]), yawRate=0)
            time.sleep(0.0001)
            self.add_log("{}, {}, {}, ".format(vel_x, vel_y, 0), datetime.now(),"{}, {}, {}, ".format(self.agents[id].position()[0],self.agents[id].position()[1],self.agents[id].position()[2]),id)
       

    def single_potential_field_pid(self, id, coordinates, obstacles = []):
    #print("id: {}  pose: {}".format(id, self.agents[id].position()))
        attractive_force_x, attractive_force_y = self.attractive_force_pid(target_pose=coordinates[id], id=id)
        repulsive_force_x, repulsive_force_y= self.repulsive_force_pid(id=id, obstacles=obstacles)
        vel_x = attractive_force_x + repulsive_force_x
        vel_y = attractive_force_y + repulsive_force_y
        if self.vehicle == "Crazyflie":
            self.agents[id].cmdVelocityWorld(np.array([vel_x, vel_y, 0]), yawRate=0)
            self.timeHelper.sleep(0.001)
            self.add_log("{}, {}, {}, ".format(vel_x, vel_y, 0), datetime.now(),"{}, {}, {}, ".format(self.agents[id].position()[0],self.agents[id].position()[1],self.agents[id].position()[2]),id)
        else:           
            self.agents[id].velocity_command(linear_x=vel_x, linear_y=vel_y)

            time.sleep(0.001)
        if self.vehicle == "Iris":
            self.agents[id].move_global(coordinates[id][0], coordinates[id][1], 5) # makes more stable


    def form_via_potential_field(self, radius): # uses potential field algorithm to form
        print(formation_coordinates(radius, self.num_of_agents))
        self.radius = radius

        height = 1
        error_radius = 0.1

        if self.vehicle == "Iris":
            height = 5
            error_radius = 1

        coordinates = formation_coordinates(radius, self.num_of_agents, height)
        coordinates = self.sort_coordinates(coordinates)
        
        begin = time.localtime()
        while not self.is_formed(coordinates, error_radius):
            if time.localtime().tm_sec - begin.tm_sec > 30:
                print("TIMEOUT !!!")
                self.log_to_csv()
                self.land()
                return
            reached = []
            for i in range(len(self.agents)):
               #Thread(target=self.single_potential_field, args = (radius,i)).start()
                if self.is_goal_reached(i,coordinates[i], error_radius):
                    reached.append(i)
                    self.agents[i].cmdVelocityWorld(np.array([0.0, 0.0, 0.0]), 0.0)
               
                self.single_potential_field_pid(i, coordinates)                    
            self.stop_all()
            
        
                  
    def form_polygon(self, radius, num_of_edges, height = 1, displacement=[0,0,0]): # uses potential field algorithm to form
        

        coordinates = self.sort_coordinates(formation_coordinates(radius, num_of_edges, height=height, displacement=displacement))

        print(coordinates)

        if self.vehicle == "Crazyflie":
            while not self.is_formed(coordinates):
            #for i in range(4000):
                for i in range(len(self.agents)):
                    
                    self.single_potential_field( i, coordinates)
                    
            self.stop_all()
            self.timeHelper.sleep(4)           

    def form_coordinates(self, coordinates, obstacles = []): # uses potential field algorithm to form
        print(coordinates)
        coordinates = self.sort_coordinates(coordinates)


        while not self.is_formed(coordinates):
            for i in range(len(self.agents)):
                
                self.single_potential_field(i, coordinates, obstacles)
                
        self.stop_all()

        try:
            self.timeHelper.sleep(4)    
        except:
            time.sleep(4)    

    def form_coordinates_pid(self, coordinates, obstacles = []): # uses potential field algorithm to form
        print(coordinates)
        coordinates = self.sort_coordinates(coordinates)


        while not self.is_formed(coordinates):
            for i in range(len(self.agents)):
                
                self.single_potential_field_pid(i, coordinates, obstacles)
                
        self.stop_all()

        try:
            self.timeHelper.sleep(4)    
        except:
            time.sleep(4) 

    
    def form_via_pose(self, radius): # uses position commands to form but collisions may occur
        coordinates = formation_coordinates(radius, self.num_of_agents)
        for i in range(len(self.agents)):
            Thread(target=self.agents[i].move_global, args=(coordinates[i][0], coordinates[i][1], 5)).start()

    def form_3d_split(self, radius, num_edges):
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

    def form_3d(self, radius, num_edges):
        if num_edges == "prism":
            coordinates = self.sort_coordinates(np.concatenate((formation_coordinates(0, 1, height=1.5), formation_coordinates(radius, 4, height=0.5))))
            self.form_coordinates(coordinates=coordinates)
        elif num_edges == "cylinder": # Here circle function can be used.
            pass
        else:
            coordinates = self.sort_coordinates(np.concatenate((formation_coordinates(radius=radius, num_of_edges=num_edges, height=1.5), formation_coordinates(radius, num_of_edges=num_edges, height=0.5))))
            self.form_coordinates(coordinates=coordinates)


    def add_drone(self,id):
   
        self.agents.append(Iris(id))

    def add_turtlebot(self,id):
   
        self.agents.append(TurtleBot(id))

    def swarm_square(self, length):
        if self.vehicle == "Crazyflie":

            self.go(np.array([length,0,0]))
            self.go(np.array([0,length,0]))
            self.go(np.array([-length,0,0]))
            self.go(np.array([0,-length,0]))
        else:

            for i in range(len(self.agents)):
                Thread(target = self.agents[i].draw_square, args=(5,)).start()


    def distance_of_drones(self,drone_a, drone_b): # in meters

        lat_b = drone_b.gps_pose_getter().latitude
        long_b = drone_b.gps_pose_getter().longitude

        return self.distance_to_pose(drone_a, lat_b, long_b)


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

    def go(self, vector, obstacles=[]):
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

        self.form_coordinates(coordinates, obstacles)


        time.sleep(2)

    def go_pid(self, vector, obstacles=[]):
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

        self.form_coordinates_pid(coordinates, obstacles)


        time.sleep(2)

    def star_formation(self, radius = 4, angle=0, height = 1):
        pentagon1 = formation_coordinates(radius,5,height,angle)
        pentagon2 = formation_coordinates(radius/2,5,1,36+angle)
        pentagons = []

        for i in pentagon1:
            pentagons.append(i)
        for i in pentagon2:
            pentagons.append(i)

        self.form_coordinates(pentagons)
        self.timeHelper.sleep(2)