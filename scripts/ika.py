#!/usr/bin/env python3
from matplotlib.pyplot import grid
import rospy
import numpy as np
import math
from utils import *
from munkres import Munkres

from geometry_msgs.msg import Point
from swarm.msg import general_parameters

class Ika:
    lim = 3.5
    grid_w = 20
    def __init__(self,name:str):
        self.pose = np.array([0.0,0.0,0.0])
        rospy.init_node("exting")
        rospy.Subscriber("/general_parameters/ika_" + name, general_parameters, self.pose_listener_callback)
        self.pose_pub = rospy.Publisher("/goal_pose/ika_" + name, Point, queue_size=10)
        print("Connected to ika with id " + name)

    def pose_listener_callback(self,data):
        """general_parameters
        string_1 = "\nTeam Name: " + str(data.team_name) +"\nUav Name: "+  str(data.uav_name) 
        string_2 ="\nRoll: " + str(data.rpy.roll) + "\nPitch: " + str(data.rpy.pitch) + "\nYaw: "+str(data.rpy.roll)
        string_3 ="\nx: " + str(data.pose.x) + "\ny: " + str(data.pose.y) + "\nz: " + str(data.pose.z)  
        string_data = string_1 + string_2 + string_3
        rospy.loginfo(rospy.get_caller_id() + " I heard %s", string_data)
        """
        self.pose = np.array([data.pose.x,data.pose.y,data.pose.z])

    def goTo(self,pos):
        msg = Point(pos[0],pos[1],pos[2])
        self.pose_pub.publish(msg)

    def goToIdx(self,idx):
        i = idx*self.lim/self.grid_w - self.lim/2
        msg = Point(i[0],i[1],0)
        self.pose_pub.publish(msg)

    def position(self):
        return self.pose
    
    def idx(self):
        pos = self.pose[:2]
        return np.floor(np.clip((pos+(self.lim/2))*self.grid_w/self.lim,0,19)).astype("int")
    
class IkaSwarm:
    def __init__(self,ika_names:list) -> None:
        self.agents = []
        for name in ika_names:
            self.agents.append(Ika(name))
        self.ika_count = len(self.agents)
    
    def formation_coordinates(self,distance_between, num_of_edges, height = 1, displacement=np.array([0,0,0]) ,rotation_angle=0):
        vectors = np.zeros(shape=(num_of_edges,3)) # [id][0: for x, 1: for y, 2: for z]
        radius = distance_to_radius(distance_between, num_of_edges)
        vectors[0][0]=radius
        vectors[0][1]=0
        vectors[0][2]=height

        angle = degree_to_radian(360/num_of_edges)
        agent_angle = 0

        for i in range(num_of_edges):
            agent_angle = i*angle + degree_to_radian(rotation_angle)
            vectors[i][0] = math.floor((radius*math.cos(agent_angle) + displacement[0]) * 1000)/ 1000
            vectors[i][1] = math.floor((radius*math.sin(agent_angle) + displacement[1]) * 1000)/ 1000
            vectors[i][2] = math.floor((height + displacement[2]) * 1000)/ 1000
        return vectors

    def sort_coordinates(self, coordinates):
        sorted_coordinates = [[0, 0]]*self.ika_count
        cost_matrix = [[math.sqrt((target[0] - drone.position()[0])**2 + (target[1] - drone.position()[1])**2) for target in coordinates] for drone in self.agents]
        
        assigner = Munkres()
        assignments = assigner.compute(cost_matrix)
        
        for assignment in assignments:
            sorted_coordinates[assignment[0]] = coordinates[assignment[1]]

        return sorted_coordinates

    def form(self,radius,offset = np.array([0.0,0.0,0.0])):
        coordinates = self.formation_coordinates(radius, self.ika_count,height= 0, displacement= offset)
        coordinates = self.sort_coordinates(coordinates)
        print(coordinates)
        for i,ika in enumerate(self.agents):
            ika.goTo(coordinates[i])

if __name__ == "__main__":
    from Swarm import Swarm
    swarm = Swarm(3,"Crazyflie")
    swarm.fire_extinguish(pixels=500)

