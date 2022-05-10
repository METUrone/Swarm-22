#!/usr/bin/env python3

from Swarm import Swarm 
from TurtleBot import TurtleBot
import time
import math
from threading import Thread
import numpy as np

from utils import *

def goto(goal, turtle):
    pose = turtle.position()

    angle = angle_of_vector(goal[0]-pose[0],goal[1]-pose[1])
    print(angle*radian_to_degree)
    turtle.turn_to(angle)

    d = ((goal[0]-pose[0])**2 + (goal[1]-pose[1])**2)**0.5
    d_error = 0.2
    max_vel = 0.5
    vel = d

    vel = max(min(vel,max_vel), -max_vel)
    linear_gain = 0.3
    print(vel)
    turtle.cmdVelocityWorld(np.array([0, 0, 0]), yawRate=0)
    while d > d_error:
        pose = turtle.position()
        angle = angle_of_vector(goal[0]-pose[0],goal[1]-pose[1])
        angular_vel = turtle.turn_to(angle, True) if turtle.turn_to(angle, True) else 0

        d = ((goal[0]-pose[0])**2 + (goal[1]-pose[1])**2)**0.5
        vel = max(min(vel,max_vel), -max_vel)
        turtle.cmdVelocityWorld(np.array([vel* linear_gain , 0, 0]), yawRate=float(angular_vel))
        """ print(d) """
    turtle.cmdVelocityWorld(np.array([0, 0, 0]), yawRate=0)

if __name__ == "__main__":

    agent_count = 3
    radius = 1.5
    vehicle1 = "Iris"
    vehicle2 = "Crazyflie"
    vehicle3 = "TurtleBot"
    radian_to_degree = 57.2957795
    """ swarm = Swarm(agent_count, vehicle3)
    swarm.form_via_potential_field(3) """
    turtle0 = TurtleBot(0)
    turtle1 = TurtleBot(1)
    turtle2 = TurtleBot(2)
    agents = [turtle0, turtle1, turtle2]

    center = center_of_agents(agents)
    goal0 = [1,1]
    goal1 = [2,2]
    goal2 = [0,2]
    coordinates = formation_coordinates(radius, 3, height = 1, displacement=np.array([0,0,0]) ,rotation_angle=0)
    
    Thread(target = goto, args=(coordinates[0], turtle0)).start()
    Thread(target = goto, args=(coordinates[1], turtle1)).start()
    Thread(target = goto, args=(coordinates[2], turtle2)).start()
    """ goto(goal0, turtle0)
    goto(goal1, turtle1)
    goto(goal2, turtle2) """

    """ pose = turtle.position()

    angle = angle_of_vector(goal[0]-pose[0],goal[1]-pose[1])
    print(angle*radian_to_degree)
    turtle.turn_to(angle)

    d = ((goal[0]-pose[0])**2 + (goal[1]-pose[1])**2)**0.5
    d_error = 0.3
    max_vel = 0.5
    vel = d

    vel = max(min(vel,max_vel), -max_vel)
    print(vel)
    turtle.cmdVelocityWorld(np.array([0, 0, 0]), yawRate=0)
    while d > d_error:
        pose = turtle.position()
        d = ((goal[0]-pose[0])**2 + (goal[1]-pose[1])**2)**0.5
        vel = max(min(vel,max_vel), -max_vel)
        turtle.cmdVelocityWorld(np.array([vel*0.3, 0, 0]), yawRate=0)
        print(d)
    turtle.cmdVelocityWorld(np.array([0, 0, 0]), yawRate=0) """

    """ turtle.cmdVelocityWorld(np.array([0.0, 0, 0]), 0.1)
    while True:
        time.sleep(1)
        print(turtle.orientation()*radian_to_degree)
        #print(turtle.position()) """
    """ swarm.form_via_potential_field(4)
    swarm.log_to_csv() """