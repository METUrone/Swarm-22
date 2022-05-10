import math
import numpy as np
import os

def angle_of_vector(x, y):
    angle = math.atan(abs(y)/abs(x))

    if x >= 0 and y >= 0:
        return angle
    elif x < 0 and y >= 0:
        return math.pi - angle
    elif x < 0 and y < 0:
        return math.pi + angle
    else:
        return 2*math.pi - angle

def formation_coordinates(radius, num_of_edges, height = 1, displacement=np.array([0,0,0]) ,rotation_angle=0):
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

def center_of_agents(agents):
    center = np.array([0, 0, 0])
    num_of_agents = len(agents)
    for i in agents:
        for j in range(3):
            center[j] += i.position()[j]
    return center[0]/num_of_agents, center[1]/num_of_agents, center[2]/num_of_agents


def clear_log():
    for i in range(50):
        try:
            os.remove('./agent{}_logs.csv'.format(i))
        except FileNotFoundError:
            return
    
def delta_angle(x,y):
    angle = math.atan(y/x)
    if x < 0 and y >= 0:
        angle =  angle + math.pi
    elif x < 0 and y < 0:
        angle =  angle + math.pi
    else:
        angle =  angle + math.pi
    return angle if math.pi <=angle else -angle

def formation_coordinates(radius, num_of_edges, height = 2, displacement=np.array([0,0,0]) ,rotation_angle=0):
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

def distance_to_pose(drone_a, lat_b, long_b): # in meters

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

