#!/usr/bin/env python3
import rospy
from Swarm import Swarm
import numpy as np
import math
from custom_msg.msg import general_parameters #import custom_msg which is in the workspace
from swarm.srv import Pair, AStarReq

def opponent_listener_callback(data):
    """
    string_1 = "\nTeam Name: " + str(data.team_name) +"\nUav Name: "+  str(data.uav_name) 
    string_2 ="\nRoll: " + str(data.rpy.roll) + "\nPitch: " + str(data.rpy.pitch) + "\nYaw: "+str(data.rpy.roll)
    string_3 ="\nx: " + str(data.pose.x) + "\ny: " + str(data.pose.y) + "\nz: " + str(data.pose.z)  
    string_data = string_1 + string_2 + string_3
    rospy.loginfo(rospy.get_caller_id() + " I heard %s", string_data)
    """
    opponent_poses[str(data.uav_name)] = np.array([data.pose.x,data.pose.y,data.pose.z])

def opponent_center():
    pos = np.array([0.0,0.0,0.0])
    for id in opponent_poses:
        pos += opponent_poses[id]
    return pos/3

def opponent_radius():
    pos = opponent_center()
    dist = 0
    for id in opponent_poses:
        dist = max(dist,np.linalg.norm(pos - opponent_poses[id]))
    return min(0.8,dist)

def check_right():
    return opponent_center()[1] > 0.1

def calcError(x1,y1,x2,y2):
    return np.linalg.norm(np.array([x1,y1])-np.array([x2,y2]))

def callAStar(start,dest,grid):
    global last_path
    s = Pair()
    s.first,s.second = int(start[0]),int(start[1])
    d = Pair()
    d.first,d.second = int(dest[0]),int(dest[1])
    try:
        resp = caller(s,d,grid.flatten().tolist())
        if resp.pathFound:
            last_path = resp.path
            last_path.pop()
            return last_path.pop() # return the next step
        else:
            if last_path:
                return last_path.pop() # blocked, return the next step
            else:
                return s # blocked, proceed anyway
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
        return s # stay where you are

def coord2idx(x):
    return np.floor(np.clip((x+(lim/2))*grid_w/lim,0,19)).astype("int")

def idx2coord(x):
    return x*lim/grid_w - lim/2

def createObstacle(grid,x,y,ir= 5):
    ix = max(min(((x+(lim/2))*grid_w)/lim,grid_w),0)
    iy = max(min(((y+(lim/2))*grid_w)/lim,grid_w),0)
    # ir = max(min((0.6*20)//lim,19),0) #= 4.62
    grid[math.floor(max(0,ix-ir)):math.floor(min(grid_w,ix+ir)),math.floor(max(0,iy-ir)):math.floor(min(grid_w,iy+ir))] = 0

if __name__ == "__main__":
    opponent_team_name = "team_" # dont forget _ at the end
    opponent_poses = {"C5":np.array([0,0,0]),"C6":np.array([0,0,0]),"C7":np.array([0,0,0])}
    z = 1
    t1 = 5
    lim = 2.6 # width of the square our formation center can be in
    grid_w = 20
    dest_pt = np.array([-1.3,0])
    uav_count = 3
    radius = 0.4

    last_path = None
    base_grid = np.ones([grid_w,grid_w],dtype=int)
    rospy.init_node('rosapi', anonymous=True)
    print("Connecting to drones")
    for id in opponent_poses:
        rospy.Subscriber("/general_parameters/" + opponent_team_name + id, general_parameters, opponent_listener_callback)
        print("Connected to drone with id " + id)
    print("Waiting for A* Handler")
    rospy.wait_for_service('/astar/calc')
    caller = rospy.ServiceProxy('/astar/calc', AStarReq)
    print("Done!")
    
    swarm = Swarm(uav_count, "Crazyflie")
    current_pt = swarm.formation_center()
    swarm.init_pose_pub()
    swarm.hover(t1)
    swarm.form_via_potential_field(radius,z=z,displacement=(current_pt[0],current_pt[1],0))
    swarm.repulsive_pts = opponent_poses
    opponent_pt = opponent_center()
    current_idx = coord2idx(current_pt[:2])
    while(calcError(*dest_pt[:2],*current_pt[:2]) > 0.3): #30 cm error allowed
        if opponent_pt[2] < (z-0.3): #Maybe the opponent falled
            grid = base_grid
        else:
            grid = base_grid.copy()
            createObstacle(grid,opponent_pt[0],opponent_pt[1])
            grid[current_idx[0],current_idx[1]] = 1
            
        a = callAStar(current_idx,coord2idx(dest_pt),grid)
        if False:
            grid[current_idx[0],current_idx[1]] = 8
            print(grid)
            print(round(calcError(*dest_pt,*current_pt[:2]),2),end="\t")
            print(current_idx,end="\t")
            print(current_pt[:2],end="\t")
            print(f"{a.first} {a.second}",end="\t")
            print(f"{idx2coord(a.first):.2} {idx2coord(a.second):.2}")
        swarm.form_via_potential_field(radius,z=z,displacement=(idx2coord(a.first),idx2coord(a.second),0))
        swarm.hover(0.2)
        current_idx = [a.first,a.second]
        current_pt = swarm.formation_center() #np.array([idx2coord(a.first),idx2coord(a.second),z])
        opponent_pt = opponent_center()

    swarm.log_to_csv()
    swarm.land()

    swarm.timeHelper.sleep(4)