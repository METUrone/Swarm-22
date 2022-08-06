#!/usr/bin/env python3
import rospy
from Swarm import Swarm
from cv import *
import numpy as np
import math
from custom_msg.msg import general_parameters #import custom_msg which is in the workspace
from swarm.srv import Pair, AStarReq

class Cargo:
    opponent_team_name = "team_" # dont forget _ at the end
    opponent_poses = {"C5":np.array([0,0,0]),"C6":np.array([0,0,0]),"C7":np.array([0,0,0])}
    
    @classmethod
    def set_opponent_topics(cls,teamname:str,ids:list):
        cls.opponent_team_name = teamname + "_"
        cls.opponent_poses = {f"{ids[0]:02X}":np.array([0,0,0]),f"{ids[1]:02X}":np.array([0,0,0]),f"{ids[2]:02X}":np.array([0,0,0])}

    @classmethod
    def opponent_listener_callback(cls,data):
        """
        string_1 = "\nTeam Name: " + str(data.team_name) +"\nUav Name: "+  str(data.uav_name) 
        string_2 ="\nRoll: " + str(data.rpy.roll) + "\nPitch: " + str(data.rpy.pitch) + "\nYaw: "+str(data.rpy.roll)
        string_3 ="\nx: " + str(data.pose.x) + "\ny: " + str(data.pose.y) + "\nz: " + str(data.pose.z)  
        string_data = string_1 + string_2 + string_3
        rospy.loginfo(rospy.get_caller_id() + " I heard %s", string_data)
        """
        cls.opponent_poses[str(data.uav_name)] = np.array([data.pose.x,data.pose.y,data.pose.z])

    @classmethod
    def opponent_center(cls):
        pos = np.array([0.0,0.0,0.0])
        for id in cls.opponent_poses:
            pos += cls.opponent_poses[id]
        return pos/3

    @classmethod
    def opponent_radius(cls):
        pos = cls.opponent_center()
        dist = 0
        for id in cls.opponent_poses:
            dist = max(dist,np.linalg.norm(pos - cls.opponent_poses[id]))
        return min(0.8,dist)

    @classmethod
    def check_right(cls):
        return cls.opponent_center()[1] > 0.1

    @classmethod
    def calcError(cls,x1,y1,x2,y2):
        return np.linalg.norm(np.array([x1,y1])-np.array([x2,y2]))

    @classmethod
    def callAStar(cls,start,dest,grid):
        s = Pair()
        s.first,s.second = int(start[0]),int(start[1])
        d = Pair()
        d.first,d.second = int(dest[0]),int(dest[1])
        try:
            resp = cls.caller(s,d,grid.flatten().tolist())
            if resp.pathFound:
                cls.last_path = resp.path
                cls.last_path.pop()
                return cls.last_path.pop() # return the next step
            else:
                if cls.last_path:
                    return cls.last_path.pop() # blocked, return the next step
                else:
                    return s # blocked, proceed anyway
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
            return s # stay where you are

    @classmethod
    def coord2idx(cls,x):
        return np.floor(np.clip((x+(cls.lim/2))*cls.grid_w/cls.lim,0,19)).astype("int")

    @classmethod
    def idx2coord(cls,x):
        return x*cls.lim/cls.grid_w - cls.lim/2

    @classmethod
    def createObstacle(cls,grid,x,y,ir= 5):
        ix = max(min(((x+(cls.lim/2))*cls.grid_w)/cls.lim,cls.grid_w),0)
        iy = max(min(((y+(cls.lim/2))*cls.grid_w)/cls.lim,cls.grid_w),0)
        # ir = max(min((0.6*20)//lim,19),0) #= 4.62
        grid[math.floor(max(0,ix-ir)):math.floor(min(cls.grid_w,ix+ir)),math.floor(max(0,iy-ir)):math.floor(min(cls.grid_w,iy+ir))] = 0

    @classmethod
    def execute(cls,z = 1,t1 = 5):
        cls.lim = 2.6 # width of the square our formation center can be in
        cls.grid_w = 20
        cls.dest_pt = np.array([-1.3,0])
        uav_count = 3
        radius = 0.4

        cls.last_path = None
        cls.base_grid = np.ones([cls.grid_w,cls.grid_w],dtype=int)
        rospy.init_node('rosapi', anonymous=True)
        print("Connecting to drones")
        for id in cls.opponent_poses:
            rospy.Subscriber("/general_parameters/" + cls.opponent_team_name + id, general_parameters, cls.opponent_listener_callback)
            print("Connected to drone with id " + id)
        print("Waiting for A* Handler")
        rospy.wait_for_service('/astar/calc')
        cls.caller = rospy.ServiceProxy('/astar/calc', AStarReq)
        print("Done!")
        
        swarm = Swarm(uav_count, "Crazyflie")
        current_pt = swarm.formation_center()
        swarm.init_pose_pub()
        swarm.hover(t1)
        swarm.form_via_potential_field(radius,z=z,displacement=(current_pt[0],current_pt[1],0))
        swarm.repulsive_pts = cls.opponent_poses
        opponent_pt = cls.opponent_center()
        current_idx = cls.coord2idx(current_pt[:2])
        while(cls.calcError(*cls.dest_pt[:2],*current_pt[:2]) > 0.3): #30 cm error allowed
            if opponent_pt[2] < (z-0.3): #Maybe the opponent falled
                grid = cls.base_grid
            else:
                grid = cls.base_grid.copy()
                cls.createObstacle(grid,opponent_pt[0],opponent_pt[1])
                grid[current_idx[0],current_idx[1]] = 1
                
            a = cls.callAStar(current_idx,cls.coord2idx(cls.dest_pt),grid)
            if False:
                grid[current_idx[0],current_idx[1]] = 8
                print(grid)
                print(round(cls.calcError(*cls.dest_pt,*current_pt[:2]),2),end="\t")
                print(current_idx,end="\t")
                print(current_pt[:2],end="\t")
                print(f"{a.first} {a.second}",end="\t")
                print(f"{cls.idx2coord(a.first):.2} {cls.idx2coord(a.second):.2}")
            swarm.form_via_potential_field(radius,z=z,displacement=(cls.idx2coord(a.first),cls.idx2coord(a.second),0))
            swarm.hover(0.2)
            current_idx = [a.first,a.second]
            current_pt = swarm.formation_center() #np.array([idx2coord(a.first),idx2coord(a.second),z])
            opponent_pt = cls.opponent_center()

        swarm.log_to_csv()
        swarm.land()

        swarm.timeHelper.sleep(4)

if __name__ == "__main__":
    Cargo.execute()