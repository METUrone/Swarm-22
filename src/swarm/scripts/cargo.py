#!/usr/bin/env python3
import rospy
from Swarm import Swarm

from custom_msg.msg import general_parameters #import custom_msg which is in the workspace

def opponent_listener_callback(data):
    """
    string_1 = "\nTeam Name: " + str(data.team_name) +"\nUav Name: "+  str(data.uav_name) 
    string_2 ="\nRoll: " + str(data.rpy.roll) + "\nPitch: " + str(data.rpy.pitch) + "\nYaw: "+str(data.rpy.roll)
    string_3 ="\nx: " + str(data.pose.x) + "\ny: " + str(data.pose.y) + "\nz: " + str(data.pose.z)  
    string_data = string_1 + string_2 + string_3
    rospy.loginfo(rospy.get_caller_id() + " I heard %s", string_data)
    """
    opponent_poses[str(data.uav_name)] = [data.pose.x,data.pose.y,data.pose.z]

def opponent_center():
    x,y,z = 0,0,0
    for id in opponent_poses:
        x += opponent_poses[id][0]
        y += opponent_poses[id][1]
        z += opponent_poses[id][2]
    return x/3,y/3,z/3

def check_right():
    return opponent_center()[1] > 0.1

if __name__ == "__main__":
    opponent_team_name = "team_" # dont forget _ at the end
    opponent_poses = {"C5":[-1.5,-1.8,1],"C6":[-1.5,-1,1],"C7":[-1.0,-1.3,1]}

    rospy.init_node('rosapi', anonymous=True)
    for id in opponent_poses:
        rospy.Subscriber("/general_parameters/" + opponent_team_name + id, general_parameters, opponent_listener_callback)
        print(id)
    
    d = 1.5
    w = 0.1
    current_side = True #right == True
    uav_count = 3
    radius = 0.5
    vehicle1 = "Iris"
    vehicle2 = "Crazyflie"
    swarm = Swarm(uav_count, vehicle2)
    swarm.init_pose_pub()
    swarm.hover(5)
    swarm.form_via_potential_field(radius,displacement=(d,w,0))
    swarm.repulsive_pts = opponent_poses
    swarm.hover(3)
    while d > -1.5:
        if check_right() == current_side:
            d -= 0.1
            if abs(w) < 0.6:
                if current_side:
                    w -= 0.1
                else:
                    w += 0.1
        else:
            current_side = check_right()
            if current_side:
                w -= 0.1
            else:
                w += 0.1
        swarm.form_via_potential_field(radius,displacement=(d,max(-1.3,min(1.3,w)),0))
        swarm.hover(1)

    swarm.log_to_csv()
    swarm.land()

    swarm.timeHelper.sleep(4)