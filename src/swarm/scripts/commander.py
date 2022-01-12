#!/usr/bin/env python3

import rospy
from mavros_msgs.srv import CommandTOL 
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix

import math
import sys
from swarm.srv import PoseCommand

from copy import deepcopy

gps_pose = NavSatFix()
odomery_pose = Odometry()
id = 3

def wait_until_pose(x, y, z):
    rate = rospy.Rate(5)
    error = 0.5
    dx = x - odomery_pose.pose.pose.position.x
    dy = y - odomery_pose.pose.pose.position.y
    dz = z - odomery_pose.pose.pose.position.z
    distance = math.sqrt(abs(math.pow(dx,2) + math.pow(dy,2) + math.pow(dz,2)))

    while True:
        rate.sleep()
        
        dx = x - odomery_pose.pose.pose.position.x
        dy = y - odomery_pose.pose.pose.position.y
        dz = z - odomery_pose.pose.pose.position.z
        distance = math.sqrt(abs(math.pow(dx,2) + math.pow(dy,2) + math.pow(dz,2)))


        if distance < error:
            return True
        else:
            continue

def current_gps_pose_callback(data):
    global gps_pose
    gps_pose = deepcopy(data)

def current_odometry_pose_callback(data):
    global odomery_pose
    odomery_pose = deepcopy(data)
    



def current_odometry_pose():
    global id
    odometry_sub = rospy.Subscriber("/uav{}/mavros/global_position/local".format(id), Odometry, current_odometry_pose_callback)

def current_gps_pose():
    global id
    pose_sub = rospy.Subscriber("/uav{}/mavros/global_position/global".format(id), NavSatFix, current_gps_pose_callback)

def land(): # doesn't work !!!!!!!!!!!! 
    global id
    try:

        pose_commander(odomery_pose.pose.pose.position.x, odomery_pose.pose.pose.position.y, 0)

        rospy.wait_for_service("/uav{}/mavros/cmd/land".format(id))

        land_service = rospy.ServiceProxy("/uav{}/mavros/cmd/land".format(id), CommandTOL)
        land_service(min_pitch=0, yaw=0, latitude=0, longitude=0, altitude=0)

        print("land service called")

    except rospy.ServiceException as e:

        print("Service call failed: %s"%e)
        return False

def pose_commander(x, y, z):
    global id

    try:

        rospy.wait_for_service("PoseCommand{}".format(id))

        client = rospy.ServiceProxy("PoseCommand{}".format(id), PoseCommand)
        resp = client(x, y, z)

        #wait_until_pose(x, y, z)
        rospy.sleep(5)

        return resp
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
        return False

def draw_square(length):

    pose_commander(length, 0, length)

    pose_commander(length, length, length)

    pose_commander(0, length, length)

    pose_commander(0, 0, length)
    

if __name__ == "__main__":
    id = sys.argv[1]
    rospy.init_node("commander{}".format(id), anonymous=True)

    rate = rospy.Rate(0.5)
    rate.sleep()

    try:

        current_gps_pose()
        current_odometry_pose()
        
        print("waiting...")

        while True:
            rate.sleep()
            if odomery_pose.pose.pose.position.x != 0:
                print("ready")
                break

        draw_square(5)            


    except rospy.exceptions.ROSInterruptException:
        print("\nshutdown")

    
    