#!/usr/bin/env python3

import rospy
from mavros_msgs.srv import CommandTOL, CommandVtolTransition
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix

import math
from swarm.srv import PoseCommand

from copy import deepcopy

class Vtol:
    def __init__(self, id):
        self.id = id
        rospy.init_node("Vtol", anonymous=True)

        rate = rospy.Rate(10)
        rate.sleep()

        self.odomery_pose = Odometry()
        self.gps_pose = NavSatFix()

        self.pose_sub = rospy.Subscriber("/uav{}/mavros/global_position/global".format(self.id), NavSatFix, self.current_gps_pose_callback)
        self.odometry_sub = rospy.Subscriber("/uav{}/mavros/global_position/local".format(self.id), Odometry, self.current_odometry_pose_callback)

        print("Vtol{} waiting...".format(self.id))
        
        while self.gps_pose_getter().latitude == 0:
            rate.sleep()

        print("Vtol{} ready".format(self.id))


    def current_gps_pose_callback(self, data):
        
        self.gps_pose = deepcopy(data)

    def current_odometry_pose_callback(self, data):
        
        self.odomery_pose = deepcopy(data)

    def gps_pose_getter(self):
        return self.gps_pose


    def pose_commander(self, x, y, z):
    

        try:

            rospy.wait_for_service("PoseCommand{}".format(self.id))

            client = rospy.ServiceProxy("PoseCommand{}".format(self.id), PoseCommand)
            resp = client(x, y, z)

            #wait_until_pose(x, y, z)
            rospy.sleep(2)

            return resp
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
            return False

    def draw_square(self, length):

        self.pose_commander(length, 0, length)

        self.pose_commander(length, length, length)

        self.pose_commander(0, length, length)

        self.pose_commander(0, 0, length)

    def vtol_transition(self):
        rospy.wait_for_service("/uav{}/mavros/cmd/vtol_transition".format(self.id))
        client = rospy.ServiceProxy("/uav{}/mavros/cmd/vtol_transition".format(self.id), CommandVtolTransition)
        resp = client(state= 4)
        return resp