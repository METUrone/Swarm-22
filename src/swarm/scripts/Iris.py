#!/usr/bin/env python3

import rospy
from mavros_msgs.srv import CommandTOL 
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix

import math
from swarm.srv import PoseCommand

from copy import deepcopy

class Iris:
    def __init__(self, id):
        self.id = id
        rospy.init_node("iris", anonymous=True)

        rate = rospy.Rate(0.5)
        rate.sleep()

        self.odomery_pose = Odometry()
        self.gps_pose = NavSatFix()

        self.pose_sub = rospy.Subscriber("/uav{}/mavros/global_position/global".format(self.id), NavSatFix, self.current_gps_pose_callback)
        self.odometry_sub = rospy.Subscriber("/uav{}/mavros/global_position/local".format(self.id), Odometry, self.current_odometry_pose_callback)

        
        print("Iris{} waiting...".format(self.id))

        while True:
            rate.sleep()
            if self.odomery_pose.pose.pose.position.x != 0:
                print("Iris{} ready".format(self.id))
                break

    def wait_sec(sec):
        rate = rospy.Rate(1/sec)
        rate.sleep()

    def current_gps_pose_callback(self, data):
        
        self.gps_pose = deepcopy(data)

    def current_odometry_pose_callback(self, data):
        
        self.odomery_pose = deepcopy(data)


    def pose_commander(self, x, y, z):
    

        try:

            rospy.wait_for_service("PoseCommand{}".format(self.id))

            client = rospy.ServiceProxy("PoseCommand{}".format(self.id), PoseCommand)
            resp = client(x, y, z)

            #wait_until_pose(x, y, z)
            rospy.sleep(5)

            return resp
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
            return False

    def draw_square(self, length):

        self.pose_commander(length, 0, length)

        self.pose_commander(length, length, length)

        self.pose_commander(0, length, length)

        self.pose_commander(0, 0, length)