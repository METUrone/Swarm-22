#!/usr/bin/env python3

import rospy

from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TwistStamped

from swarm.srv import PoseCommand, VelocityCommand

from copy import deepcopy

class Iris:
    def __init__(self, id):
        self.id = id
        rospy.init_node("iris", anonymous=True)

        rate = rospy.Rate(10)
        rate.sleep()

        self.odomery_pose = Odometry()
        self.gps_pose = NavSatFix()
        self.starting_x = 0
        self.starting_y = 0

        self.pose_sub = rospy.Subscriber("/uav{}/mavros/global_position/global".format(self.id), NavSatFix, self.current_gps_pose_callback)
        self.odometry_sub = rospy.Subscriber("/uav{}/mavros/global_position/local".format(self.id), Odometry, self.current_odometry_pose_callback)

        self.global_pose = Odometry()
        self.global_pose.pose.pose.position.x = self.starting_x + self.odomery_pose.pose.pose.position.x 
        self.global_pose.pose.pose.position.y = self.starting_y + self.odomery_pose.pose.pose.position.y 
        self.global_pose.pose.pose.position.z = self.odomery_pose.pose.pose.position.z

        print("Iris{} waiting...".format(self.id))
        
        while self.gps_pose_getter().latitude == 0:
            rate.sleep()

        print("Iris{} ready".format(self.id))


    def current_gps_pose_callback(self, data):
        
        self.gps_pose = deepcopy(data)

    def current_odometry_pose_callback(self, data):
        
        self.odomery_pose = deepcopy(data)
        self.global_pose.pose.pose.position.x = self.starting_x + self.odomery_pose.pose.pose.position.x
        self.global_pose.pose.pose.position.y = self.starting_y + self.odomery_pose.pose.pose.position.y 
        self.global_pose.pose.pose.position.z = self.odomery_pose.pose.pose.position.z
        

    def gps_pose_getter(self):
        return self.gps_pose

    def set_starting_pose(self, x, y):
        self.starting_x = x
        self.starting_y = y
        self.global_pose.pose.pose.position.x = self.starting_x + self.odomery_pose.pose.pose.position.x 
        self.global_pose.pose.pose.position.y = self.starting_y + self.odomery_pose.pose.pose.position.y 
        self.global_pose.pose.pose.position.z = self.odomery_pose.pose.pose.position.z

    def get_global_pose(self):
        #print("x: {}\ny: {}\n".format(self.global_pose.pose.pose.position.x, self.global_pose.pose.pose.position.y))
        return self.global_pose

    def get_odometry_pose(self):
        print(self.odomery_pose)
        return self.odomery_pose

    def move_global(self, x, y, z):
        self.move_local(x-self.starting_x, y-self.starting_y, z)

    def move_local(self, x, y, z):
    
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

    def velocity_command(self, linear_x=0, linear_y=0, linear_z=0, angular_x=0, angular_y=0, angular_z=0):
        
        try:

            rospy.wait_for_service("VelocityCommand{}".format(self.id))
            client = rospy.ServiceProxy("VelocityCommand{}".format(self.id), VelocityCommand)

            resp = client.call(linear_x, linear_y, linear_z, angular_x, angular_y, angular_z)
            return resp

        except rospy.ServiceException as e:
            print("Velocity Service call failed: %s"%e)


    def draw_square(self, length):
        z = 5
        current_x = self.odomery_pose.pose.pose.position.x
        current_y = self.odomery_pose.pose.pose.position.y

        self.move_local(current_x + length, current_y + 0, z)

        self.move_local(current_x + length, current_y + length, z)

        self.move_local(current_x + 0, current_y + length, z)

        self.move_local(current_x + 0, current_y + 0, z)

    

    