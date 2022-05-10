#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
import math
 
def euler_from_quaternion(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians

""" rospy.init_node("Turtle", anonymous=True) """


class TurtleBot:
    def __init__(self, id):
        self.id = id
        self.vel = Twist()
        self.vel.linear.x = 0
        self.vel.angular.z = 0

        self.pose = Odometry()

        self.vel_pub = rospy.Publisher("/tb3_{}/cmd_vel".format(id), Twist, queue_size=1)
        rospy.Subscriber("/tb3_{}/odom".format(self.id), Odometry,self.odometry_callback)

        print("TurtleBot{} waiting...".format(self.id))
        for i in range(10):
            self.vel_pub.publish(self.vel)
            rospy.sleep(0.1)

        print("TurtleBot{} ready".format(self.id))

    def velocity_command(self, linear_x=0, angular_z=0):
        self.vel.linear.x = linear_x
        self.vel.angular.z = angular_z
        self.vel_pub.publish(self.vel)

    def get_global_pose(self):
        return self.pose

    def cmdVelocityWorld(self, vel, yawRate=0):
        self.vel.linear.x = vel[0]
        self.vel.angular.z = yawRate
        self.vel_pub.publish(self.vel)

    def position(self):
        return np.array([self.pose.pose.pose.position.x, self.pose.pose.pose.position.y, self.pose.pose.pose.position.z])

    def orientation(self):
        yaw = euler_from_quaternion(self.pose.pose.pose.orientation.x, self.pose.pose.pose.orientation.y, self.pose.pose.pose.orientation.z, self.pose.pose.pose.orientation.w)[2]
        return yaw if yaw >= 0 else yaw + math.pi * 2

    def turn_to(self, yaw, only_calculate=False): #when only calculate True it does not give velocity command but just returns it.
        error = 0.2 / 57.2957795
        gain = 2
        curr_yaw = self.orientation()

        begin_sec = rospy.Time.now()

        while abs(yaw - curr_yaw) > error:
            if rospy.Time.now() - begin_sec > rospy.Duration(3):
                print("Time Out !!!!!!!!")
                return
            curr_yaw = self.orientation()
            if not only_calculate:
                self.cmdVelocityWorld(np.array([0, 0, 0]), gain*(yaw - curr_yaw))
            else:
                return gain*(yaw - curr_yaw)

        if not only_calculate:
            print("turned to: " + str(self.orientation() * 57.2957795))
        self.cmdVelocityWorld(np.array([0, 0, 0]), 0)


    def odometry_callback(self, data):
        self.pose = data

    