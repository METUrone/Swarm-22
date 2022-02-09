#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

rospy.init_node("turtle_bot")


class TurtleBot:
    def __init__(self, id):
        self.id = id
        self.vel = Twist()
        self.vel.linear.x = 0
        self.vel.angular.z = 0

        self.pose = Odo

        self.vel_pub = rospy.Publisher("/tb3_{}/cmd_vel".format(id), Twist, queue_size=1)

        print("TurtleBot{} waiting...".format(self.id))
        for i in range(10):
            self.vel_pub.publish(self.vel)
            rospy.sleep(0.1)

        print("TurtleBot{} ready".format(self.id))

    def velocity(self=0, linear_x=0, linear_y=0, angular_z=0):
        self.vel.linear.x = linear_x
        self.vel.linear.y = linear_y
        self.vel.angular.z = angular_z
        self.vel_pub.publish(self.vel)

    def pose_getter(self):
        self.pose

    