#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

#rospy.init_node("turtlebot", anonymous=True)


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

    def velocity_command(self=0, linear_x=0, linear_y=0, angular_z=0):
        print(linear_y)
        self.vel.linear.x = linear_x
        self.vel.angular.z = angular_z
        self.vel_pub.publish(self.vel)

    def get_global_pose(self):
        return self.pose

    def odometry_callback(self, data):
        self.pose.pose.pose.position.x = data.pose.pose.position.x  
        self.pose.pose.pose.position.y = data.pose.pose.position.y
        self.pose.pose.pose.position.z = data.pose.pose.position.z

    