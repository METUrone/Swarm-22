import rospy
from Swarm import Swarm
import numpy as np

from geometry_msgs.msg import Point

def ika(swarm:Swarm,coords):

    swarm.hover(2)

