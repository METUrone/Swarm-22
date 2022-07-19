import rospy
import copy
from munkres import Munkres
from datetime import datetime
import csv
import random

from TurtleBot import TurtleBot
from pycrazyswarm import Crazyswarm
from utils import *

from custom_msg.msg import general_parameters #import custom_msg which is in the workspace

opponent_poses = {"C5":general_parameters(),"C6":general_parameters(),"C7":general_parameters()}

class Swarm_commander:
    def __init__(self,num_of_drones,vehicle, first_time = True, crazyswarm_class=None):
        self.agents = []
        self.vehicle = vehicle
        self.radius = 2
        self.num_of_edges = 3
        self.obstacles = {}

        self.log = {
            "speed": np.array_str(np.array([0,0,0])),
            "time" : datetime.now().strftime("%H:%M:%S:%f"),
            "position" : np.array_str(np.array([0,0,0])),
            "id" : str(0)
        }

        self.logs = {}

        if vehicle == "Crazyflie":
            if first_time:
                self.crazyswarm = Crazyswarm()
                self.agents = self.crazyswarm.allcfs.crazyflies[0:num_of_drones]
                self.timeHelper = self.crazyswarm.timeHelper

                for i in range(len(self.agents)):
                    self.takeoff(i)
            else:
                self.timeHelper = crazyswarm_class.timeHelper
            
            self.timeHelper.sleep(1)

        self.num_of_agents = num_of_drones

        simulation_origin_latitude = 47.3977419
        simulation_origin_longitude = 8.5455935

    def log_to_csv(self):
        for i in self.logs:

            try:

                with open("./agent{}_logs.csv".format(i),"w") as f:
                    writer_csv = csv.writer(f)
                    header = self.log.keys()
                    header_str = []

                    for j in header:
                        j = '{0:''<10}'.format(j)
                        header_str.append(j)

                    #print(header_str)
                    writer_csv.writerow(header_str)
                    for j in self.logs[i]:

                        writer_csv.writerow(j.values())
                    print("LOGGING COMPLETED !!!")
            except LookupError:
                print("FILE COULD NOT OPENED")

    def add_log(self, speed, time, position, id):
        self.log["speed"] = speed
        self.log["time"] = time
        self.log["position"] = position
        self.log["id"] = id

        try:
            self.logs[str(id)].append(copy.copy(self.log))
        except KeyError:
            self.logs[str(id)] = []
            self.logs[str(id)].append(copy.copy(self.log))



def opponent_listener_callback(data):
    """
    string_1 = "\nTeam Name: " + str(data.team_name) +"\nUav Name: "+  str(data.uav_name) 
    string_2 ="\nRoll: " + str(data.rpy.roll) + "\nPitch: " + str(data.rpy.pitch) + "\nYaw: "+str(data.rpy.roll)
    string_3 ="\nx: " + str(data.pose.x) + "\ny: " + str(data.pose.y) + "\nz: " + str(data.pose.z)  
    string_data = string_1 + string_2 + string_3
    rospy.loginfo(rospy.get_caller_id() + " I heard %s", string_data)
    """
    opponent_poses[str(data.uav_name)] = data

def init_opponent_listener():
    for id in opponent_poses:
        rospy.Subscriber("/general_parameters/METU-rone_" + id, general_parameters, opponent_listener_callback)


def main():
    rospy.init_node('rosapi', anonymous=True)




if __name__ == "__main__":
    main()