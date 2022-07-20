import numpy as np
from Swarm import Swarm
from pycrazyswarm import Crazyswarm

class Cluster:
    def __init__(self, cluster_count, cluster_size, radius = 1.5) -> None:
        self.crazyswarm = Crazyswarm()
        self.clusters = []
        self.cluster_size = cluster_size
        self.centers = []
        self.radius = radius # max(ip_uzunluğu, formasyon yarıçapı) olarak seçilecek
        for i in range(cluster_count):
            new_swarm = Swarm(cluster_size, "Crazyflie", False, self.crazyswarm)
            new_swarm.agents = self.crazyswarm.allcfs.crazyflies[i*cluster_size:(i+1)*cluster_size]
            for i in range(len(new_swarm.agents)):
                new_swarm.takeoff(i, 1)
            self.crazyswarm.timeHelper.sleep(3)
            self.clusters.append(new_swarm)

    def initailize_positions(self, positions):
        for i in range(len(positions)):
            self.clusters[i].form_polygon(self.radius, self.cluster_size, displacement = positions[i])
            self.centers.append(positions[i])

    def update_centers(self):
        for i in range(len(self.clusters)):
            x_pose = 0
            y_pose = 0
            z_pose = 0
            for j in range(self.cluster_size):
                x_pose += self.clusters[i].agents[j].position()[0]
                y_pose += self.clusters[i].agents[j].position()[1]
                z_pose += self.clusters[i].agents[j].position()[2]
            self.centers[i] = [x_pose/self.cluster_size, y_pose/self.cluster_size, z_pose/self.cluster_size]

    def is_goal_reached(self, id, target, error):
        center = self.centers[id]

        distance = ( (center[0]-target[0])**2 + (center[1]-target[1])**2 + (center[2]-target[2])**2 )**(1/2)
        
        if distance <= error:
            return True
        else:
            return False

    def is_all_delivered(self, targets):
        delivered = 0
        for i in range(len(self.clusters)):
            if self.is_goal_reached(i, targets[i], 0.2):
                delivered += 1
        
        if delivered == len(self.clusters):
            return True
        else:
            return False

    def attractive_force(self, id, target_pose, attractive_constant = 2):
        speed_limit = 0.9 # must be float

        attractive_force_x = (target_pose[0] - self.centers[id][0])*attractive_constant
        attractive_force_y = (target_pose[1] - self.centers[id][1])*attractive_constant
        attractive_force_z = (target_pose[2] - self.centers[id][2])*attractive_constant

        return min(max(float(attractive_force_x),-1*speed_limit),speed_limit), min(max(float(attractive_force_y),-1*speed_limit),speed_limit), min(max(float(attractive_force_z),-1*speed_limit),speed_limit)


    def repulsive_force(self,id, repulsive_constant =-1.5,repulsive_threshold = 1.3):
        repulsive_force_x = 0
        repulsive_force_y = 0
        repulsive_force_z = 0
        speed_limit = 0.5 # must be float

        for i in range(len(self.centers)):
                if i == id:
                    continue
                z_distance = self.centers[id][2] - self.centers[i][2]
                y_distance = self.centers[id][1] - self.centers[i][1]
                x_distance = self.centers[id][0] - self.centers[i][0]

                z_distance = (z_distance - self.radius) if (z_distance > 0 )else (z_distance + self.radius)
                y_distance = (y_distance - self.radius) if (y_distance > 0 )else (y_distance + self.radius)
                x_distance = (x_distance - self.radius) if (x_distance > 0 )else (x_distance + self.radius) 

                if z_distance != 0:
                    repulsive_force_z += (1/(z_distance**2))*(1/repulsive_threshold - 1/z_distance)*repulsive_constant * (-(z_distance) / abs(z_distance))
                if y_distance != 0:
                    repulsive_force_y += (1/(y_distance**2))*(1/repulsive_threshold - 1/y_distance)*repulsive_constant * (-(y_distance) / abs(y_distance))
                if x_distance != 0:
                    repulsive_force_x += (1/(x_distance**2))*(1/repulsive_threshold - 1/x_distance)*repulsive_constant * (-(x_distance) / abs(x_distance))

        return min(max(float(repulsive_force_x),-1*speed_limit),speed_limit), min(max(float(repulsive_force_y),-1*speed_limit),speed_limit), min(max(float(repulsive_force_z),-1*speed_limit),speed_limit)


    def single_potential_field(self, id, target):
        attractive_force_x, attractive_force_y, attractive_force_z = self.attractive_force(target_pose=target, id=id)
        repulsive_force_x, repulsive_force_y, repulsive_force_z = self.repulsive_force(id=id)

        vel_x = attractive_force_x + repulsive_force_x
        vel_y = attractive_force_y + repulsive_force_y
        vel_z = attractive_force_z + repulsive_force_z

        for i in range(self.cluster_size):
            self.clusters[id].agents[i].cmdVelocityWorld(np.array([vel_x, vel_y, vel_z]), yawRate=0)
            self.crazyswarm.timeHelper.sleep(0.001)


    def deliver_via_potential_field(self, targets):
        self.update_centers()
        while not self.is_all_delivered(targets):

            self.update_centers()

            reached = []
            for i in range(len(self.clusters)):
                
                if i in reached: 
                    continue

                if self.is_goal_reached(i, targets[i], 0.2):
                    reached.append(i)
                    for i in range(self.cluster_size):
                        self.clusters[id].agents[i].cmdVelocityWorld(np.array([0.0, 0.0, 0.0]), 0.0)
                        self.crazyswarm.timeHelper.sleep(0.001)
                    continue

                self.single_potential_field(i, targets[i])
        
        self.stop_all()

    def stop_all(self):
        for crazyflie in self.crazyswarm.allcfs.crazyflies:
            crazyflie.cmdVelocityWorld(np.array([0.0, 0.0, 0.0]), yawRate=0)