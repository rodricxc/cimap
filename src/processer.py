#!/usr/bin/env python

import rospy
import math
import errno
import os
import numpy as np
import copy
import collections
import rospkg
import pickle

import robot
import mapa


inv = np.linalg.inv
det = np.linalg.det
 
class Solver(object):
    """docstring for Solver"""

    def __init__(self, show_output=False):
        super(Solver, self).__init__()

    def init_node(self):
        rospy.init_node('cimap_processer', anonymous=False)
        self._num_robots = rospy.get_param("/num_robots")
        self._robot_perception_distance = rospy.get_param("/robot_prerception_distance", 1)
        self._experiment_id = rospy.get_param("/experiment_id", 0)
        self._map_name = rospy.get_param("/map_name", 'default')
        self._stop_time = rospy.get_param("/stoptime", 60)

        self._map_resolution = rospy.get_param("/map_resolution", 0.1)
        self._ranger_max = 1
        self._ranger_irm_side = int(
            2*(1+math.ceil(self._ranger_max/self._map_resolution))+1)

        self.swarm = []

    def start_maps(self):
        self._map_gt = mapa.Mapa(self._map_resolution, "map_server_ideal")
        self._map_filtered = mapa.Mapa(self._map_resolution, "map_server_filtered")
        self._map_odom = mapa.Mapa(self._map_resolution, "map_server_noised")


    def publish_maps(self):
        self._map_gt.publishMap()
        self._map_odom.publishMap()
        self._map_filtered.publishMap()

    def update_maps(self, t):
        for r in self.swarm:
            s = r.getState(t, insert=False)
            if not s or s.laser is None:
                continue
            
            ranges = np.array(s.laser)
            k = float(len(ranges)+1.0)/len(ranges)

            try:
                # gt 
                pose = s.gt.mean
                proportion = 0.003
                
                rim = self.ranger_to_inverse_model(
                    ranges, 
                    np.pi*k+pose[2], 
                    l_0=0.5, l_occ=0.5+3*proportion, l_free=0.5-proportion)
                
                self._map_gt.logUpdateRegion(pos=pose[:2], m=rim)
                

                # filtered 
                pose = s.filtered.mean
                area = det(s.filtered.cov[:2,:2])
                proportion = 0.03 * (self._map_resolution **2)/(1.0*area)

                if proportion > 0.03:
                    proportion = 0.03
                
                # proportion = 0.003
                    
                rim = self.ranger_to_inverse_model(
                    ranges, 
                    np.pi*k+pose[2], 
                    l_0=0.5, l_occ=0.5+3*proportion, l_free=0.5-proportion)
                
                self._map_filtered.logUpdateRegion(pos=pose[:2], m=rim)
                 

                # odom 
                pose = s.odom.mean
                proportion = 0.003
                rim = self.ranger_to_inverse_model(
                    ranges, 
                    np.pi*k+pose[2], 
                    l_0=0.5, l_occ=0.5+3*proportion, l_free=0.5-proportion)
                
                self._map_odom.logUpdateRegion(pos=pose[:2], m=rim)
            except:
                print(s.gt.mean)
                print(s.filtered.mean)
                print(s.odom.mean)
                print 'error on update of robot %2d at time %5.2f' % (r.id, t)

    #  TODO: rewrite to optimise
    def ranger_to_inverse_model(self, ranger, ori, l_0, l_occ, l_free):

        ranger = np.array(ranger)
        dimension = self._ranger_irm_side

        irm = np.full((dimension, dimension), l_occ, dtype=np.float)
        slice_ang = 2*math.pi/len(ranger)
        center = np.array([(dimension+1)//2, (dimension+1)//2, ], dtype=np.int)

        poses = (np.swapaxes(np.swapaxes(
            np.mgrid[0:dimension, 0:dimension], 0, 1), 1, 2) - center)*self._map_resolution
        poses_val = np.sqrt(
            poses[:, :, 0]*poses[:, :, 0]+poses[:, :, 1]*poses[:, :, 1])

        poses_ang = np.arctan2(poses[:, :, 1], poses[:, :, 0]) - ori
        while np.any(poses_ang < 0):
            poses_ang[poses_ang < 0] += 2*math.pi
        poses_ang_d, poses_ang_m = np.divmod(poses_ang, slice_ang)
        poses_ang_m /= slice_ang

        p1 = poses_ang_d.astype(int)
        p2 = ((poses_ang_d+1) % len(ranger)).astype(int)

        limiar = ranger[p2]*poses_ang_m + ranger[p1]*(1-poses_ang_m)

        max_trashold = self._ranger_max-self._map_resolution

        irm[poses_val-1.0*self._map_resolution > limiar] = l_0
        irm[limiar > max_trashold] = l_0
        irm[poses_val+1.0*self._map_resolution < limiar] = l_free

        return irm

    def run(self):
        self.init_node()
        self.start_maps()

        self.rate = rospy.Rate(10)  # 10hz default
        self.rate.sleep()

        self.loadSwarm()
        print("[cimap_processer]: Swarm Loaded")

        # calcualting odometry for all robots
        odom_parameters = [0.01, 0.01, 0.01, 0.01]
        perception_noise = [0.05, 0.05]


        [r.calcOdometry(odom_parameters) for r in self.swarm]
        print("[cimap_processer]: Odometry processed")

        # init pose Publishers
        [r.initPosePublishers() for r in self.swarm]

        # set to use gps
        [r.useGps(False) for r in self.swarm]
        # [self.swarm[i].useGps(True) for i in {0, 5,9}]


        MAX_TIME = self._stop_time
        t=0
        # For each time
        while t <= MAX_TIME:
            if int(t*10)%10 == 0:            
                print('at time %5.2f'%t)
                self.publish_maps()
            ## PREDICTION STEP
            # -> calc estimator of controls AND
            #    estimate error matrix of control
            # -> calc transition matrixes and model for control
            # -> calc prediction State for filter, applying matrixes
            [r.prediction(t, odom_parameters) for r in self.swarm]
            
            ## UPDATE STEP
            [r.update(t, self.swarm, update_type='k', perception_noise=perception_noise) for r in self.swarm]
            # [r.update(t, self.swarm, update_type='ci', perception_noise=perception_noise) for r in self.swarm]
            


            # update maps
            self.update_maps(t)









            # DEBUG: Publiush calcs
            [r.publishPoses(t) for r in self.swarm[:]]
            # [r.publishPoses(t) for r in self.swarm[4:8]]
            rospy.sleep(0.002)
            if rospy.is_shutdown():
                return

            # time increase
            t+=0.1

        rospy.sleep(10)
        # end 


    def loadSwarm(self):
        print('[cimap_processer]: Loading Swarm Pickle')
        
        pack_path = rospkg.RosPack().get_path('cimap')
        dir_path = '{}/log/simulation/{}/{}/{}/'.format(pack_path,self._map_name, self._num_robots, self._experiment_id)
        
        # dump data into plicke object
        try:            
            file_input = open(dir_path+'swarm.pkl', 'rb')
        except Exception as e:
            print(e)
            raise Exception("Error while reading the picke file")
        
        self.swarm = pickle.load(file_input)        
        [r.onPickleLoad() for r in self.swarm]
        file_input.close()

if __name__ == '__main__':
    try:
        solver = Solver()
        solver.run()
        print "[cimap_processer]: Exiting node cimap_processer."
    except rospy.ROSInterruptException:
        pass





