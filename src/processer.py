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
        self._odom_err = rospy.get_param("/odom_error", 0.01)


        self._ranger_max = 1
        self._ranger_irm_side = int(
            2*(1+math.ceil(self._ranger_max/self._map_resolution))+1)

        self.swarm = []

        self.dir_path = self.calcSavepath()

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
                proportion = 0.01
                
                rim = self.ranger_to_inverse_model(
                    ranges, 
                    np.pi*k+pose[2], 
                    l_0=0.5, l_occ=0.5+3*proportion, l_free=0.5-proportion)
                
                self._map_gt.logUpdateRegion(pos=pose[:2], m=rim)
                

                # filtered 
                pose = s.filtered.mean
                area = np.log(det(s.filtered.cov[:2,:2])+1)
                proportion = 0.06 * (self._map_resolution **2)/(1.0*area)

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
                proportion = 0.01
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

        processed = self.loadSwarm()
        print("[cimap_processer]: Swarm Loaded")
        if processed:
            print "Swarm previously processed"
        
        # calcualting odometry for all robots
        odom_parameters = [self._odom_err, self._odom_err, self._odom_err, self._odom_err]
        # odom_parameters = [0.005, 0.005, 0.005, 0.005]
        perception_noise = [0.05, 0.05]
    
        # init pose Publishers
        [r.initPosePublishers() for r in self.swarm]

        if not processed:
            [r.calcOdometry(odom_parameters) for r in self.swarm]
            print("[cimap_processer]: Odometry processed")

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
                rospy.sleep(0.05)
            ## PREDICTION STEP
            # -> calc estimator of controls AND
            #    estimate error matrix of control
            # -> calc transition matrixes and model for control
            # -> calc prediction State for filter, applying matrixes
            if not processed:
                [r.prediction(t, odom_parameters) for r in self.swarm]
                
                ## UPDATE STEP
                [r.update(t, self.swarm, update_type='k', perception_noise=perception_noise) for r in self.swarm]
                # [r.update(t, self.swarm, update_type='ci', perception_noise=perception_noise) for r in self.swarm]
            
            # update maps
            self.update_maps(t)


            if int(t*10)%100 == 0:
                self.save_maps(t)

            # DEBUG: Publiush calcs
            [r.publishPoses(t) for r in self.swarm[:]]
            # [r.publishPoses(t) for r in self.swarm[4:8]]
            rospy.sleep(0.01)
            if rospy.is_shutdown():
                return

            # time increase
            t+=0.1

        if not processed:
            [r.setProcessed(True) for r in self.swarm]
            self.saveSwarm()
        
        print "mapping finished"
        while not rospy.is_shutdown():
            rospy.sleep(1)
            self.publish_maps()

        # end 

    def save_maps(self, t): 

        axis = [-16,16,-16,16]

        path = self.dir_path + 'maps/%s_%03d_t-%03d.png'%('filtered', np.floor(self._map_resolution*100), np.floor(t))
        self._map_filtered.plotMapa(path=path, round_at=[0.35,0.6], axis=axis)
        path = self.dir_path + 'maps/%s_%03d_t-%03d.png'%('gfiltered', np.floor(self._map_resolution*100), np.floor(t))
        self._map_filtered.plotMapa(path=path, axis=axis)       
        path = self.dir_path + 'maps/%s_%03d_t-%03d.png'%('gt', np.floor(self._map_resolution*100), np.floor(t))
        self._map_gt.plotMapa(path=path, round_at=[0.35,0.6], axis=axis)       
        path = self.dir_path + 'maps/%s_%03d_t-%03d.png'%('odom', np.floor(self._map_resolution*100), np.floor(t))
        self._map_odom.plotMapa(path=path, round_at=[0.35,0.6], axis=axis)       




    def calcSavepath(self):
        pack_path = rospkg.RosPack().get_path('cimap')
        dir_path = '{}/log/simulation/{}/{}/{}/'.format(pack_path,self._map_name, self._num_robots, self._experiment_id)

        if not os.path.exists(dir_path):
            os.makedirs(dir_path)

        if not os.path.exists(dir_path+"maps"):
            os.makedirs(dir_path+"maps")

        return dir_path

    def loadSwarm(self):
        print('[cimap_processer]: Loading Swarm Pickle')
        
        dir_path = self.calcSavepath()
        
        # dump data into plicke object
        
        try:            
            file_input = open(dir_path+'swarm_processed.pkl', 'rb')
            processed = True
        except Exception as e:
            try:            
                file_input = open(dir_path+'swarm.pkl', 'rb')
                processed = False
            except Exception as e:
                print(e)
                raise Exception("Error while reading the picke file")
        
        self.swarm = pickle.load(file_input)        
        [r.onPickleLoad() for r in self.swarm]
        file_input.close()
        return processed

    def saveSwarm(self):
        
        print('[Status]: Saving Swarm Pickle')

        [r.stopPosePublishers() for r in self.swarm]

        dir_path = self.calcSavepath()
        
        # dump data into plicke object
        output = open(dir_path+'swarm_processed.pkl', 'wb')
        [self.swarm[i].prepareToPickle() for i in range(len(self.swarm))]
        

        pickle.dump(self.swarm, output)
        output.close()

if __name__ == '__main__':
    try:
        solver = Solver()
        solver.run()
        print "[cimap_processer]: Exiting node cimap_processer."
    except rospy.ROSInterruptException:
        pass





