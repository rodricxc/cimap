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

import matplotlib.pyplot as plt

import robot
import mapa


from std_msgs.msg import Bool

class Solver(object):
    """docstring for Solver"""

    def __init__(self, show_output=False):
        super(Solver, self).__init__()

    def init_node(self):
        rospy.init_node('cimap_explorer', anonymous=False)
        self._num_robots = rospy.get_param("/num_robots")
        self._robot_perception_distance = rospy.get_param("/robot_prerception_distance", 1)
        self._experiment_id = rospy.get_param("/experiment_id", 0)
        self._map_name = rospy.get_param("/map_name", 'default')
        self.swarm = [robot.Robot(i) for i in range(self._num_robots)]
        self.stoptime = rospy.get_param("/stoptime", 60)
        self.stop = False
        self.sub_stopper = rospy.Subscriber("/stop_all", Bool, self.stopSim)

        self._map_resolution = 0.01
        self._map_gt = mapa.Mapa(self._map_resolution, "map_server_ideal")
        self._map_odom = mapa.Mapa(self._map_resolution, "map_server_odom")

        self._ranger_max = 0.1
        self._ranger_irm_side = int(
            2*(1+math.ceil(self._ranger_max/self._map_resolution))+1)


    def stopSim(self, data):
        print 'stoping simulation'
        self.stop = True
        [r.stop() for r in self.swarm]
        self.rate.sleep()
        self.rate.sleep()


    def run(self):
        self.init_node()

        self.rate = rospy.Rate(10)  # 10hz default
        self.rate.sleep()

        # store start time
        starttime = rospy.get_rostime()
        self.start_at = robot.stamptofloat(starttime)

        # start robots
        [r.start() for r in self.swarm]
        [r.useGps(False) for r in self.swarm]
        # [r.initPosePublishers() for r in self.swarm]


        while not rospy.is_shutdown() and not self.stop:
            now = rospy.get_rostime()
            # self.plot_path()
            # self.mapUpdate()
            # self._map_gt.publishMap()
            # self._map_odom.publishMap()

            self.rate.sleep()

        # self.calcAllHits()
        # [r.stop() for r in self.swarm]
        self.stoped_at = robot.stamptofloat(rospy.get_rostime())
        self.rate.sleep()
        self.rate.sleep()
        self.rate.sleep()


        if not  rospy.is_shutdown() :
            self.calcAllHits()
            self.saveSwarm()

    def mapUpdate(self):
        for r in self.swarm:

            s = r.getState().previous
            if s is None:
                continue
            s = s.previous
            t = 0
            
            if not s or s.laser is None:
                continue
            
            ranges = np.array(s.laser)
            k = float(len(ranges)+1.0)/len(ranges)

            try:
                # gt 
                pose = s.gt.mean
                pose[2] = robot.wrap_pi(pose[2])
                proportion = 0.03
                
                rim = self.ranger_to_inverse_model(
                    ranges, 
                    np.pi*k+pose[2], 
                    l_0=0.5, l_occ=0.5+3*proportion, l_free=0.5-proportion)
                
                self._map_gt.logUpdateRegion(pos=pose[:2], m=rim)

                # odom 
                pose = s.odom.mean
                pose[2] = robot.wrap_pi(pose[2])
                proportion = 0.03
                
                rim = self.ranger_to_inverse_model(
                    ranges, 
                    np.pi*k+pose[2], 
                    l_0=0.5, l_occ=0.5+3*proportion, l_free=0.5-proportion)
                
                self._map_odom.logUpdateRegion(pos=pose[:2], m=rim)
                
            except Exception as e:
                # print(pose)
                print e
                print 'error on update of robot %2d at time %5.2f' % (r.id, t)

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

    def plot_path(self):
        plt.figure(num='r0', figsize=(12,12))
        plt.clf()
        for r in self.swarm[:1]:

            x = []
            y = []

            s = r.getState()

            count = 2000
            # print s

            while s.gt is None:
                s = s.previous
            s = s.previous
            first = s.gt.mean
            while s is not None and count > 0:

                if s.gt is not None:
                    xa, ya = s.gt.mean[:2]
                    x.append(xa)
                    y.append(ya)

                count -=1
                s = s.previous

            ax = plt.axes()

            plt.plot(x,y,'-')
            ax.arrow(first[0], first[1], 0.1*np.cos(first[2]), 0.1*np.sin(first[2]), head_width=0.01, head_length=0.02, fc='k', ec='k')

        for r in self.swarm[:1]:

            x = []
            y = []

            s = r.getState()

            count = 2000
            # print s

            while s.gt is None:
                s = s.previous
            s = s.previous
            first = s.odom.mean
            while s is not None and count > 0:
                if s.gt is not None:
                    xa, ya = s.odom.mean[:2]
                    x.append(xa)
                    y.append(ya)

                count -=1
                s = s.previous

            ax = plt.axes()

            plt.plot(x,y,'-')
            ax.arrow(first[0], first[1], 0.1*np.cos(first[2]), 0.1*np.sin(first[2]), head_width=0.01, head_length=0.02, fc='k', ec='k')



        plt.axis([-0.9,0.9,-0.9,0.9])
        plt.show(block=False)
        plt.pause(0.001)




    def calcSavepath(self):
        pack_path = rospkg.RosPack().get_path('cimap')
        dir_path = '{}/log/simulation/{}/{}/{}/'.format(pack_path,self._map_name, self._num_robots, self._experiment_id)

        if not os.path.exists(dir_path):
            os.makedirs(dir_path)

        if not os.path.exists(dir_path+"maps"):
            os.makedirs(dir_path+"maps")

        return dir_path

    def calcAllHits(self):
        # process al robots for this time

        t = self.start_at
        print('[Status]: calculating hits')
        while t <= self.stoped_at:
            for r1 in self.swarm:
                for r2 in self.swarm:
                    if r1.id != r2.id:
                        ed, ori, rs1 = robot.r_distance(r1, r2, t)
                        if ed and ed <= self._robot_perception_distance:
                            rs1.addCloseRobot(robot.CloseRobot(r2.id, ed, ori)) 

            k = t - self.start_at

            if int(k*10)%100 == 0:
                print "calc at time %6.1f" % k

            t += 0.1



    def saveSwarm(self):

        
        print('[Status]: Saving Swarm Pickle')

        [r.set_max_min_times([self.start_at, self.stoped_at]) for r in self.swarm]

        dir_path = self.calcSavepath()
        
        # dump data into plicke object
        output = open(dir_path+'swarm.pkl', 'wb')
        [self.swarm[i].prepareToPickle() for i in range(len(self.swarm))]
        

        pickle.dump(self.swarm, output)
        output.close()


if __name__ == '__main__':
    try:
        solver = Solver()
        solver.run()
        print "[Status]: Exiting node cimap_explorer."
    except rospy.ROSInterruptException:
        pass





