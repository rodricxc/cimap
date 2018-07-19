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
        self.swarm = []


    def run(self):
        self.init_node()

        self.rate = rospy.Rate(10)  # 10hz default
        self.rate.sleep()

        self.loadSwarm()
        print("[cimap_processer]: Swarm Loaded")

        # calcualting odometry for all robots
        odom_parameters = [0.01, 0.01, 0.01, 0.01]
        [r.calcOdometry(odom_parameters) for r in self.swarm]
        print("[cimap_processer]: Odometry processed")

        # init pose Publishers
        [r.initPosePublishers() for r in self.swarm]


        MAX_TIME = 60
        t=0
        # For each time
        while t <= MAX_TIME:            

            ## PREDICTION STEP
            # -> calc estimator of controls AND
            #    estimate error matrix of control
            [r.calcControlAndError(t) for r in self.swarm]
            # -> calc transition matrixes and model for control
            [r.calcTransitionMatrixes(t) for r in self.swarm]
            # -> calc prediction State for filter, applying matrixes
            [r.calcPrediction(t) for r in self.swarm]

            ## UPDATE STEP





        # t = 0 
        # while not rospy.is_shutdown():
        #     t += 0.1
        #     self.swarm[0].publishPoses(t)
        #     self.swarm[1].publishPoses(t)
        #     self.swarm[2].publishPoses(t)
        #     self.swarm[3].publishPoses(t)
        #     # self.rate.sleep()
        #     rospy.sleep(0.04)


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





