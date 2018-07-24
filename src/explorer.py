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
        rospy.init_node('cimap_explorer', anonymous=False)
        self._num_robots = rospy.get_param("/num_robots")
        self._robot_perception_distance = rospy.get_param("/robot_prerception_distance", 1)
        self._experiment_id = rospy.get_param("/experiment_id", 0)
        self._map_name = rospy.get_param("/map_name", 'default')
        self.swarm = [robot.Robot(i) for i in range(self._num_robots)]
        self.stoptime = rospy.get_param("/stoptime", 60)


    def run(self):
        self.init_node()

        self.rate = rospy.Rate(10)  # 10hz default
        self.rate.sleep()

        # store start time
        starttime = rospy.get_rostime()
        self.start_at = starttime

        # start robots
        [r.start() for r in self.swarm]
        [r.useGps(False) for r in self.swarm]

        while not rospy.is_shutdown():
            now = rospy.get_rostime()
            # stop after k seconds
            if now.secs >= self.stoptime:
                [r.stop() for r in self.swarm]
                self.rate.sleep()
                self.rate.sleep()
                break

            self.rate.sleep()

        self.calcAllHits()
    
        self.saveSwarm()



    def calcAllHits(self):
        time_limit = robot.stamptofloat(rospy.get_rostime())
        t = 0.0
        print('[Status]: calculating hits')
        while t <= time_limit:
            # process al robots for this time
            for r1 in self.swarm:
                for r2 in self.swarm:
                    if r1.id != r2.id:
                        ed, ori, rs1 = robot.r_distance(r1, r2, t)
                        if ed and ed <= self._robot_perception_distance:
                            rs1.addCloseRobot(robot.CloseRobot(r2.id, ed, ori)) 

            # increment time
            t += 0.1

    def saveSwarm(self):
        print('[Status]: Saving Swarm Pickle')
        
        pack_path = rospkg.RosPack().get_path('cimap')
        dir_path = '{}/log/simulation/{}/{}/{}/'.format(pack_path,self._map_name, self._num_robots, self._experiment_id)

        if not os.path.exists(dir_path):
            os.makedirs(dir_path)
        
        # dump data into plicke object
        output = open(dir_path+'swarm.pkl', 'wb')
        [r.prepareToPickle() for r in self.swarm]
        pickle.dump(self.swarm, output)
        output.close()


if __name__ == '__main__':
    try:
        solver = Solver()
        solver.run()
        print "[Status]: Exiting node cimap_explorer."
    except rospy.ROSInterruptException:
        pass





