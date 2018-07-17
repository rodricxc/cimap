#!/usr/bin/env python

import rospy
import math
import errno
import os
import numpy as np
import copy
import collections

import robot


class Solver(object):
    """docstring for Solver"""

    def __init__(self, show_output=False):
        super(Solver, self).__init__()

    def init_node(self):
        rospy.init_node('cimap_explorer', anonymous=False)
        self._num_robots = rospy.get_param("/num_robots")

        self.swarm = [robot.Robot(i) for i in range(self._num_robots)]






    def run(self):
        self.init_node()

        self.rate = rospy.Rate(10)  # 10hz default
        self.rate.sleep()

        # store start time
        starttime = rospy.get_rostime()
        self.start_at = starttime
        self.stoptime = 600

        # start robots
        [r.start() for r in self.swarm]

        while not rospy.is_shutdown():
            now = rospy.get_rostime()
            # stop after k seconds
            if now.secs >= self.stoptime:
                [r.stop() for r in self.swarm]
                self.rate.sleep()
                break

            self.rate.sleep()



if __name__ == '__main__':
    try:
        solver = Solver()
        solver.run()
        print "[Status]: Exiting node swarm_map."
    except rospy.ROSInterruptException:
        pass





