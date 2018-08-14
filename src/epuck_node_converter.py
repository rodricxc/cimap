#!/usr/bin/env python

import rospy
import math
import errno
import os
import numpy as np
import copy
import collections
import rospkg
from scipy.interpolate import interp1d
import matplotlib.pyplot as plt
import tf

from ar_track_alvar_msgs.msg import AlvarMarkers
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range, LaserScan


class Solver(object):

    def __init__(self, show_output=False):
        super(Solver, self).__init__()

    def init_node(self):
        rospy.init_node('epuck_converter', anonymous=False)
        # self._num_robots = rospy.get_param("/num_robots")
        self._in_ids = [0, 1, 2, 3, 4, 5]

        self.swarm_converters = [RobotConverter(id_in, id_out) for id_out, id_in in enumerate(self._in_ids)]
        [r.init_comm() for r in self.swarm_converters]

        self.converter = {id_in: self.swarm_converters[id_out] for id_out, id_in in enumerate(self._in_ids)}

        self.sub_ar_track = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.republishGt)

    def republishGt(self, data):
        
        for m in data.markers:
            if m.id in self.converter:
                self.converter[m.id].setGt(m.pose)
        
    def run(self):
        self.init_node()
        while not rospy.is_shutdown():
            rospy.sleep(0.1)





class RobotConverter(object):

    def __init__(self, id_in, id_out, ):
        super(RobotConverter, self).__init__()
        self._id_in = id_in
        self._id_out = id_out
        self.topic_in  = 'robot_real_%d'%id_in
        self.topic_out = 'robot_%d'%id_out
        self.topic_ar = 'robot_%d'%id_out

        self.ranges = np.full((8,), 0.085)
        self._ranges_base = np.full((8,), 0.001)    


    def init_comm(self):


        # proximity
        self.sub_proximity = [rospy.Subscriber("/" + self.topic_in + "/proximity%d"%i, Range, self.updateProximity(i)) for i in range(8)] 
        self.pub_laser = rospy.Publisher("/" + self.topic_out + "/base_scan", LaserScan, queue_size=3)
        self.timer = rospy.Timer(rospy.Duration(0.1), self.publishRanges)

        # odom
        self.pub_odom = rospy.Publisher("/" + self.topic_out + "/odom", Odometry, queue_size=3)
        self.sub_odom = rospy.Subscriber("/" + self.topic_in + "/odom", Odometry, self.publishOdom)

        # groud trhuth
        self.pub_gt = rospy.Publisher("/" + self.topic_out + "/base_pose_ground_truth", Odometry, queue_size=3)

        # publisher of velocity commands
        self.pub_cmd = rospy.Publisher("/" + self.topic_in + "/mobile_base/cmd_vel", Twist, queue_size=3)
        self.sub_cmd = rospy.Subscriber("/" + self.topic_out + "/cmd_vel", Twist, self.repassCmd)
    
    def setGt(self, data):
        odom = Odometry()

        odom.header = data.header
        odom.header.frame_id='odom'
        odom.pose.pose = data.pose
        
        q = [odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w]

        q_rot = tf.transformations.quaternion_from_euler(np.pi, 0, 0)
        q_new = tf.transformations.quaternion_multiply(q_rot, q)
        x, y, z, w = q_new
        odom.pose.pose.orientation.x = x
        odom.pose.pose.orientation.y = y
        odom.pose.pose.orientation.z = z
        odom.pose.pose.orientation.w = w

        odom.pose.pose.position.z = 0
        # odom.pose.pose.position.y *=-1
        self.pub_gt.publish(odom)


    def publishOdom(self, data):
        # data.header.frame_id = 'odom',i
        self.pub_odom.publish(data)
        

    def publishRanges(self, event):
        lr = LaserScan()
        
        len_laser = 24

        lr.header.stamp = rospy.get_rostime()
        lr.header.frame_id = 'world'
        lr.angle_min = -np.pi
        lr.angle_max = np.pi 
        lr.angle_increment = 2*np.pi/(len_laser)
        lr.range_min = 0
        lr.range_max = 0.12

        lr.ranges = self.interpolate_epuck(self.ranges, len_laser)
        lr.intensities = np.full((8,), 1)

        # self.plotscan(lr.ranges)

        # print "minimo laser: %.3f" % np.min(lr.ranges)

        self.pub_laser.publish(lr)


    def interpolate_epuck(self, a, len_laser):

        pi = np.pi  
        y_train = np.array([     a[2],     a[3],     a[4],   a[5],   a[6],    a[7],     a[0],    a[1],    a[2],      a[3],      a[4],      a[5]])*1.8 - 0.01666  +0.035
        x_train = np.array([ 3*pi/2.0, 7*pi/6.0, 5*pi/6.0, pi/2.0, pi/4.0, pi/12.0, -pi/12.0, -pi/4.0, -pi/2.0, -5*pi/6.0, -7*pi/6.0, -3*pi/2.0])
        f = interp1d(x_train, y_train, kind='linear')
        x = np.linspace(-np.pi, np.pi, len_laser+1)
        return f(x)[:len_laser]

    def updateProximity(self, i):
        def _update(data):
            # if self._ranges_base[i] < data.range:
            #      self._ranges_base[i] = min(data.range, 0.05)
            # self.ranges[i] = (data.range/self._ranges_base[i])*0.05

            self.ranges[i] = data.range
        return _update

    def repassCmd(self, data):
        self.pub_cmd.publish(data)

    def plotscan(self, data):
        plt.figure(num=self.topic_out, figsize=(12,12))
        plt.clf()
        data = np.append(data, [data[0]]) 
        x = np.linspace(-np.pi, np.pi, len(data))
        plt.plot(np.cos(x)*data,np.sin(x)*data, '-')
        plt.axis([-0.1, 0.1,-0.1, 0.1])
        plt.show(block=False)
        plt.pause(0.01)
        



if __name__ == '__main__':
    try:
        solver = Solver()
        solver.run()
        print "[cimap_processer]: Exiting node cimap_processer."
    except rospy.ROSInterruptException:
        pass





