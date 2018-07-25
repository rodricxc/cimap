from __future__ import (absolute_import, division,
                        print_function, unicode_literals)

import os
import rospy
import tf
import numpy as np
import math
from scipy import arange
import matplotlib.pyplot as plt
import matplotlib.mlab as mlab
import threading


from nav_msgs.msg import OccupancyGrid


class Mapa(object):
    """docstring for Mapa"""

    def __init__(self, precision, map_name, publish_map=True):
        super(Mapa, self).__init__()
        self.map_name = map_name
        self.precision = precision
        self.invPrecision = 1.0/precision
        self._l0 = self.log_odds(0.5)
        self.lock = threading.RLock()
        self.publish_map = publish_map
        self._initMapa()

    def _initMapa(self):
        self.interval = np.array([[-25, 25], [-25, 25]])
        self.dimensions = np.array([int(self.invPrecision * (self.interval[0, 1] - self.interval[0, 0])),
                                    int(self.invPrecision * (self.interval[1, 1] - self.interval[1, 0]))])
        self.grid = np.full(self.dimensions, 0.5, dtype=np.float)

        if self.publish_map:
            self.initComunication()

    def initComunication(self):
        self.pub_map = rospy.Publisher(self.map_name+'/map', OccupancyGrid, queue_size=10)
        self.map_seq = 0

        self.tf_timer = rospy.Timer(rospy.Duration(0.5), self.publishMapTransform)


    def publishMapTransform(self, data):
        
        br = tf.TransformBroadcaster()

        br.sendTransform((0, 0, 0) ,
                        (0, 0, 0, 1),
                        rospy.Time.now(),
                        self.map_name+'/map_tf',
                        "map_base")

    def publishMap(self):
        if not self.publish_map:
            return 

        self.lock.acquire()
        og = OccupancyGrid()
        self.map_seq += 1
        og.header.seq = self.map_seq
        og.header.stamp = rospy.get_rostime()
        og.header.frame_id = self.map_name+'/map_tf'

        og.info.map_load_time = og.header.stamp
        og.info.resolution = self.precision
        og.info.width,og.info.height = self.grid.shape[:2]

        x, y = self.gridToPos([0,0])[:2]

        og.info.origin.position.x = x
        og.info.origin.position.y = y
        og.info.origin.position.z = 0

        x, y, w, z = tf.transformations.quaternion_from_euler(0,0,math.pi)[:4]

        og.info.origin.orientation.x = x
        og.info.origin.orientation.y = y
        og.info.origin.orientation.z = z
        og.info.origin.orientation.w = w

        og.data = list(np.around(self.grid*100).T.flatten())

        self.lock.release()


        self.pub_map.publish(og)

    def log_odds(self, prob):
        if prob >= 1:
            prob = 0.999
        elif prob <= 0:
            prob = 0.001
        return math.log(prob/(1-prob))

    def np_calc_prob(self, log_odds):
        return 1 - (1/(1+np.exp(log_odds)))

    def np_log_odds(self, p):
        p[p >= 1] = 0.9999
        p[p <= 0] = 0.0001
        return np.log(p/(1-p))

    def expandMapInterval(self, new_interval):

        if (self.interval[0, 0] < new_interval[0, 0] or
            self.interval[0, 1] > new_interval[0, 1] or
            self.interval[1, 0] < new_interval[1, 0] or
                self.interval[1, 1] > new_interval[1, 1]):
            raise ValueError(
                'Error when expanding the interval. New interval is smaller than the current.')

        new_dimensions = np.array([int(self.invPrecision * (new_interval[0, 1] - new_interval[0, 0])),
                                   int(self.invPrecision * (new_interval[1, 1] - new_interval[1, 0]))])
        new_grid = np.full(new_dimensions, 0.5, dtype=np.float)
        new_pose_update = np.array([(self.interval[0, 0] - new_interval[0, 0])*self.invPrecision,
                                    (self.interval[1, 0] - new_interval[1, 0])*self.invPrecision], dtype=np.int)
        new_grid[new_pose_update[0]:(new_pose_update[0]+self.dimensions[0]),
                 new_pose_update[1]:(new_pose_update[1]+self.dimensions[1])] = self.grid

        self.grid = new_grid
        self.interval = new_interval
        self.dimensions = new_dimensions

    def getCopyToShow(self):
        ts = np.copy(self.grid)
        a00 = self.posToGrid([-8, -8])
        a01 = self.posToGrid([-8, 8])
        a10 = self.posToGrid([8, -8])
        a11 = self.posToGrid([8, 8])

        ts[a00[0], a00[1]:a01[1]] = 1
        ts[a00[0]:a10[0], a00[1]] = 1
        ts[a01[0]:a11[0], a01[1]] = 1
        ts[a10[0], a10[1]:a11[1]] = 1
        return ts.T

    def plotMapa(self, path=None, round_at=None):

        s = np.copy(self.grid.T)
        if round_at is not None:
            s[s < round_at[0]] = 0
            s[s > round_at[1]] = 1
            s[(s > 0) & (s < 1)] = 0.5
            
        s[0, 0], s[1, 0] = 1, 0
        figure = plt.figure(figsize=(15, 15))
        plt.matshow(s, cmap=plt.cm.binary, origin='lower', fignum=1)
        plt.grid(False)
        plt.title(self.map_name)
        plt.xticks(np.arange(0, self.dimensions[0], step=self.dimensions[0]//5), np.arange(
            self.interval[0, 0], self.interval[0, 1], step=(self.interval[0, 1]-self.interval[0, 0])//5))
        plt.yticks(np.arange(0, self.dimensions[1], step=self.dimensions[1]//5), np.arange(
            self.interval[1, 0], self.interval[1, 1], step=(self.interval[1, 1]-self.interval[1, 0])//5))
        if path is not None:
            plt.draw()
            plt.savefig(path)
        else:
            plt.show()
        # plt.close(figure)


    def storeMapObject(self):

        tostore = dict()
        tostore['interval'] = (-20, 20, -20, 20)
        tostore['precision'] = self.precision

        p1 =self.posToGrid([tostore['interval'][0],tostore['interval'][2]])
        p2 =self.posToGrid([tostore['interval'][1],tostore['interval'][3]])
        p1 =self.posToGrid([tostore['interval'][0],tostore['interval'][2]])

        tostore['map'] = np.array(self.grid[p1[0]:p2[0],p1[1]:p2[1]], dtype=np.float16)
        return tostore


    def logUpdateRegion(self, pos, m):
        self.lock.acquire()
        
        pos = np.array(pos)
        grid_pos = np.array(self.posToGrid(pos))

        center = (np.array(m.shape)-1)//2
        gp00 = grid_pos - center
        gp11 = gp00 + np.array(m.shape)

        # to reescale map if needed
        gtp0 = self.gridToPos(gp00)
        gtp1 = self.gridToPos(gp11)
        self.posToGrid(gtp0)
        self.posToGrid(gtp1)

        grid_pos = np.array(self.posToGrid(pos))
        gp00 = grid_pos - center
        gp11 = gp00 + np.array(m.shape)

        odd_current = self.np_log_odds(
            self.grid[gp00[0]:gp11[0], gp00[1]:gp11[1]])
        odd_m = self.np_log_odds(m)

        prob_val = self.np_calc_prob(odd_current + odd_m + self._l0)

        self.grid[gp00[0]:gp11[0], gp00[1]:gp11[1]] = prob_val
        self.lock.release()


    def updateRegion(self, pos, m):
        pos = np.array(pos)
        grid_pos = np.array(self.posToGrid(pos))

        center = (np.array(m.shape)-1)//2
        gp00 = grid_pos - center
        gp11 = gp00 + np.array(m.shape)

        # to reescale map if needed
        self.posToGrid(self.gridToPos(gp00))
        self.posToGrid(self.gridToPos(gp11))

        grid_pos = np.array(self.posToGrid(pos))
        gp00 = grid_pos - center
        gp11 = gp00 + np.array(m.shape)

        self.grid[gp00[0]:gp11[0], gp00[1]:gp11[1]] += m

    def isInGrid(self, pos):
        if (pos[0] <= self.interval[0, 0] or
                pos[0] >= self.interval[0, 1] or
                pos[1] <= self.interval[1, 0] or
                pos[1] >= self.interval[1, 1]):
            return False
        return True

    def newGridInterval(self, pos):

        sz = np.copy(self.interval)

        if pos[0] <= sz[0, 0]:
            sz[0, 0] = pos[0] - 0.2*(sz[0, 1]-sz[0, 0])

        if pos[0] >= sz[0, 1]:
            sz[0, 1] = pos[0] + 0.2*(sz[0, 1]-sz[0, 0])

        if pos[1] <= sz[1, 0]:
            sz[1, 0] = pos[1] - 0.2*(sz[1, 1]-sz[1, 0])

        if pos[1] >= sz[1, 1]:
            sz[1, 1] = pos[1] + 0.2*(sz[1, 1]-sz[1, 0])

        return sz

    def gridToPos(self, grid_pos):
        x = self.interval[0, 0] + grid_pos[0]*self.precision
        y = self.interval[1, 0] + grid_pos[1]*self.precision
        return (x, y)

    def posToGrid(self, pos):

        if not self.isInGrid(pos):
            newGrid = self.newGridInterval(pos)
            self.expandMapInterval(newGrid)

        x = int((pos[0] - self.interval[0][0])*self.invPrecision)
        y = int((pos[1] - self.interval[1][0])*self.invPrecision)
        x = min(max(0, x), self.dimensions[0]-1)
        y = min(max(0, y), self.dimensions[1]-1)
        return (x, y)

    def probAt(self, pos):
        (x, y) = self.posToGrid(pos)
        return self.grid[x, y]

    def updateAt(self, pos, val):
        (x, y) = self.posToGrid(pos)
        self.grid[x, y] = val

    def updateMapa(self, config=(0, 0, 0), laser=[0]):
        pass
