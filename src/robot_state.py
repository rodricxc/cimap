#!/usr/bin/env python

import numpy as np
# import copy
import collections


class State(object):
    """ Base class for state representation"""

    def __init__(self, mean=None, cov=None):
        super(State, self).__init__()
        self._mean = np.asarray(
            mean).reshape(-1) if mean is not None else np.zeros(1)
        self._variance = np.array(cov) if cov is not None else np.eye(len(self._mean))*0.0001
        self._dimensions = self._mean.size

    def get(self):
        return (self.getMean(), self.getCov())

    def getDimensions(self):
        return self._dimensions

    def getMean(self):
        return self._mean

    @property
    def mean(self):
        return self.getMean()

    def getCov(self):
        return self._variance

    @property
    def cov(self):
        return self.getCov()

    def __str__(self):
        return "N({}, {})".format(str(np.round(self._mean, decimals=2)), str(np.round(np.diag(self._variance), decimals=2)))

    def __repr__(self):
        return self.__str__()

    def add(self, b):
        return State(self.getMean()+b.getMean(), self.getCov()+b.getCov())

    def __add__(self, b):
        return self.add(b)

    def mul(self, b):

        C = np.linalg.inv(self.getCov()+b.getCov())
        new_mean = b.getCov().dot(C).dot(self.getMean()) + \
            self.getCov().dot(C).dot(b.getMean())

        if (self.getDimensions() == 1) and (abs(self.getCov())+abs(b.getCov())) <= 0:
            new_variance = 0
        else:
            new_variance = self.getCov().dot(C).dot(b.getCov())
        return State(new_mean, new_variance)

    def __mul__(self, b):
        return self.mul(b)

    # def _update(self, measurement=None, c=None):

    #     if c is None:
    #         c = np.eye(self._dimensions)
    
    #     if measurement is None:
    #         return copy.copy(self)
        
    #     if isinstance(measurement, collections.Iterable):
    #         k = copy.copy(self)
    #         for st in measurement:
    #             k = k._update(st, c)
    #         return k
        
    #     # calculate gain
    #     _inv_sum_cov_mat = np.linalg.pinv(c.dot(self.cov).dot(c.T)+measurement.cov)
    #     gain_K = self.cov.dot(c.T).dot(_inv_sum_cov_mat)
        
    #     # update
    #     _mean = self.mean + gain_K.dot(measurement.mean - c.dot(self.mean))
    #     _variance = (np.eye(self.getDimensions()) - gain_K.dot(c)).dot(self.cov)
        
    #     return State(_mean, _variance)




class RobotState(object):
    def __init__(self, timestamp,  previous=None):
        super(RobotState, self).__init__()

        self._setTime(timestamp)
        self._state_gt = None
        self._state_noised = None
        self._state_filtered = None

        self._on_hit = None
        self._hit_direction = None
        self._hit_distance = None
        self._laser_raw = None
        self.setPoseFinished(False)
        self.setLaserFinished(False)
        self.recalcLater = False

        self._close_robots = {}

        self._previous = previous

    def __str__(self):
        s = 'RobotState:'
        s += '\n\ttime: %s' % str(self._timestamp)
        s += '\n\ton_hit: %s' % str(self._on_hit)
        s += '\n\thit_direction: %s' % str(self._hit_direction)
        s += '\n\tpose_gt: %s' % str(self._state_gt)
        s += '\n\tpose_noised: %s' % str(self._state_noised)
        s += '\n\tpose_filtered: %s' % str(self._state_filtered)
        return s

    @property
    def previous(self):
        return self._previous

    def setPreviousRobotState(self, value):
        if value is not None and not isinstance(value, RobotState):
            raise AssertionError('Please assign a valid RobotState value type')
        self._previous = value

    # @previous.setter
    # def previous(self, value):
    #     self._previous = value

    @property
    def yaw_gt(self):
        return self._state_gt.getMean()[2]

    @property
    def gt(self):
        return self._state_gt

    @gt.setter
    def gt(self, value):
        if not isinstance(value, (State)):
            raise AssertionError('Timestamp must be a State type')
        self._state_gt = value

    @property
    def noised(self):
        return self._state_noised

    @noised.setter
    def noised(self, value):
        self._state_noised = value

    @property
    def odom(self):
        return self.noised

    @odom.setter
    def odom(self, value):
        self.noised = value

    @property
    def filtered(self):
        return self._state_filtered

    @filtered.setter
    def filtered(self, value):
        self._state_filtered = value

    @property
    def time(self):
        return self._timestamp

    def _setTime(self, timestamp):
        if not isinstance(timestamp, (int, float)):
            raise AssertionError('Timestamp must be an int or float')
        self._timestamp = timestamp

    @property
    def laser(self):
        return self._laser_raw

    @laser.setter
    def laser(self, value):
        self._laser_raw = value

    @property
    def on_hit(self):
        return self._on_hit

    @on_hit.setter
    def on_hit(self, value):
        self._on_hit = value

    @property
    def hit_direction(self):
        return self._hit_direction

    @hit_direction.setter
    def hit_direction(self, value):
        self._hit_direction = value

    @property
    def hit_distance(self):
        return self._hit_distance

    @hit_distance.setter
    def hit_distance(self, value):
        self._hit_distance = value

    def isPoseFinished(self):
        return self._is_pose_finished

    def setPoseFinished(self, value):
        self._is_pose_finished = value

    def isLaserFinished(self):
        return self._is_laser_finished

    def setLaserFinished(self, value):
        self._is_laser_finished = value

    @property
    def close_robots(self):
        return self._close_robots

    def addCloseRobot(self, close_robot):
        if close_robot.id in self._close_robots:
            return False
        
        if not isinstance(close_robot, (CloseRobot)):
            raise AssertionError('close_robot must be a CloseRobot object type')

        self._close_robots[close_robot.id]  =  close_robot.id 
        return True


class CloseRobot(object):
    def __init__(self, id, distance, direction):
        super(CloseRobot, self).__init__()
        self._id = id
        self._distance = distance
        self._direction = direction

    @property
    def id(self):
        return self._id

    @property
    def distance(self):
        return self._distance

    @property
    def direction(self):
        return self._direction

