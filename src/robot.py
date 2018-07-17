#!/usr/bin/env python

import rospy
import tf
import math
import random
import numpy as np
import threading 
import copy
import scipy.interpolate as interpolate

# local libs
from robot_state import RobotState, State

# ros message packs
from geometry_msgs.msg import Twist, Quaternion
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovariance, PoseWithCovarianceStamped, PoseArray
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan



class Robot(object):
    """docstring for Robot"""

    def __init__(self, numero=''):
        super(Robot, self).__init__()
        self.numero = numero
        if numero != "":
            self.nome = "robot_" + str(numero)

        self.initStates()
        self.initRosCom()
        
        # state variables
        self.randomDirection = random.random()*np.pi*2
        self.reverse = 1

        # constants
        self._max_vel = 0.6
        self._max_vel_ang = np.pi/2.0
        self.started = False
        self.min_hit_direction = 0.8

        print "[Status]: Starting robot named: ", self.nome

    def initStates(self):
        self.state_lock = threading.RLock()
        self._states = {0: RobotState(timestamp=0)}
        self._lastState = self._states[0]
        self._lastState.gt = State([0, 0, 0])
        self._lastState.noised = State([0, 0, 0])
        self._lastState.filtered = State([0, 0, 0])

    def getState(self, timestamp=None):
        self.state_lock.acquire()

        # case of external call, for non modification
        if timestamp is None:
            c = copy.copy(self._lastState)
            self.state_lock.release()
            return c

        timestamp = self.stamptofloat(timestamp)
        
        if timestamp not in self._states:
            self._states[timestamp] = RobotState(timestamp=timestamp)
            self._states[timestamp].setPreviousRobotState(self._lastState)
            # if timestamp > self._lastState.time:
            #     self._lastState = self._states[timestamp]
        self.state_lock.release()
        return self._states[timestamp]

    def initRosCom(self):
        self.pub_cmd_vel = rospy.Publisher('/' + self.nome + '/cmd_vel', Twist, queue_size=10)
        self.sub_base_pose_ground_truth = rospy.Subscriber("/" + self.nome + "/base_pose_ground_truth", Odometry, self.updateBasePose)
        self.sub_base_scan = rospy.Subscriber("/" + self.nome + "/base_scan", LaserScan, self.updateLaser)

    def stopRosComunicacao(self):
        self.unregistered = True
        #self.sub_odom.unregister()
        self.sub_base_pose_ground_truth.unregister()
        self.sub_base_scan.unregister()
        self.pub_cmd_vel.unregister()

    def start(self):
        self.started = True

    
    def stop(self):
        self.started = False
        self.stopRosComunicacao()

    # Function to publish velocity
    def sendVelocities(self, vel=(0, 0, 0)):
        if self.started:
            t = Twist()
            t.linear.x = vel[0]
            t.linear.y = vel[1]
            t.angular.z = vel[2]
            self.pub_cmd_vel.publish(t)


    #############################################
    ## Calbacks
    #############################################

    ####  Laser
    def updateLaser(self, data):
        current_state = self.getState(data.header.stamp)
        current_state.laser = np.array(data.ranges, dtype=np.float)
        current_state.on_hit = self.verifyHits(current_state)

        # trigger finished
        current_state.setLaserFinished(True)
        self.runOnCalbackFinished(current_state)


    #### POSE
    def updateBasePose(self, data):
        current_state = self.getState(data.header.stamp)

        # GT update
        _, _, yaw = self.oriToEuler(data.pose.pose.orientation)
        current_state.gt = State([data.pose.pose.position.x, data.pose.pose.position.y, yaw])

        # finished to process poses transitions
        current_state.setPoseFinished(True)
        self.runOnCalbackFinished(current_state)

    def runOnCalbackFinished(self, current_state):

        if not current_state.isLaserFinished() or not current_state.isPoseFinished():
            return




        self.runControl(current_state)


    # CONTROL
    def runControl(self, robot_state):
        # self.randomWalkOnObstacle(robot_state)
        self.randomWalkOnObstacleDiferencial(robot_state)
        

    def randomWalkOnObstacle(self, robot_state): 
        if robot_state.on_hit and (not robot_state.previous.on_hit or robot_state.hit_distance < 0.4):
            self.randomDirection = robot_state.hit_direction + robot_state.yaw_gt + 0.5 * np.pi + random.random() * np.pi
        x = np.cos(self.randomDirection) * self._max_vel
        y = np.sin(self.randomDirection) * self._max_vel
        w = 0
        self.sendVelocities((x, y, w))

    def randomWalkOnObstacleDiferencial(self, robot_state): 
        if robot_state.on_hit: 
            dif_ang_prev = wrap_pi(self.randomDirection - robot_state.yaw_gt)
            if dif_ang_prev > np.pi/2.0:
                dif_ang_prev = wrap_pi(dif_ang_prev-np.pi)

            
            if robot_state.hit_distance < 0.4 :
                self.randomDirection = robot_state.hit_direction + robot_state.yaw_gt + 0.75 * np.pi + random.random() * np.pi*0.5    
                
            # elif (not robot_state.previous.on_hit ) or abs(dif_ang_prev) < 0.5: 
            elif (not robot_state.previous.on_hit and robot_state.previous.previous and not robot_state.previous.previous.on_hit ): 
                self.randomDirection = robot_state.hit_direction + robot_state.yaw_gt + 0.5 * np.pi + random.random() * np.pi    

                if self.numero == 0:
                    print("changed direction")


        dif_ang = wrap_pi(self.randomDirection - robot_state.yaw_gt)
        
        if abs(dif_ang) > np.pi/2.0:
            self.reverse = -1
            dif_ang = wrap_pi(dif_ang + np.pi)
        else: 
            self.reverse = 1

        x = self._max_vel*np.cos(dif_ang)*self.reverse

        w = dif_ang
        y = 0

        self.sendVelocities((x, y, w))












    def verifyHits(self, current_state):
        if self.willHit(current_state, required=0.8):
            self.storeHitPos(current_state)
            hitting = True
        else:
            hitting = False

        return hitting

    def storeHitPos(self, robot_state):
        direction, distance = self.calcHitDirectionDistance(robot_state.laser)
        robot_state.hit_direction = direction
        robot_state.hit_distance = distance

    def calcHitDirectionDistance(self, laser):
        k = min(len(laser)-1, 7)
        a = np.pad(laser[:-1], (k, k+1), 'reflect')
        offset = 2*np.pi/(len(laser)-1)
        xa = np.linspace(-np.pi-k*offset , np.pi+k*offset, len(a))
        x = np.linspace(-np.pi, np.pi, 361)
        f = interpolate.Rbf(xa, a)
        argmin = np.argmin(f(x))
        angmin = x[argmin]
        return angmin, f(angmin)

    def willHit(self, current_state, required=0.667):
        if np.any(current_state.laser <= required) :
            return True
        return False



    # auxiliary
    def oriToEuler(self, ori):
        return oriToEuler(ori)

    def stamptostr(self, stamp):
        return "%.2f" % self.stamptofloat(stamp)

    def stamptofloat(self, stamp):
        if  isinstance(stamp, (float,)):
            return stamp
        return np.round(stamp.secs+stamp.nsecs/1000000000.0, decimals=3)



# auxiliary
def wrap_pi(a):
    while a <= -np.pi:
        a += 2*np.pi
    while a > np.pi:
        a -= 2*np.pi
    return a 

def oriToEuler(ori):
    euler = tf.transformations.euler_from_quaternion((ori.x, ori.y, ori.z, ori.w))
    return euler[0], euler[1], euler[2]