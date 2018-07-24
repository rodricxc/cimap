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
from robot_state import RobotState, State, CloseRobot
from models import sample_motion_model_odometry as m_odom, motion_model_odometry_to_control as odom_to_control

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
        self.follow_wall = False
        self.follow_wall_begin = None
        self.on_scape = False
        self.pose_publishing = False

        # constants
        self._max_vel = 0.9
        self._max_vel_ang = np.pi/2.0
        self.started = False
        self.min_hit_direction = 0.8
        self.follow_wall_maxtime = 5

        print "[Status]: Starting robot named: ", self.nome

    @property
    def id(self):
        return self.numero

    def prepareToPickle(self):
        for t, s in self._states.iteritems():
            s.setPreviousRobotState(None)
        self.state_lock = None
        

    def onPickleLoad(self):
        self.state_lock = threading.RLock()
        sorted_idx = [0]+sorted(self._states)
        for i in range(1,len(sorted_idx)):
            self.getState(sorted_idx[i]).setPreviousRobotState(self.getState(sorted_idx[i-1])) 

    def initStates(self):
        self.state_lock = threading.RLock()
        self._states = {0: RobotState(timestamp=0)}
        self._lastState = self._states[0]
        self._lastState.gt = State([0, 0, 0])
        self._lastState.noised = State([0, 0, 0])
        self._lastState.filtered = State([0, 0, 0])

    def getState(self, timestamp=None, insert=True):
        self.state_lock.acquire()

        # case of external call, for non modification
        if timestamp is None:
            c = copy.copy(self._lastState)
            self.state_lock.release()
            return c

        timestamp = stamptofloat(timestamp)
        

        if timestamp not in self._states:
            if not insert:
                self.state_lock.release()
                return None
                
            self._states[timestamp] = RobotState(timestamp=timestamp)
            self._states[timestamp].setPreviousRobotState(self._lastState)
            # if timestamp > self._lastState.time:
            #     self._lastState = self._states[timestamp]
        self.state_lock.release()
        return self._states[timestamp]

    def initRosCom(self):
        self.unregistered = False
        self.pub_cmd_vel = rospy.Publisher('/' + self.nome + '/cmd_vel', Twist, queue_size=10)
        self.sub_base_pose_ground_truth = rospy.Subscriber("/" + self.nome + "/base_pose_ground_truth", Odometry, self.updateBasePose)
        self.sub_base_scan = rospy.Subscriber("/" + self.nome + "/base_scan", LaserScan, self.updateLaser)

    def stopRosComunicacao(self):
        self.unregistered = True
        #self.sub_odom.unregister()

        self.sub_base_scan.unregister()
        self.sub_base_pose_ground_truth.unregister()
        self.pub_cmd_vel.unregister()

    def initPosePublishers(self):
        self.pub_pose_gt = rospy.Publisher('/' + self.nome + '/pose_gt_with_cov', PoseWithCovarianceStamped, queue_size=1)
        self.pub_pose_odom = rospy.Publisher('/' + self.nome + '/pose_noised_with_cov', PoseWithCovarianceStamped, queue_size=1)
        self.pub_pose_filtered = rospy.Publisher('/' + self.nome + '/pose_filtered_with_cov', PoseWithCovarianceStamped, queue_size=1)
        self.pose_publishing = True
        

    def stopPosePublishers(self):
        self.pose_publishing = False
        self.pub_pose_gt.unregister()
        self.pub_pose_odom.unregister()
        self.pub_pose_filtered.unregister()

    def publishPoses(self, t):
        state = self.getState(t,insert=False)
        if state and self.pose_publishing:
            self.publishPoseWithCov(self.pub_pose_gt, state.gt, t)
            self.publishPoseWithCov(self.pub_pose_odom, state.odom, t)

    def publishPoseWithCov(self, publisher, state, t):
        if not state:
            return
        # header (no seq)
        pwc = PoseWithCovarianceStamped()
        # pwc.header.stamp = t
        pwc.header.frame_id = 'odom'

        # pose cov
        pwc.pose.covariance = self.covConverter(state.cov)
        pwc.pose.pose = self.poseConverter(state.mean)

        # publish
        publisher.publish(pwc)

    def covConverter(self, cov):
        cov = np.insert(cov, (2, 2, 2), 0, axis=0)
        cov = np.insert(cov, (2, 2, 2), 0, axis=1)
        return list(cov.flatten())

    def poseConverter(self, p):
        pose = Pose()

        x, y, w = p[:3]
        pose.position.x = x
        pose.position.y = y
        pose.position.z = 0

        x, y, z, w = tf.transformations.quaternion_from_euler(0, 0, w)[:4]
        pose.orientation.x = x
        pose.orientation.y = y
        pose.orientation.z = z
        pose.orientation.w = w

        return pose


    def start(self):
        self.started = True
   
    def stop(self):
        self.started = False
        self.sendVelocities()
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
        if self.unregistered:
            return

        current_state = self.getState(data.header.stamp)
        current_state.laser = np.array(data.ranges, dtype=np.float)
        current_state.on_hit = self.verifyHits(current_state)

        # trigger finished
        current_state.setLaserFinished(True)
        self.runOnCalbackFinished(current_state)


    #### POSE
    def updateBasePose(self, data):
        if self.unregistered:
            return

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

    # OTHER
    def calcOdometry(self, odom_parameters):
        s1 = self.getState(0)
        s1.odom = s1.gt
        s1.filtered = s1.gt
        for t in self.state_time_sequence():
            rs1 = self.getState(t)
            rs0 = rs1.previous
            rs1.odom = State(m_odom([rs0.gt.mean, rs1.gt.mean], rs0.odom.mean, odom_parameters))

    
    def calcControlAndError(self, t, error):
        current_state = self.getState(t, insert=False)
        self._u, self._E = odom_to_control([current_state.previous.odom.mean, current_state.previous.odom.mean], error)
        
    def calcTransitionMatrix(self, t):
        

    def calcPrediction(self, t):
        pass



    # CONTROL
    def runControl(self, robot_state):
        # self.randomWalkOnObstacle(robot_state)
        self.randomWalkOnObstacleDiferencial(robot_state)
        # self.randomWalkFollowDiferencial(robot_state)
        

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


            hit_dif =   wrap_pi(robot_state.hit_direction + robot_state.yaw_gt  - self.randomDirection)

            
            if robot_state.hit_distance < 0.5 :
                self.randomDirection = robot_state.hit_direction + robot_state.yaw_gt + 0.75 * np.pi + random.random() * np.pi*0.5    
                
            elif abs(hit_dif) < np.pi/3.0: 
            # elif (not robot_state.previous.on_hit) and abs(hit_dif) < np.pi/3: 
            # elif (not robot_state.previous.on_hit ): 
                self.randomDirection = robot_state.hit_direction + robot_state.yaw_gt + 0.5 * np.pi + random.random() * np.pi    



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

    def randomWalkFollowDiferencial(self, robot_state): 
        # control state
        if robot_state.on_hit: 
            # start state if not started
            if not self.follow_wall:
                self.follow_wall_begin = robot_state.time
                self.follow_wall = True
            
            if self.follow_wall:
                if self.numero == 0:
                    print("following wall", robot_state.time)
                # leave wall after time
                if robot_state.time - self.follow_wall_begin > self.follow_wall_maxtime:
                    self.randomDirection = robot_state.hit_direction + robot_state.yaw_gt + 0.5 * np.pi + random.random() * np.pi    

                else:
                    hit_dir =   wrap_pi(robot_state.hit_direction + robot_state.yaw_gt)
                    walk_dir =  wrap_pi(hit_dir+np.pi/2)
                    walk_dif =  wrap_pi(walk_dir - self.randomDirection)

                    if abs(walk_dif) > np.pi/2:
                        self.randomDirection = wrap_pi(walk_dir+np.pi)
                    else:
                        self.randomDirection = walk_dir

            
            if robot_state.hit_distance < 0.4 or (self.on_scape and robot_state.hit_distance < 0.5):
                self.on_scape=True
                self.randomDirection = robot_state.hit_direction + robot_state.yaw_gt + 0.75 * np.pi + random.random() * np.pi*0.0    
            else:
                self.on_scape=False                
        
        # easy hit
        elif not (np.any(robot_state.laser < 0.8)):
            self.follow_wall = False



        # control direction
        dif_ang = wrap_pi(self.randomDirection - robot_state.yaw_gt)
        
        if abs(dif_ang) > np.pi/2.0:
            self.reverse = -1
            dif_ang = wrap_pi(dif_ang + np.pi)
        else: 
            self.reverse = 1 

        x = self._max_vel*(np.cos(dif_ang)**2)*self.reverse
        if dif_ang > np.pi/3:
            x = 0
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


    def state_time_sequence(self):
        return sorted(self._states.keys())







# auxiliary
def r_distance(r1, r2, time):
    rs1 = r1.getState(time, insert=False)
    rs2 = r2.getState(time, insert=False)

    if not rs1 or not rs1.gt or not rs2 or not rs2.gt:
        return None, None, None

    ed =  np.linalg.norm(rs1.gt.mean[:2]-rs2.gt.mean[:2])
    glob_ori =  np.arctan2(rs2.gt.mean[1]-rs1.gt.mean[1], rs2.gt.mean[0]-rs1.gt.mean[0])
    relative_ori = wrap_pi(glob_ori - rs1.yaw_gt)

    return ed, relative_ori, rs1


def stamptostr(stamp):
    return "%.2f" % stamptofloat(stamp)

def stamptofloat(stamp):
    if  isinstance(stamp, (float,int,)):
        return np.round(stamp,decimals=2)
    return np.round(stamp.secs+stamp.nsecs/1000000000.0, decimals=2)

def wrap_pi(a):
    while a <= -np.pi:
        a += 2*np.pi
    while a > np.pi:
        a -= 2*np.pi
    return a 

def oriToEuler(ori):
    euler = tf.transformations.euler_from_quaternion((ori.x, ori.y, ori.z, ori.w))
    return euler[0], euler[1], euler[2]