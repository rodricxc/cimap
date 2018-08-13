#!/usr/bin/env python

import rospy
import tf
import math
import random
import numpy as np
import threading 
import copy
import collections
import scipy.interpolate as interpolate
from scipy import optimize

# local libs
from robot_state import RobotState, State, CloseRobot
from models import sample_motion_model_odometry as m_odom, motion_model_odometry_to_control as odom_to_control

# ros message packs
from geometry_msgs.msg import Twist, Quaternion
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovariance, PoseWithCovarianceStamped, PoseArray
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan


# defining/renaming  some matematical functions
cos, sin = np.cos, np.sin
inv = np.linalg.inv
det = np.linalg.det


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
        self.has_gps = False
        self._x_pred = np.zeros(3)
        self._P_pred = np.diag([1,1,1])
        self._neightbour_update_list = collections.defaultdict(lambda: -999)

        self._processed = False

        # constants
        self._max_vel = 1.5
        self._max_vel_ang = np.pi/2.0
        self.started = False
        self.min_hit_direction = 0.8
        self.follow_wall_maxtime = 5


        self.last_pose = None

        print "[Status]: Starting robot named: ", self.nome

    @property
    def id(self):
        return self.numero

    def isProcessed(self):
        return self._processed

    def setProcessed(self, value):
        self._processed = value

    def hasGps(self):
        return self.has_gps
        # return False

    def useGps(self, value):
        self.has_gps = value

    def prepareToPickle(self):
        self._neightbour_update_list = None
        for t, s in self._states.iteritems():
            s.setPreviousRobotState(None)
        self.state_lock = None
        

    def onPickleLoad(self):
        self._neightbour_update_list = collections.defaultdict(lambda: -999)
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
            if timestamp > self._lastState.time:
                self._lastState = self._states[timestamp]
        self.state_lock.release()
        return self._states[timestamp]

    def getLastPose(self):
        s = self.getState()
        c=5
        while s.gt is None and c > 0:
            s= s.previous
            c-=1
            if c <=0:
                return [0,0,0]
        return s.gt.mean

    def initRosCom(self):
        self.unregistered = False
        self.pub_cmd_vel = rospy.Publisher('/' + self.nome + '/cmd_vel', Twist, queue_size=10)
        self.sub_base_pose_ground_truth = rospy.Subscriber("/" + self.nome + "/base_pose_ground_truth", Odometry, self.updateBasePoseNonSync)
        self.sub_base_scan = rospy.Subscriber("/" + self.nome + "/base_scan", LaserScan, self.updateLaser)

    def stopRosComunicacao(self):
        self.unregistered = True
        #self.sub_odom.unregister()

        self.sub_base_scan.unregister()
        self.sub_base_pose_ground_truth.unregister()
        # self.pub_cmd_vel.unregister()

    def initPosePublishers(self):
        self.pub_pose_gt = rospy.Publisher('/' + self.nome + '/pose_gt_with_cov', PoseWithCovarianceStamped, queue_size=1)
        self.pub_pose_odom = rospy.Publisher('/' + self.nome + '/pose_noised_with_cov', PoseWithCovarianceStamped, queue_size=1)
        self.pub_pose_filtered = rospy.Publisher('/' + self.nome + '/pose_filtered_with_cov', PoseWithCovarianceStamped, queue_size=1)
        self.pose_publishing = True
        

    def stopPosePublishers(self):
        self.pose_publishing = False
        self.pub_pose_gt.unregister()
        self.pub_pose_gt = None
        self.pub_pose_odom.unregister()
        self.pub_pose_odom = None
        self.pub_pose_filtered.unregister()
        self.pub_pose_filtered = None

    def publishPoses(self, t):
        state = self.getState(t,insert=False)
        if state and self.pose_publishing:
            self.publishPoseWithCov(self.pub_pose_gt, state.gt, t)
            self.publishPoseWithCov(self.pub_pose_odom, state.odom, t)
            self.publishPoseWithCov(self.pub_pose_filtered, state.filtered, t)
            

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
        print 'stoping robot %d' % self.id
        self.started = True
        self.sendVelocities(vel=[0, 0, 0])
        self.sendVelocities(vel=[0, 0, 0])
        self.sendVelocities(vel=[0, 0, 0])
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
        if self.unregistered:
            return

        current_state = self.getState(data.header.stamp)
        current_state.laser = np.array(data.ranges, dtype=np.float)
        current_state.on_hit = self.verifyHits(current_state)

        # trigger finished
        current_state.setLaserFinished(True)

        if self.last_pose is not None:
            self.last_pose.header.stamp = data.header.stamp
            self.updateBasePose(self.last_pose)

        self.runOnCalbackFinished(current_state)


    #### POSE
    def updateBasePoseNonSync(self, data):
        self.last_pose = data
    
    def updateBasePose(self, data):
        if self.unregistered:
            return

        current_state = self.getState(data.header.stamp)

        if self.id==0:
            print current_state.time

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
        seq = self.state_time_sequence()
        i=1
        while not self.getState(seq[i], insert=False).gt:
            i+=1

        while i>0:
            s = self.getState(seq[i])
            s.previous.gt = s.gt
            i+=-1

        s1 = self.getState(seq[1])
        s0 = s1.previous
        s0.gt = s1.gt
        s0.odom = s1.gt
        s0.filtered = s1.gt
        
        for t in seq:
            rs1 = self.getState(t)
            rs0 = rs1.previous
            if rs1.gt is None:
                rs1.gt = rs0.gt
            rs1.odom = State(m_odom([rs0.gt.mean, rs1.gt.mean], rs0.odom.mean, odom_parameters))
            rs1.filtered = rs1.odom
        

    # UPDATE FUNCTIONS
    def update(self, t, swarm, update_type='ci', perception_noise=None):

        current_state = self.getState(t, insert=False)
        if not current_state:
            return False

        x = self._x_pred
        P = self._P_pred

        # error expected on perception
        # R_ij = R_ji = np.diag([0.05, 0.05])
        if not perception_noise:
            R_ij = R_ji = np.diag([0.05, 0.05])
        else:
            R_ij = R_ji = np.diag(perception_noise)
            
        C = np.eye(3)

        # in case of using gps, update with kalman filter
        if self.hasGps():
            GPS_NOISE = np.diag([0.5, 0.5, 0.3])

            x_gps = current_state.gt.mean +  np.random.multivariate_normal((0,0,0), GPS_NOISE)
            P_gps = GPS_NOISE            
            
            # normalize angles into smalest distance possible 
            x[2] = wrap_pi(x[2])
            x[2], x_gps[2] = np.unwrap([x[2], x_gps[2]])

            # Gain matrix
            K = P.dot(C.T).dot(inv(C.dot(P).dot(C.T)+P_gps))

            # update values and probabilities with gain
            x = x + K.dot(x_gps - C.dot(x))
            x[2] = wrap_pi(x[2])
            P = (np.eye(3) - K.dot(C)).dot(P)

        else:
            for cr_id, close_i_robot in current_state.close_robots.iteritems():

                x_cr, P_cr = swarm[cr_id].getPredicted()
                close_j_robot = swarm[cr_id].getState(t,insert=False).close_robots[self.id]

                r_ji = np.random.multivariate_normal([close_j_robot.distance, close_j_robot.direction], R_ji)
                r_ij = np.random.multivariate_normal([close_i_robot.distance, close_i_robot.direction], R_ij)

                # pose that the other robot imagines that this one have
                x_meas, P_meas = self.calcRelativeState(r_ji, R_ji,  # perceptions from this robot 
                                                        r_ij, R_ij,  # perception from the other robot
                                                        x_cr, P_cr)  # Pose estimation of the other robot given by the other robot

                # make sure that the angles are in the closest form possible
                x[2] = wrap_pi(x[2])
                x[2], x_meas[2] = np.unwrap([x[2], x_meas[2]])

                ## exchange with GPS robots
                if swarm[cr_id].hasGps():
                    # Gain matrix
                    K = P.dot(C.T).dot(inv(C.dot(P).dot(C.T)+P_meas))
                    # update values and probabilities with gain
                    x = x + K.dot(x_meas - C.dot(x))
                    x[2] = wrap_pi(x[2])
                    P = (np.eye(3) - K.dot(C)).dot(P)
                ## exchange with Non-GPS robot
                else:
                    # CI update
                    if update_type=='ci':
                        # w = optimize.fminbound(lambda w : det(inv(inv(P) * w + inv(P_meas) * (1 - w))), 0, 1)
                        w = 0.5

                        P_ci = inv(inv(P) * w + inv(P_meas) * (1 - w))
                        x_ci = P_ci.dot(w * inv(P).dot(x) + (1 - w) * inv(P_meas).dot(x_meas))

                        x_ci[2] = wrap_pi(x_ci[2])

                        P = P_ci
                        x = x_ci

                    # Kalman update non recursive
                    elif update_type=='k':
                        if t - self.lastUpdateFrom(cr_id) > 10:
                            # Gain matrix
                            K = P.dot(C.T).dot(inv(C.dot(P).dot(C.T)+P_meas))
                            # update values and probabilities with gain
                            x = x + K.dot(x_meas - C.dot(x))
                            x[2] = wrap_pi(x[2])
                            P = (np.eye(3) - K.dot(C)).dot(P)

                            self.setUpdateFrom(cr_id, t)                    
            
            

        # save results on state
        x[2] = wrap_pi(x[2])
        current_state.filtered = State(x, P)

    def lastUpdateFrom(self, neighbour_id):        
        return self._neightbour_update_list[neighbour_id]

    def setUpdateFrom(self, neighbour_id, t):
        self._neightbour_update_list[neighbour_id] = t


    def calcRelativeState(self, r_ij, R_ij, r_ji, R_ji, xi, Pi):
        # r_ij  -->  relative estimate measurement (distance and angle) of j from i
        # r_ji  -->  relative estimate measurement (distance and angle) of i from j
        
        # Calc Relative Pose

        m_ij = np.array([r_ij[0] * cos(r_ij[1] + xi[2]),
                        r_ij[0] * sin(r_ij[1] + xi[2]),
                        math.pi + r_ij[1] - r_ji[1]])

        xi[2] = wrap_pi(xi[2])
        xi[2], m_ij[2] = np.unwrap([xi[2], m_ij[2]])
        
        x_ji = xi + m_ij
        x_ji[2] = wrap_pi(x_ji[2])


        # calc relative uncertain
        Fij = np.array([[ 1, 0, -r_ij[0] * sin(xi[2] + r_ij[1])],
                        [ 0, 1,  r_ij[0] * cos(xi[2] + r_ij[1])],
                        [ 0, 0,  1                             ]])

        Sij = np.array([[ cos(xi[2]+r_ij[1]), -sin(xi[2]+r_ij[1]),  0],
                        [ sin(xi[2]+r_ij[1]),  cos(xi[2]+r_ij[1]),  0],
                        [                  0,                   1, -1]])
        
        _R_ij = np.diag([R_ij[0,0], R_ij[1,1], R_ji[1,1]])
        
        Pji = Fij.dot(Pi).dot(Fij.T) + Sij.dot(_R_ij).dot(Sij.T)

        return x_ji, Pji



    # PREDICTION FUNCTIONS
    def prediction(self, t, error):
        current_state = self.getState(t, insert=False)

        if not current_state:
            return False
        
        # calc variables for transition
        self._last_t_processed = t
        self._u, self._E = odom_to_control([current_state.previous.odom.mean, current_state.odom.mean], error)
        self._phi = self.createPhiMatrix(self._u, current_state.previous.filtered.mean[2])
        self._G = self.createGMatrix(current_state.previous.filtered.mean[2])

        # calc next pose prediction 'x' and covariance 'P'
        #     x1 = x0 + motion_matrix * control_u 
        self._x_pred = current_state.previous.filtered.mean + self._G.dot(self._u)
        #     P1 =  phi * P0 * phi_T    +   motion_matrix * control_Error * motion_matrix_T
        self._P_pred =  self._phi.dot(current_state.previous.filtered.cov).dot(self._phi.T) +  self._G.dot(self._E).dot(self._G.T)

        
        # FOR TEST ONLY
        # current_state.filtered = State(self._x_pred, self._P_pred)
        # if self.id == 1:
        #     print("---------------------------")
        #     print "u:      ", self._u
        #     print "pred 0: ", current_state.previous.filtered.mean
        #     print "pred 1: ", self._x_pred
        #     print "odom 0: ", current_state.previous.odom.mean
        #     print "odom 1: ", current_state.odom.mean
        #     print("---------------------------")
        
    def getPredicted(self):
        return self._x_pred, self._P_pred

    def calcControlAndError(self, t, error):
        current_state = self.getState(t, insert=False)
        if not current_state:
            return False
        self._u, self._E = odom_to_control([current_state.previous.odom.mean, current_state.previous.odom.mean], error)
        self._phi = self.createPhiMatrix(self._u, current_state.previous.odom.mean[2])
        self._G = self.createGMatrix(current_state.previous.odom.mean[2])



    def createMotionMatrix(self, theta):
        return np.array([[cos(theta), 0],
                         [sin(theta), 0],
                         [0         , 1]])

    def createPhiMatrix(self, u, theta, dt=1):
        return np.array([[1, 0, -dt*u[0]*sin(theta)],
                         [0, 1,  dt*u[0]*cos(theta)],
                         [0, 0,  1                ]])

    def createGMatrix(self, theta, dt=1):
        return self.createMotionMatrix(theta)*dt


    # CONTROL
    def runControl(self, robot_state):
        # self.randomWalkOnObstacle(robot_state)
        self.randomWalkOnObstacleDiferencial(robot_state)
        # self.randomWalkOnObstacleDiferencialForward(robot_state)
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

            
            if robot_state.hit_distance < 0.05 :
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

    def randomWalkOnObstacleDiferencialForward(self, robot_state): 
        if robot_state.on_hit: 
            dif_ang_prev = wrap_pi(self.randomDirection - robot_state.yaw_gt)
            if dif_ang_prev > np.pi/2.0:
                dif_ang_prev = wrap_pi(dif_ang_prev-np.pi)


            hit_dif =   wrap_pi(robot_state.hit_direction + robot_state.yaw_gt  - self.randomDirection)

            
            if robot_state.hit_distance < 0.5 :
                self.randomDirection = robot_state.hit_direction + robot_state.yaw_gt + 0.75 * np.pi + random.random() * np.pi*0.5    
                
            elif abs(hit_dif) < np.pi/4.0: 
            # elif (not robot_state.previous.on_hit) and abs(hit_dif) < np.pi/3: 
            # elif (not robot_state.previous.on_hit ): 
                self.randomDirection = robot_state.hit_direction + robot_state.yaw_gt + 0.5 * np.pi + random.random() * np.pi    



        dif_ang = wrap_pi(self.randomDirection - robot_state.yaw_gt)

        self.reverse = 1
        # if abs(dif_ang) > np.pi/2.0:
        #     self.reverse = -1
        #     dif_ang = wrap_pi(dif_ang + np.pi)

        if abs(dif_ang) < np.pi/2.0:
            x = self._max_vel*np.cos(dif_ang)*self.reverse
            w = dif_ang
        else:
            x = 0
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
        if self.willHit(current_state, required=0.09):
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

    def willHit(self, current_state, required=0.08):
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
    return "%.1f" % stamptofloat(stamp)

def stamptofloat(stamp):
    if  isinstance(stamp, (float,int,)):
        return np.round(stamp,decimals=1)
    return np.round(stamp.secs+stamp.nsecs/1000000000.0, decimals=1)

def wrap_pi(a):
    while a <= -np.pi:
        a += 2*np.pi
    while a > np.pi:
        a -= 2*np.pi
    return a 

def oriToEuler(ori):
    euler = tf.transformations.euler_from_quaternion((ori.x, ori.y, ori.z, ori.w))
    return euler[0], euler[1], euler[2]
