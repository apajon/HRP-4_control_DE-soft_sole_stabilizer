#! /usr/bin/env python
from zmp_com_playback_bridge_stableloop import PlaybackBridge

import roslib; roslib.load_manifest('mc_control')
import rospy

import numpy as np

from eigen3 import Vector3d, Matrix3d, toEigenX, Quaterniond
#import spacevecalg as sva
import rbdyn as rbd
import tasks

from mc_control.msg import Robot


from mc_rbdyn import loadRobots, rbdList, MRContact, robotCopy
from mc_solver import MRQPSolver, DynamicsConstraint, ContactConstraint, \
  KinematicsConstraint
from joint_state_publisher import JointStatePublisher

from mc_robot_msgs.msg import MCRobotState

from ask_user import ask_user

from tasks_helper import orientationTask, positionTrackingTask, toVecX, comTrajectoryTask, positionTask
from stop_experiment_helper import stopMotion, goHalfSitting

from stabilizer_msg_helper import stabilizerMsg
from walk_PG_markers_stableloop import WalkPGMarkers
from markers_zmp_com_stableloop import ZMPCOMMarkers

from gain_interpolator import GainInterpolator

from mc_ros_utils import RobotDisplayer

from itertools import chain

import time

class hrp4W_helper(object):
  def __init__(self, robot):
    self.hrp4W = robotCopy(robot)
    self.hrp4WDisplayer=RobotDisplayer(self.hrp4W,'2')
    
    self.q0Vel=Vector3d().Zero()
#    self.q0Pos=Vector3d(-0.021915977704131524, 0.0, 0.7473573364829554)+Vector3d(0,1.0,0)
    self.q0Pos=Vector3d(0, 1.0, 0.7473573364829554)

    
    rbd.forwardKinematics(self.hrp4W.mb, self.hrp4W.mbc)
    self.hrp4W.mbc.q = robot.mbc.q
    hrp4W_q = rbdList(self.hrp4W.mbc.q)
    hrp4W_q[0] = [1., 0., 0., 0., self.q0Pos[0], self.q0Pos[1], self.q0Pos[2]]

    self.hrp4W.mbc.q = hrp4W_q
     
    self.display_helper()
    
    self.isDebug=False
    
  def display_helper(self):
    rbd.eulerIntegration(self.hrp4W.mb, self.hrp4W.mbc, 0.005)
    rbd.forwardKinematics(self.hrp4W.mb, self.hrp4W.mbc)
    rbd.forwardVelocity(self.hrp4W.mb, self.hrp4W.mbc)
    rbd.forwardAcceleration(self.hrp4W.mb, self.hrp4W.mbc)  
    self.hrp4WDisplayer.display()
    
  def update(self, rs):
    hrp4W_q=rbdList(self.hrp4W.mbc.q)
    for i, name in enumerate(rs.joint_name): 
        if self.hrp4W.hasJoint(name):
            hrp4W_q[self.hrp4W.jointIndexByName(name)]=[rs.joint_position[i]]
#            if name=='L_ANKLE_P':
#                print 'L_ANKLE_P'
#                print hrp4W_q[self.hrp4W.jointIndexByName(name)]
#                print rs.joint_position[i]

#    quat=Quaterniond(rs.imu_orientation.w, rs.imu_orientation.x, rs.imu_orientation.y, rs.imu_orientation.z)
#    quat.normalize()
#    rot=quat.toRotationMatrix()
#    acc=rot*Vector3d(rs.imu_linear_acceleration.x,rs.imu_linear_acceleration.y,rs.imu_linear_acceleration.z)-Vector3d(0,0,9.81)

#    self.q0Pos=self.q0Pos+0.005*self.q0Vel
#    self.q0Vel=self.q0Vel+0.005*acc

#    hrp4W_q[0] = [0.9962, 0., 0.0872, tx, 1, tz]
#    hrp4W_q[0] = [rs.imu_orientation.w, rs.imu_orientation.x, rs.imu_orientation.y, rs.imu_orientation.z, tx, 1., tz]    
    hrp4W_q[0] = [rs.imu_orientation.w, rs.imu_orientation.x, rs.imu_orientation.y, rs.imu_orientation.z, self.q0Pos[0], self.q0Pos[1], self.q0Pos[2]]    
#    hrp4W_q[0] = [0.996194698091746, 0, -0.087155742747658, 0, tx, 1, tz]#self.q0Pos[2]]    
#    hrp4W_q[0] = [0.9848, 0, 0, 0.1736, self.q0Pos[0], self.q0Pos[1], self.q0Pos[2]]
    
    if self.isDebug:
#      hrp4W_q[0] = [0.996, 0, -0.259, 0, self.q0Pos[0], self.q0Pos[1], self.q0Pos[2]]
#      hrp4W_q[0] = [0.996, 0, 0, -0.259, self.q0Pos[0], self.q0Pos[1], self.q0Pos[2]]
#      hrp4W_q[0] = [0.933, -0.0670, -0.250, 0.250, self.q0Pos[0], self.q0Pos[1], self.q0Pos[2]]
      
      quat=Quaterniond(1,0,0,0)
#      quat=Quaterniond(rs.imu_orientation.w, rs.imu_orientation.x, rs.imu_orientation.y, rs.imu_orientation.z)*quat
#      quat=Quaterniond(0.965925826289068, 0, 0, -0.258819045102521)*quat
#      quat=Quaterniond(0.999377106785000,-0.003316122395400,0.000945268338669,-0.035121335878400)*quat
      quat=Quaterniond(0.997375740497000,-0.001451862084460,-0.008628711870370,0.071868419320500)*quat
      hrp4W_q[0] = [quat.w(), quat.x(), quat.y(), quat.z(), self.q0Pos[0], self.q0Pos[1], self.q0Pos[2]]

    else:
      hrp4W_q[0] = [rs.imu_orientation.w, rs.imu_orientation.x, rs.imu_orientation.y, rs.imu_orientation.z, self.q0Pos[0], self.q0Pos[1], self.q0Pos[2]]    
    
    self.hrp4W.mbc.q=hrp4W_q
    
#    print rbdList(self.hrp4W.mbc.q)[self.hrp4W.jointIndexByName('R_ANKLE_P')]
    
    self.display_helper()

# control parameter
timeStep = 0.005

if __name__ == '__main__':
  rospy.init_node('test_zmp_com_playback_bridge_stableloop')
  
  # load the robot and the environment
  robots = loadRobots()
  for r in robots.robots:
    r.mbc.gravity = Vector3d(0., 0., 9.81)

  hrp4_index = 0
  env_index = 1

  hrp4 = robots.robots[hrp4_index]
  env = robots.robots[env_index]

  # compute foot position to be in contact with the ground
  rbd.forwardKinematics(hrp4.mb, hrp4.mbc)
  tz = -hrp4.surfaces['LeftFoot'].X_0_s(hrp4).translation().z()
  tx = -hrp4.surfaces['LeftFoot'].X_0_s(hrp4).translation().x() #zero the feet surface for the wPG
  hrp4_q = rbdList(hrp4.mbc.q)

  hrp4_q[0] = [1., 0., 0., 0., tx, 0., tz]
  hrp4.mbc.q = hrp4_q
#  print len(rbdList(hrp4.mbc.q))
  # compute init fk and fv
  for r in robots.robots:
    rbd.forwardKinematics(r.mb, r.mbc)
    rbd.forwardVelocity(r.mb, r.mbc)

  hrp4Jsp = JointStatePublisher(hrp4)

  # create stabilizer helper
  hrp4Stab = stabilizerMsg(hrp4)

  # create solver
  qpsolver = MRQPSolver(robots, timeStep)

  # add dynamics constraint to QPSolver
  # Use 50% of the velocity limits cf Sebastien Langagne.
  contactConstraint = ContactConstraint(timeStep, ContactConstraint.Position)
#  contactConstraint = ContactConstraint(timeStep, ContactConstraint.Velocity)
  dynamicsConstraint1 = DynamicsConstraint(robots, hrp4_index, timeStep,
                                           damper=(0.1, 0.01, 0.5), velocityPercent=0.5)
  kinConstraint1 = KinematicsConstraint(robots, hrp4_index, timeStep,
                                        damper=(0.1, 0.01, 0.5), velocityPercent=0.5)
  qpsolver.addConstraintSet(contactConstraint)
  qpsolver.addConstraintSet(dynamicsConstraint1)

  # stability tasks
  postureTask1 = tasks.qp.PostureTask(robots.mbs, hrp4_index,
                                      hrp4_q, 0.1, 10.)

  rFoot = hrp4.surfaces['RightFoot']
  lFoot = hrp4.surfaces['LeftFoot']
#  rFoot = hrp4.surfaces['RightFootAnkProj']
#  lFoot = hrp4.surfaces['LeftFootAnkProj']

  rf_pos_goal = rFoot.X_0_s(hrp4).translation() - rFoot.X_b_s.translation()
#  rfPosTask, rfPosTaskTr = positionTrackingTask(robots, hrp4_index, 'r_ankle',
#                                                rf_pos_goal,
#                                                5., 5., 100000.)                                                
  rfPosTask, rfPosTaskTr = positionTask(robots, hrp4_index, 'r_ankle', rf_pos_goal, 5., 100000.)

  rf_ori_goal = rFoot.X_0_s(hrp4).rotation()
  rfOriTask, rfOriTaskSp = orientationTask(robots, hrp4_index, 'r_ankle', rf_ori_goal,
                                           5., 1000.)

  lf_pos_goal = lFoot.X_0_s(hrp4).translation() - lFoot.X_b_s.translation()
#  lfPosTask, lfPosTaskTr = positionTrackingTask(robots, hrp4_index, 'l_ankle', lf_pos_goal,
#                                                5., 5., 100000.)                                                
  lfPosTask, lfPosTaskTr = positionTask(robots, hrp4_index, 'l_ankle', lf_pos_goal, 5., 100000.)

  lf_ori_goal = lFoot.X_0_s(hrp4).rotation()
  lfOriTask, lfOriTaskSp = orientationTask(robots, hrp4_index, 'l_ankle', lf_ori_goal,
                                           5., 10000.)

  torsoOriTask, torsoOriTaskSp =\
  orientationTask(robots, hrp4_index, 'torso', list(hrp4.mbc.bodyPosW)[hrp4.bodyIndexByName('torso')].rotation(), 10., 200.)
#    orientationTask(robots, hrp4_index, 'torso', Matrix3d.Identity(), 10., 200.)
#    orientationTask(robots, hrp4_index, 'torso', list(hrp4.mbc.bodyPosW)[hrp4.bodyIndexByName('torso')].rotation(), 10., 200.)

  compp = rbd.computeCoM(hrp4.mb, hrp4.mbc)+Vector3d(0.0,0,0)
  compp[0] = -0.003653+0.0
  comTask, comTaskTr = comTrajectoryTask(robots, hrp4_index, compp,
                                         5., 5., 10000.)

  # allow the CoM to move in the Z direction
  comTaskTr.dimWeight(toEigenX(np.mat([1., 1., 0.1]).T))

  # allow the torso to rotate about the Z world axis
  torsoOriTaskSp.dimWeight(toEigenX(np.mat([1., 1., 0.1]).T))

  qpsolver.solver.addTask(rfPosTaskTr)
  qpsolver.solver.addTask(rfOriTaskSp)
#
  qpsolver.solver.addTask(lfPosTaskTr)
  qpsolver.solver.addTask(lfOriTaskSp)

  qpsolver.solver.addTask(torsoOriTaskSp)
  qpsolver.solver.addTask(comTaskTr)
  qpsolver.solver.addTask(postureTask1)

  # setup all
  c1L = MRContact(hrp4_index, env_index,
                  lFoot, env.surfaces['AllGround'])
  c1R = MRContact(hrp4_index, env_index,
                  rFoot, env.surfaces['AllGround'])

  contactConstraint.contactConstr.addVirtualContact(c1L.contactId(robots))
  contactConstraint.contactConstr.addVirtualContact(c1R.contactId(robots))

#  contactConstraint.contactConstr.addDofContact(c1L.contactId(robots),toEigenX(matL))
#  contactConstraint.contactConstr.addDofContact(c1R.contactId(robots),toEigenX(matR))
  qpsolver.setContacts([c1L, c1R])
  #qpsolver.setContacts([])  
  qpsolver.update()
  
  hrp4W=hrp4W_helper(hrp4)

  wpg_markers = WalkPGMarkers('wpg_markers')
  
#  rospy.Publisher(topic_name, MarkerArray)
  zmp_com_markers = ZMPCOMMarkers('zmp_com_markers')
  


  class Controller(object):
    def __init__(self):
      self.isRunning = True
      self.stopCB = ask_user.askUserNonBlock('stop_control', 'Stop')
      self.tongle_StartControlLoop = ask_user.askUserNonBlock('Start_Control_Loop', 'Start Control Loop')
      self.tongle_StopControlLoop = None #ask_user.askUserNonBlock('Stop_Control_Loop', 'Stop Control Loop')
      self.tongle_StartWalking = ask_user.askUserNonBlock('Start_Walking', 'Start Walking')
      self.tongle_StopWalking = None #ask_user.askUserNonBlock('Stop_Walking', 'Stop Walking')
      self.tongle_StartFootWidthEnlarge = ask_user.askUserNonBlock('Start_FootWidthEnlarge', 'Start FootWidthEnlarge')
      self.tongle_StopFootWidthEnlarge = None #ask_user.askUserNonBlock('Stop_FootWidthEnlarge', 'Stop FootWidthEnlarge')
      self.fsm = self.wait_init_position
      self.move_sequence = ['walk']
      self.next_move = None

      self.isWPGMarkerPublished = True
      # gains during the walking task
#      self.walkingGains = [[rfOriTaskSp, 40000.],
#                           [lfOriTaskSp, 40000.],
#                           [torsoOriTaskSp, 100.],
#                           [postureTask1, 100.]]
      self.walkingGains = [[rfOriTaskSp, 400.],
                           [lfOriTaskSp, 400.],
                           [torsoOriTaskSp, 10.],
                           [postureTask1, 1.]]
#      self.walkingGains = [[rfOriTaskSp, 400.],
#                           [lfOriTaskSp, 400.],
#                           [postureTask1, 1.]]
                           
      self.gainsToUse = []

      self.zmp_d = Vector3d.Zero()
      
      self.time_playback=0.0
      self.time_run=0.0
      self.start=0.0
      self.time_qp=0.0

    def wait_init_position(self, rs):
      if torsoOriTask.eval().norm() < 0.1 and torsoOriTask.speed().norm() < 0.001:
        print 'done waiting'

        # set the com tracking task gains
        comStiffness = 150.
        comDamping = 2*np.sqrt(comStiffness) # critically damped
        comTaskTr.setGains(comStiffness, comDamping)

        # set the foot position tracking task gains
        #footStiffness = 5000.
        footStiffness=5000. #for stabilizer off
        footDamping = 2*np.sqrt(footStiffness) # critically damped
#        rfPosTaskTr.setGains(footStiffness, footDamping)
        rfPosTaskTr.stiffness(footStiffness)
#        lfPosTaskTr.setGains(footStiffness, footDamping)*
        lfPosTaskTr.stiffness(footStiffness)

        # Do the next move in the sequence
        self.next_move = self.move_sequence.pop(0)
        if self.next_move == 'walk':
          # transition to high gains on the tracking tasks for walking and low gains for grasping
          print 'doing a walk'
          self.gainInterp = GainInterpolator(self.walkingGains, 100)
          self.fsm = self.prepareWPG

        else:
          print 'unknown FSM... going idle'
          self.fsm = self.idle

    # FSM state: prepare to do the walk
    def prepareWPG(self, rs):
      if torsoOriTask.eval().norm() < 0.05 and torsoOriTask.speed().norm() < 0.001:
        if self.gainInterp.oneStep():
          print 'gains ended with: '
          print map(lambda x:x.stiffness(), self.gainInterp.tasks)

          # Create the walking pattern generator\
          hrp4W.update(rs)
          self.playbackBridge = PlaybackBridge(hrp4, rFoot, lFoot, timeStep,
                                               '../trajectoire_zmp/zmp_and_com.txt',
                                               '../trajectoire_zmp/pstep.txt',
                                               list(hrp4.mbc.bodyPosW)[hrp4.bodyIndexByName('torso')].rotation(),
                                               hrp4W.hrp4W)

          self.fsm = self.wPGiteration
#          self.fsm = self.idle
          
          print 'starting walk'
        else:
          print 'interpolating'
#      else:
#        print 'waiting for torso ori task convergence'

    # FSM state: walking
    def wPGiteration(self, rs):
      if not self.playbackBridge.hasEnded:
        if self.tongle_StartControlLoop is not None and self.tongle_StartControlLoop.check(): 
            if self.tongle_StopControlLoop is None:
                self.tongle_StopControlLoop=ask_user.askUserNonBlock('Stop_Control_Loop', 'Stop Control Loop')
                self.tongle_StartControlLoop=None                            
            self.playbackBridge.IsControlLoopActiv=True
            self.playbackBridge.IsWalkActiv=False
        elif self.tongle_StopControlLoop is not None and self.tongle_StopControlLoop.check(): 
            if self.tongle_StartControlLoop is None:            
                self.tongle_StopControlLoop=None
                self.tongle_StartControlLoop=ask_user.askUserNonBlock('Start_Control_Loop', 'Start Control Loop')            
            self.playbackBridge.IsControlLoopActiv=False
            self.playbackBridge.IsWalkActiv=False
            
        if self.tongle_StartWalking is not None and self.tongle_StartWalking.check(): 
            if self.tongle_StopWalking is None:
                self.tongle_StopWalking=ask_user.askUserNonBlock('Stop_Walking', 'Stop Walking')
                self.tongle_StartWalking=None                            
            self.playbackBridge.IsWalkActiv=True
        elif self.tongle_StopWalking is not None and self.tongle_StopWalking.check(): 
            if self.tongle_StartWalking is None:            
                self.tongle_StopWalking=None
                self.tongle_StartWalking=ask_user.askUserNonBlock('Start_Walking', 'Start Walking')
            self.playbackBridge.IsWalkActiv=False
            
        if self.tongle_StartFootWidthEnlarge is not None and self.tongle_StartFootWidthEnlarge.check(): 
            if self.tongle_StopFootWidthEnlarge is None:
                self.tongle_StopFootWidthEnlarge=ask_user.askUserNonBlock('Stop_FootWidthEnlarge', 'Stop FootWidthEnlarge')
                self.tongle_StartFootWidthEnlarge=None
            self.playbackBridge.FootWidthEnlarge=True
        elif self.tongle_StopFootWidthEnlarge is not None and self.tongle_StopFootWidthEnlarge.check(): 
            if self.tongle_StartFootWidthEnlarge is None:            
                self.tongle_StopFootWidthEnlarge=None
                self.tongle_StartFootWidthEnlarge=ask_user.askUserNonBlock('Start_FootWidthEnlarge', 'Start FootWidthEnlarge')
            self.playbackBridge.FootWidthEnlarge=False
            
#        self.playbackBridge.callWPG(qpsolver, comTask, comTaskTr, torsoOriTask, \
#                            rfPosTaskTr, lfPosTaskTr, rfOriTask, lfOriTask, c1L, c1R, rs)
        
#        hrp4W.isStarted=True        
        
        start=time.time()    
        self.playbackBridge.callWPG(qpsolver, comTask, comTaskTr, torsoOriTask, \
                            rfPosTask, lfPosTask, rfOriTask, lfOriTask, c1L, c1R, rs)
        print 'callWPG duration'
        self.time_playback=time.time()-start
        print self.time_playback
      else:
        print 'walk ended'
        self.checkSequence()

    def checkSequence(self):
      # check if there are still moves to make
      if self.move_sequence:
        print 'Sequence left: ', self.move_sequence
        self.fsm = self.wait_init_position
      else:
#        qpsolver.solver.removeTask(torsoOriTaskSp)
#        qpsolver.solver.removeTask(comTaskTr)
#        qpsolver.solver.removeTask(postureTask1)
        
#        qpsolver.solver.removeTask(rfPosTaskTr)
#        qpsolver.solver.removeTask(rfOriTaskSp)
#        qpsolver.solver.removeTask(lfPosTaskTr)
#        qpsolver.solver.removeTask(lfOriTaskSp)
        
#        contactConstraint.contactConstr.removeVirtualContact(c1L.contactId(robots))
#        contactConstraint.contactConstr.removeVirtualContact(c1R.contactId(robots))
##        
#        c1Ltoto = MRContact(hrp4_index, env_index,
#                  lFoot, env.surfaces['AllGround'])
#        c1Rtoto = MRContact(hrp4_index, env_index,
#                  rFoot, env.surfaces['AllGround'])
#        qpsolver.setContacts([c1Ltoto, c1Rtoto])
##        qpsolver.setContacts([])  
#        qpsolver.setContacts([c1L, c1R])
#        qpsolver.update()
#        print c1L.contactId(robots)
                        
        contactConstraint.contactConstr.removeVirtualContact(c1L.contactId(robots))
        contactConstraint.contactConstr.removeVirtualContact(c1R.contactId(robots))
        qpsolver.updateNrVars()        
        qpsolver.update()
        self.fsm = self.idle
        print 'idling'

    # main control loop
    def run(self, rs):
      'qp duration'
      self.time_qp=time.time()-self.start
      start=time.time()
      if self.stopCB is not None and self.stopCB.check():
        print 'stopping'
        self.stopCB = None
        self.isRunning = True
        self.hsCB = stopMotion(robots, qpsolver, postureTask1, None, rbdList(hrp4.mbc.q))
        self.fsm = self.waitHS

      if self.isRunning:
        if not qpsolver.run():
          print 'FAIL !!!'
          self.isRunning = False
          return
        curTime = rs.header.stamp

        # update the center of mass state
        rbd.forwardAcceleration(hrp4.mb, hrp4.mbc)
        self.com = rbd.computeCoM(hrp4.mb, hrp4.mbc)
        self.comA = rbd.computeCoMAcceleration(hrp4.mb, hrp4.mbc)
        
        hrp4W.update(rs)

#        print [rs.imu_orientation.w, rs.imu_orientation.x, rs.imu_orientation.y, rs.imu_orientation.z]
#        print rbdList(hrp4.mbc.q)[0]
#        print rbdList(hrp4W.mbc.bodyPosW)[0].rotation()
#        hrp4_q=rbdList(hrp4.mbc.q)
#        print hrp4_q[0]
#        hrp4W_q=rbdList(hrp4W.hrp4W.mbc.q)
#        print hrp4W_q[0]
#        i=1
#        name=rs.joint_name[i]
#        if hrp4.hasJoint(name):
#          print hrp4_q[hrp4.jointIndexByName(name)]
#          print rs.joint_position[i]
#          print hrp4_q[hrp4.jointIndexByName(name)][0]-rs.joint_position[i]
#        hrp4_q=rbdList(hrp4.mbc.q) 
#        for i, name in enumerate(rs.joint_name): 
#            if hrp4.hasJoint(name):
#              print name
#              print hrp4_q[hrp4.jointIndexByName(name)]
#              print rs.joint_position[i]
#              print hrp4_q[hrp4.jointIndexByName(name)][0]-rs.joint_position[i]        

        if self.fsm == self.wPGiteration:
          # Update ZMP to be published
          self.zmp_d = Vector3d(self.playbackBridge.zmp_des[0], self.playbackBridge.zmp_des[1], 0.0)

          # markers for debugging the walking pattern generator
          if self.isWPGMarkerPublished:
            self.zmp_actual = rbd.computeCentroidalZMP(hrp4.mbc, self.com, self.comA, 0.)

            # TODO: use the new API for this!
            #compute capture point:
            omega = np.sqrt(9.81/self.playbackBridge.robot_params.com_height_)
#            omega = np.sqrt(9.81/rbd.computeCoM(hrp4.mb, hrp4.mbc)[2])
            comVel = rbd.computeCoMVelocity(hrp4.mb, hrp4.mbc)
            capturePoint = self.com + (comVel/omega)
            capturePoint[2] = 0.0
                        
#            robotH = hrp4
#            bodyIdxR = robotH.bodyIndexByName('r_ankle')
#            posR=(list(robotH.mbc.bodyPosW)[bodyIdxR]).translation()
#            rotR=(list(robotH.mbc.bodyPosW)[bodyIdxR]).rotation()
#      
#            bodyIdxL = robotH.bodyIndexByName('l_ankle')
#            posL=(list(robotH.mbc.bodyPosW)[bodyIdxL]).translation()
#            rotL=(list(robotH.mbc.bodyPosW)[bodyIdxL]).rotation()

            # walking pattern generator RViZ markers
            wpg_markers.fill(self.zmp_actual, self.zmp_d,
                             self.com, self.playbackBridge.comRefPos,
                             [self.playbackBridge.nextStepPos[0], self.playbackBridge.nextStepPos[1], 0.0],
                             capturePoint,
                             self.playbackBridge.LoopControlHelper.zmpPosWanted_,
                             self.playbackBridge.LoopControlHelper.zmpPosWantedSatur_,
                             self.playbackBridge.LoopControlHelper.comPosMesured,
                             self.playbackBridge.LoopControlHelper.comVelMesured,
                             self.playbackBridge.LoopControlHelper.comAccMesured,
                             self.playbackBridge.LoopControlHelper.zmpPosDesired,
                             self.playbackBridge.LoopControlHelper.zmpPosMesured,
                             self.playbackBridge.LoopControlHelper.zmpRPosMesured,
                             self.playbackBridge.LoopControlHelper.zmpLPosMesured,
                             self.playbackBridge.LoopControlHelper.zmpRPosDesired,
                             self.playbackBridge.LoopControlHelper.zmpLPosDesired,
                             self.playbackBridge.LoopControlHelper.zmpRPosWantedSatur_,
                             self.playbackBridge.LoopControlHelper.zmpLPosWantedSatur_,
                             self.playbackBridge.LoopControlHelper.footRPosMesured_,
                             self.playbackBridge.LoopControlHelper.footLPosMesured_,
                             self.playbackBridge.ROriMarkerMesured,
                             self.playbackBridge.LOriMarkerMesured,
                             self.playbackBridge.ROriMarkerWanted,
                             self.playbackBridge.LOriMarkerWanted,
                             self.playbackBridge.LoopControlHelper.zmpPosMesured_,
                             self.playbackBridge.LoopControlHelper.zmpRPosMesured_,
                             self.playbackBridge.LoopControlHelper.zmpLPosMesured_,
                             self.playbackBridge.LoopControlHelper.comPosMesured_,
                             self.playbackBridge.LoopControlHelper.DeltaDisplR_,
                             self.playbackBridge.LoopControlHelper.DeltaDisplL_,
                             self.playbackBridge.LoopControlHelper.comPosDesired,
                             self.playbackBridge.LoopControlHelper.footRForceMesured_,
                             self.playbackBridge.LoopControlHelper.footLForceMesured_,
                             self.playbackBridge.LoopControlHelper.DeltaAnglR_,
                             self.playbackBridge.LoopControlHelper.DeltaAnglL_,
                             self.playbackBridge.LoopControlHelper.zmpRForceWanted_,
                             self.playbackBridge.LoopControlHelper.zmpLForceWanted_,
                             self.playbackBridge.LoopControlHelper.comVelMesured_,
                             self.playbackBridge.LoopControlHelper.comVelDesired,
                             self.playbackBridge.LoopControlHelper.capturePointMesured_,
                             self.playbackBridge.LoopControlHelper.zmpRPosWanted_,
                             self.playbackBridge.LoopControlHelper.zmpLPosWanted_,
                             self.playbackBridge.LoopControlHelper.footRTorquMesured_,
                             self.playbackBridge.LoopControlHelper.footLTorquMesured_,
                             self.playbackBridge.LoopControlHelper.int_delta_comPos)
            wpg_markers.publish()
                        
            zmp_com_markers.fill(self.playbackBridge.LoopControlHelper.comPosDesired,
                                 self.playbackBridge.LoopControlHelper.zmpPosDesired,
#                                 self.playbackBridge.LoopControlHelper.comPosMesured_,
#                                 self.playbackBridge.LoopControlHelper.zmpPosMesured_,
                                 Vector3d(self.time_qp,self.time_run,self.time_playback),
                                 Vector3d(self.playbackBridge.LoopControlHelper.stateType,self.playbackBridge.stateType,self.playbackBridge.LoopControlHelper.forceDistrib))
            zmp_com_markers.publish()
            

        # Publish all
#        hrp4Stab.publishZMPDesired(curTime, self.zmp_d)
        hrp4Stab.publish(curTime, self.com, self.comA)
        hrp4Jsp.publish(curTime)
        
        qpsolver.fillResult()
        q_posture = list(chain.from_iterable(list(postureTask1.posture())))
        qpsolver.qpRes.robots_state.append(Robot(q_posture, [], [])) 
        q_posture = list(postureTask1.eval())
        qpsolver.qpRes.robots_state.append(Robot(q_posture, [], []))
#        print len(postureTask1.eval())
#        print len(rbdList(postureTask1.posture()))
#        print len(rbdList(hrp4.mbc.q))
#        print hrp4.mb.nrDof()
        
        qpsolver.send(curTime)
        
        

        self.fsm(rs)
#        if not ((self.fsm == self.wait_init_position) or (self.fsm == self.prepareWPG)):
#          raw_input('wait user input')
        'callrun duration'
        self.time_run=time.time()-start
        
        self.start=time.time()
    # FSM state: after stopped go back to half-sitting
    def waitHS(self, rs):
      if self.hsCB is not None and self.hsCB.check():
        self.hsCB = None
        goHalfSitting(qpsolver, postureTask1, hrp4_q, \
                      [dynamicsConstraint1, contactConstraint], \
                      [kinConstraint1])
        self.fsm = self.idle

    def idle(self, rs):
#      robotH = hrp4
#      bodyIdxR = robotH.bodyIndexByName('r_ankle')
#      posR=(list(robotH.mbc.bodyPosW)[bodyIdxR]).translation()
#      print posR[0]
#      
#      bodyIdxL = robotH.bodyIndexByName('l_ankle')
#      posL=(list(robotH.mbc.bodyPosW)[bodyIdxL]).translation()
#      print posL[0]

      pass

  ask_user.askUser('start', 'Start')
  controller = Controller()
  rospy.Subscriber('/robot/sensors/robot_state', MCRobotState,
                   controller.run, queue_size=1, tcp_nodelay=True)
  rospy.spin()