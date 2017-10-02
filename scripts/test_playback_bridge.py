#! /usr/bin/env python
from zmp_com_playback_bridge import PlaybackBridge

import roslib; roslib.load_manifest('mc_control')
import rospy

import numpy as np

from eigen3 import Vector3d, Matrix3d, toEigenX, Vector2d
import spacevecalg as sva
import rbdyn as rbd
import tasks

from mc_rbdyn import loadRobots, rbdList, MRContact
from mc_solver import MRQPSolver, DynamicsConstraint, ContactConstraint, \
  KinematicsConstraint
from joint_state_publisher import JointStatePublisher

from mc_robot_msgs.msg import MCRobotState

from ask_user import ask_user

from tasks_helper import orientationTask, positionTrackingTask, toVecX, comTrajectoryTask
from stop_experiment_helper import stopMotion, goHalfSitting

from stabilizer_msg_helper import stabilizerMsg
from walk_PG_markers import WalkPGMarkers

from gain_interpolator import GainInterpolator

import time

# control parameters
timeStep = 0.005

if __name__ == '__main__':
  rospy.init_node('test_zmp_com_playback_bridge')

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
  rfPosTask, rfPosTaskTr = positionTrackingTask(robots, hrp4_index, 'r_ankle',
                                                rf_pos_goal,
                                                5., 5., 100000.)

  rf_ori_goal = rFoot.X_0_s(hrp4).rotation()
  rfOriTask, rfOriTaskSp = orientationTask(robots, hrp4_index, 'r_ankle', rf_ori_goal,
                                           5., 1000.)

  lf_pos_goal = lFoot.X_0_s(hrp4).translation() - lFoot.X_b_s.translation()
  lfPosTask, lfPosTaskTr = positionTrackingTask(robots, hrp4_index, 'l_ankle',
                                                lf_pos_goal,
                                                5., 5., 100000.)

  lf_ori_goal = lFoot.X_0_s(hrp4).rotation()
  lfOriTask, lfOriTaskSp = orientationTask(robots, hrp4_index, 'l_ankle', lf_ori_goal,
                                           5., 10000.)

  torsoOriTask, torsoOriTaskSp =\
    orientationTask(robots, hrp4_index, 'torso', Matrix3d.Identity(), 10., 200.)

    #orientationTask(robots, hrp4_index, 'torso', list(hrp4.mbc.bodyPosW)[hrp4.bodyIndexByName('torso')].rotation(), 10., 200.)

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
#  matL=np.eye(1,6);
#  matL[0,0]=0;
#  matL[0,5]=1;
#  print matL
#  matR=np.eye(1,6);
#  matR[0,0]=0;
#  matR[0,5]=1;
#  print matR
#  matL=np.eye(6,6);
#  matL[3,3]=0;
#  matL[3,5]=1;
#  print matL
#  matR=np.eye(6,6);
#  matR[3,3]=0;
#  matR[3,5]=1;
#  print matR  
#  contactConstraint.contactConstr.addDofContact(c1L.contactId(robots),toEigenX(matL))
#  contactConstraint.contactConstr.addDofContact(c1R.contactId(robots),toEigenX(matR))
  qpsolver.setContacts([c1L, c1R])
  #qpsolver.setContacts([])  
  qpsolver.update()

  wpg_markers = WalkPGMarkers('wpg_markers')
  
  

  class Controller(object):
    def __init__(self):
      self.isRunning = True
      self.stopCB = ask_user.askUserNonBlock('stop_control', 'Stop')
      self.fsm = self.wait_init_position
      self.move_sequence = ['walk']
      self.next_move = None

      self.isWPGMarkerPublished = True
      # gains during the walking task
      self.walkingGains = [[rfOriTaskSp, 40000.],
                           [lfOriTaskSp, 40000.],
                           [torsoOriTaskSp, 100.],
                           [postureTask1, 100.]]

      self.gainsToUse = []

      self.zmp_d = Vector3d.Zero()

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
        rfPosTaskTr.setGains(footStiffness, footDamping)
        lfPosTaskTr.setGains(footStiffness, footDamping)

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
          self.playbackBridge = PlaybackBridge(hrp4, rFoot, lFoot, timeStep,
                                               '../trajectoire_zmp/zmp_and_com.txt',
                                               '../trajectoire_zmp/pstep.txt',
                                               list(hrp4.mbc.bodyPosW)[hrp4.bodyIndexByName('torso')].rotation())

          self.fsm = self.wPGiteration
          print 'starting walk'
        else:
          print 'interpolating'
      else:
        print 'waiting for torso ori task convergence'

    # FSM state: walking
    def wPGiteration(self, rs):
      if not self.playbackBridge.hasEnded:
#        self.playbackBridge.callWPG(qpsolver, comTaskTr, torsoOriTask, \
#                            rfPosTaskTr, lfPosTaskTr, rfOriTask, lfOriTask, c1L, c1R)
        self.playbackBridge.callWPG(qpsolver, comTask, comTaskTr, torsoOriTask, \
                            rfPosTaskTr, lfPosTaskTr, rfOriTask, lfOriTask, c1L, c1R)
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
                             capturePoint)
            wpg_markers.publish()

        # Publish all
#        hrp4Stab.publishZMPDesired(curTime, self.zmp_d)
        hrp4Stab.publish(curTime, self.com, self.comA)

        hrp4Jsp.publish(curTime)
        qpsolver.send(curTime)

        self.fsm(rs)
#        if not ((self.fsm == self.wait_init_position) or (self.fsm == self.prepareWPG)):
#          raw_input('wait user input')

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
                   controller.run, queue_size=10, tcp_nodelay=True)
  rospy.spin()