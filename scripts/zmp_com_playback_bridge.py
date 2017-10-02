#from humoto import RobotParameters
from swing_foot_trajectory import swing_foot_tracktask_helper, swing_foot_visualizer

from eigen3 import Vector2d, Vector3d, Matrix3d
import spacevecalg as sva
import rbdyn as rbd

import numpy as np
from tasks_helper import toVecX, bodyState

import os

def loadFile(filename):
  result = []
  filename = os.path.join(os.path.dirname(__file__), filename)
  with open(filename) as inputfile:
    for line in inputfile:
      a=line.strip().split(' ')
      result.append([float(k) for k in a])
  return result

'''
Class to help playing back trajectories with JorisQP Tracking Tasks
'''

class RobotParameters(object):
  def __init__(self):
    self.max_step_len_ = 0.2
    self.min_feet_dist_ = 0.19 # In mpc-walkgen 0.19 is in the DS only, 0.2 afterwards
    self.max_feet_dist_ = 0.3
    self.feet_dist_default_ = 0.19
    self.com_height_ = 0.814; # Height of the CoM
    self.foot_length_ = 0.1372
    self.foot_width_ = 0.058
    self.step_height_ = 0.07

class task_playback_helper(object):
  def __init__(self, robot, bodyname, task):
    self.trackingTask = task
    self.bodyState = bodyState(robot, bodyname)
    
  def update(self, posDes, velDes, accDes):
    # get actual position and velocity
    pos = self.bodyState.getPosW()
    vel = self.bodyState.getVelW()

    self.trackingTask.errorPos(toVecX(posDes - pos))
    self.trackingTask.errorVel(toVecX(velDes - vel))
    self.trackingTask.refAccel(toVecX(accDes))

'''
This class handles bridging of precomputed ZMP and CoM for walking tasks
'''
class PlaybackBridge(object):
  def __init__(self, robot, rFoot, lFoot, timeStep, zmpcomFilename, pstepFilename, initAlpha):
    self.robot = robot
    # foot surfaces
    self.rFoot = rFoot
    self.lFoot = lFoot
    self.swingFoot = None
    self.timeStep = timeStep

    # TODO: maybe rename better
    self.zmpcom = loadFile(zmpcomFilename)
    self.pstep = loadFile(pstepFilename)

    # wPG parameters, format borrowed from humoto
    self.robot_params = RobotParameters()

    # initializations for the walk FSM
    self.wPG_iters = 1
    self.stateType = None
    self.previousStateType = None
    self.hasEnded = False

    # foot pose
    next_pstep = self.pstep.pop(0)
    self.nextStepPos = [next_pstep[0], next_pstep[1]]
    self.next_step_angle = next_pstep[2]
    self.pstep.pop(0) #TODO: this is a bug, throwing away a pstep should be bad, but seems needed in the current config

    self.last_rotation_angle_lfoot = 0.
    self.last_rotation_angle_rfoot = 0.

    # visualizer
    self.foot_traj_viz = swing_foot_visualizer('swing_foot_trajectory')

    # data for stabilizer
    self.zmp_des = Vector2d.Zero()

    # CoM Target tracking variables
    self.comRefPos = Vector3d().Zero()
    self.comRefVel = Vector3d().Zero()
    self.comRefAccel = Vector3d().Zero()

    # TODO: set this parameter better
    self.swing_time = 0.700
    
    self.pos_r=open("/home/pajon/log_hrp4/pos_r.txt","w")
    self.pos_l=open("/home/pajon/log_hrp4/pos_l.txt","w")
    self.rot_r=open("/home/pajon/log_hrp4/rot_r.txt","w")
    self.rot_l=open("/home/pajon/log_hrp4/rot_l.txt","w")
    self.pos_com=open("/home/pajon/log_hrp4/pos_com.txt","w")
    
    self.initAlpha = initAlpha

    

  '''
  creates the Walking Pattern Generator
  params changes aren't taken into account anymore after this
  '''
  def createWPG(self, comTarget, zmpcomFile, pStepFile):

    # TODO: maybe comTarget is different
    self.comRefPos = comTarget
    self.comRefVel = Vector3d().Zero()
    self.comRefAccel = Vector3d().Zero()
    self.robot_params.com_height_ = comTarget[2]

    self.loadZMPandCoM(zmpcomFile)
    self.loadPStep(pStepFile)

  '''
  one iteration of the Walking Pattern Generator
  '''
  def callWPG(self, qpsolver, comTask, comTaskTr, torsoOriTask, \
              rfPosTaskTr, lfPosTaskTr, rfOriTask, lfOriTask, c1L, c1R):
    self.RFootHelper=task_playback_helper(self.robot, 'r_ankle',rfPosTaskTr)
    self.LFootHelper=task_playback_helper(self.robot, 'l_ankle',lfPosTaskTr)
    if len(self.zmpcom)>0:
      print '\n ========== wPG iteration ', self.wPG_iters
      self.wPG_iters += 1
      zmp_com_now = self.zmpcom.pop(0)

      # take the computed com state of the QP
      comPos = rbd.computeCoM(self.robot.mb, self.robot.mbc)
      self.pos_com.write(str(comPos[0])+" "+str(comPos[1])+" "+str(comPos[2])+"\n")
      
#      comVel = rbd.computeCoMVelocity(self.robot.mb, self.robot.mbc)

      # update state with walking pattern generator
      self.comRefPos = Vector3d(zmp_com_now[4]+0.0, zmp_com_now[5], zmp_com_now[6])
#      bu0 = 800
#      bu1 = 1400
#      bu2 = 1400 
#      bun = 2885
#      
#      if self.wPG_iters<=bu0:
#          self.comRefPos = Vector3d(zmp_com_now[4]+0.015, zmp_com_now[5], zmp_com_now[6])
#      elif self.wPG_iters<=bu1:
#          self.comRefPos = Vector3d(zmp_com_now[4]+0.015-(self.wPG_iters-bu0)*(0.005/(bu1-bu0)), zmp_com_now[5], zmp_com_now[6])
#      elif self.wPG_iters<=bu2:
#          self.comRefPos = Vector3d(zmp_com_now[4]+0.01, zmp_com_now[5], zmp_com_now[6])
#      elif self.wPG_iters<=bun:
#          self.comRefPos = Vector3d(zmp_com_now[4]+0.01, zmp_com_now[5], zmp_com_now[6])          

      self.comRefVel = Vector3d(zmp_com_now[7], zmp_com_now[8], zmp_com_now[9])
      self.comRefAccel = Vector3d(zmp_com_now[10], zmp_com_now[11], zmp_com_now[12])

      comTask.com(self.comRefPos)
      comTaskTr.refVel(toVecX(self.comRefVel))
      comTaskTr.refAccel(toVecX(self.comRefAccel))

      self.stateType = zmp_com_now[0] #0=TDS, 1=LSS, 2=RSS
      self.zmp_des[0] = zmp_com_now[1]
      self.zmp_des[1] = zmp_com_now[2]

      #TODO: very unsure of this part
      if ((self.stateType != self.previousStateType) and (self.previousStateType==0)):
        next_pstep=self.pstep.pop(0)
        self.nextStepPos = [next_pstep[0], next_pstep[1]]
        self.next_step_angle=next_pstep[2]

      # prevents a bug caused by the orientation task dimWeight #TODO: fix properly in Tasks self.initAlpha * 
      torsoOriTask.orientation(sva.RotZ((self.last_rotation_angle_rfoot + self.last_rotation_angle_lfoot)/2.))

      # double support
      if self.stateType==0:
        print 'state TDS'
        if not (self.previousStateType == 0):
          qpsolver.setContacts([c1L, c1R])
          qpsolver.update()
          print '------------updating contact state'

      # left single support
      elif self.stateType==1:
        self.swingFoot = self.rFoot
        print 'state LSS'
        if (self.previousStateType == 0):
          qpsolver.setContacts([c1L])
          qpsolver.update()
          print '------------updating contact state'
          self.last_rotation_angle_rfoot = self.next_step_angle
          
      # right single support
      elif self.stateType==2:
        self.swingFoot = self.lFoot
        print 'state RSS'
        if (self.previousStateType == 0):
          qpsolver.setContacts([c1R])
          qpsolver.update()
          print '------------updating contact state'
          self.last_rotation_angle_lfoot = self.next_step_angle
          
      rFootPos=Vector3d(zmp_com_now[22],zmp_com_now[23],zmp_com_now[24])
      rFootVel=Vector3d(zmp_com_now[25],zmp_com_now[26],zmp_com_now[27])
      rFootAcc=Vector3d(zmp_com_now[28],zmp_com_now[29],zmp_com_now[30])
#      rFootVel=Vector3d(0,0,0)
#      rFootAcc=Vector3d(0,0,0)
#      print self.RFootHelper.bodyState.getPosW()
      self.pos_r.write(str(self.RFootHelper.bodyState.getPosW()[0])+" "+str(self.RFootHelper.bodyState.getPosW()[1])+" "+str(self.RFootHelper.bodyState.getPosW()[2])+"\n")      
      self.RFootHelper.update(rFootPos,rFootVel,rFootAcc)
      self.rot_r.write(str(self.RFootHelper.bodyState.getOriW())+"\n")  
      rfOriTask.orientation(sva.RotX(zmp_com_now[34])*sva.RotY(zmp_com_now[35])*sva.RotZ(zmp_com_now[36]))
       
      lFootPos=Vector3d(zmp_com_now[13],zmp_com_now[14],zmp_com_now[15])
      lFootVel=Vector3d(zmp_com_now[16],zmp_com_now[17],zmp_com_now[18])
      lFootAcc=Vector3d(zmp_com_now[19],zmp_com_now[20],zmp_com_now[21])
#      lFootVel=Vector3d(0,0,0)
#      lFootAcc=Vector3d(0,0,0)
#      print self.LFootHelper.bodyState.getPosW()
      self.pos_l.write(str(self.LFootHelper.bodyState.getPosW()[0])+" "+str(self.LFootHelper.bodyState.getPosW()[1])+" "+str(self.LFootHelper.bodyState.getPosW()[2])+"\n")      
      self.LFootHelper.update(lFootPos,lFootVel,lFootAcc)
      self.rot_l.write(str(self.LFootHelper.bodyState.getOriW())+"\n")      
      lfOriTask.orientation(sva.RotX(zmp_com_now[31])*sva.RotY(zmp_com_now[32])*sva.RotZ(zmp_com_now[33]))
     
      # used to compare if contact state needs updating
      self.previousStateType = self.stateType
      
#      print zmp_com_now[22],zmp_com_now[23],zmp_com_now[24]
#      print zmp_com_now[25],zmp_com_now[26],zmp_com_now[27]
#      print zmp_com_now[28],zmp_com_now[29],zmp_com_now[30]
#      
#      print zmp_com_now[13],zmp_com_now[14],zmp_com_now[15]
#      print zmp_com_now[16],zmp_com_now[17],zmp_com_now[18]
#      print zmp_com_now[19],zmp_com_now[20],zmp_com_now[21]

    else:
      self.pos_r.write(str(self.RFootHelper.bodyState.getPosW()[0])+" "+str(self.RFootHelper.bodyState.getPosW()[1])+" "+str(self.RFootHelper.bodyState.getPosW()[2])+"\n")      
      self.pos_l.write(str(self.LFootHelper.bodyState.getPosW()[0])+" "+str(self.LFootHelper.bodyState.getPosW()[1])+" "+str(self.LFootHelper.bodyState.getPosW()[2])+"\n")      
      self.rot_r.write(str(self.RFootHelper.bodyState.getOriW())+"\n")      
      self.rot_l.write(str(self.LFootHelper.bodyState.getOriW())+"\n")      
      
      self.pos_r.close()
      self.pos_l.close()
      self.rot_r.close()
      self.rot_l.close()
      self.pos_com.close()
    
      
      print 'wPG ended with ', self.wPG_iters - 1, ' iterations'
      self.hasEnded = True