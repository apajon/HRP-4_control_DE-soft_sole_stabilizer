#from humoto import RobotParameters
from swing_foot_trajectory import swing_foot_tracktask_helper, swing_foot_visualizer

from eigen3 import Vector2d, Vector3d, Quaterniond
import spacevecalg as sva
import rbdyn as rbd
from mc_rbdyn import rbdList

import numpy as np


from tasks_helper import toVecX, bodyState

import os

from zmp_com_playback_bridge_controlloop import loop_control_helper

from math import sqrt, cos, sin

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
  def __init__(self, robot, rFoot, lFoot, timeStep, zmpcomFilename, pstepFilename, initAlpha, robotW):
    self.robot = robot
    self.robotW=robotW
    # foot surfaces
    self.rFoot = rFoot
    self.lFoot = lFoot
    self.swingFoot = None
    self.timeStep = timeStep

    # TODO: maybe rename better
    self.zmpcom = loadFile(zmpcomFilename)
    self.pstep = loadFile(pstepFilename)
    
    self.previousRFootPos=Vector3d(0,-0.081600,0.093)
    self.previousLFootPos=Vector3d(0,0.081600,0.093)
    
    self.previousRfootPosFixed=Vector3d(0,-0.081600,0.093)
    self.previousLfootPosFixed=Vector3d(0,0.081600,0.093)

    # wPG parameters, format borrowed from humoto
    self.robot_params = RobotParameters()

    # initializations for the walk FSM
    self.wPG_iters = 1
    self.stateType = 0
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
#    self.swing_time = 0.700
    
    
    self.initAlpha = initAlpha
    
    self.LoopControlHelper = loop_control_helper(self.robot,self.robotW)
    
    self.IsControlLoopActiv= False
    self.IsWalkActiv=False
    self.FootWidthEnlarge=False
    
    self.ROriMarkerMesured=Vector3d().Zero()
    self.LOriMarkerMesured=Vector3d().Zero()
    self.ROriMarkerWanted=Vector3d().Zero()
    self.LOriMarkerWanted=Vector3d().Zero()
    
    self.comRefPosInit=None
    self.comRefVelInit=None
    self.comRefAccelInit=None
    self.zmp_desInit=None
    
    self.counterWalk=None
    self.counterFootEnlarge=None
    
    self.IsDebugWalking=False
    
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
              rfPosTaskTr, lfPosTaskTr, rfOriTask, lfOriTask, c1L, c1R,rs):
                
#    raw_input('wait user input')
    
#    self.LoopControlHelper.update(rs) 
    
    if self.wPG_iters==1:
        q0=rbdList(self.LoopControlHelper.robotW.mbc.q)[0]
        quatIMU=Quaterniond(q0[0], q0[1], q0[2], q0[3])
        quatIMU.normalize()
        self.LoopControlHelper.rotInitIMU=quatIMU.toRotationMatrix()
        self.LoopControlHelper.rotInitIMUinv=self.LoopControlHelper.rotInitIMU.inverse()
                
    self.RFootHelper=task_playback_helper(self.robot, 'r_ankle',rfPosTaskTr)
    self.LFootHelper=task_playback_helper(self.robot, 'l_ankle',lfPosTaskTr)
    
      
#    self.RWFootHelper=task_playback_helper(self.robotW, 'r_ankle',rfPosTaskTr)
#    self.LWFootHelper=task_playback_helper(self.robotW, 'l_ankle',lfPosTaskTr)  
    
#    print self.LFootHelper.bodyState.getOriW()    
#    print self.LWFootHelper.bodyState.getOriW()
    
 
#    self.LoopControlHelper.update(rs) 
                                      
#    print 'delta comVel'
#    print self.LoopControlHelper.comVelDesired-self.LoopControlHelper.comVelMesured_
#    print 'delta comPos'
#    print (self.LoopControlHelper.comPosDesired-(self.LoopControlHelper.footRPosFixed+self.LoopControlHelper.footLPosFixed)/2)-(self.LoopControlHelper.comPosMesured_-(self.LoopControlHelper.footRPosMesured_+self.LoopControlHelper.footLPosMesured_)/2)
#    print 'delta zmpPos'
#    print (self.LoopControlHelper.zmpPosDesired-(self.LoopControlHelper.footRPosFixed+self.LoopControlHelper.footLPosFixed)/2)-(self.LoopControlHelper.zmpPosMesured_-(self.LoopControlHelper.footRPosMesured_+self.LoopControlHelper.footLPosMesured_)/2)

#    if self.IsWalkActiv==False:
#      self.LoopControlHelper.update(self.robot,self.robotW,
#                                  self.RWFootHelper,self.LWFootHelper,
#                                  rs.wrench[iR],rs.wrench[iL])

          
#    if self.IsWalkActiv:
#      nbiter=160
##      nbiter=len(self.zmpcom)>0:
#    else:
#      nbiter=2000
      
#    if self.wPG_iters<nbiter:
    if True:
      print '\n ========== wPG iteration ', self.wPG_iters
      self.wPG_iters += 1

      # take the computed com state of the QP
      comPos = rbd.computeCoM(self.robot.mb, self.robot.mbc)
            
      
#      comVel = rbd.computeCoMVelocity(self.robot.mb, self.robot.mbc)
      if hasattr(self.LoopControlHelper, 'footRForceMesured'):
#        CheckingFloorContact=sqrt(self.LoopControlHelper.footRForceMesured[0]**2+self.LoopControlHelper.footRForceMesured[1]**2+self.LoopControlHelper.footRForceMesured[2]**2+
#                              (self.LoopControlHelper.footLForceMesured[0]**2+self.LoopControlHelper.footLForceMesured[1]**2+self.LoopControlHelper.footLForceMesured[2]**2))
        CheckingFloorContact=sqrt((self.LoopControlHelper.footRForceMesured[0]+self.LoopControlHelper.footLForceMesured[0])**2+
              (self.LoopControlHelper.footRForceMesured[1]+self.LoopControlHelper.footLForceMesured[1])**2+
              (self.LoopControlHelper.footRForceMesured[2]+self.LoopControlHelper.footLForceMesured[2])**2)
        if CheckingFloorContact<self.LoopControlHelper.M*9.81/2:
          self.IsControlLoopActiv=False

      if self.IsDebugWalking:
        ktoto=0.05
        maxcounterwalk=500.0
        
        if self.counterWalk>maxcounterwalk:
          self.IsWalkActiv=False
          
        # update state with walking pattern generator      
        if self.IsControlLoopActiv and not self.IsWalkActiv:
          self.comRefPos = self.LoopControlHelper.comPosDesired
          self.comRefVel = self.LoopControlHelper.comVelDesired
          self.comRefAccel = self.LoopControlHelper.comAccDesired
          
          self.zmp_des[0] = self.LoopControlHelper.zmpPosDesired[0]
          self.zmp_des[1] = self.LoopControlHelper.zmpPosDesired[1]
          self.stateType = 0
          
          
          if self.counterWalk>maxcounterwalk:
            self.comRefPos[1] = ktoto
            self.comRefVel[1]= 0
            self.comRefAccel[1]= 0
            self.zmp_des[1] = ktoto
            
        elif not self.IsControlLoopActiv and self.IsWalkActiv:
  #        # update state with walking pattern generator        
          self.comRefPos = self.LoopControlHelper.comPosDesired
          self.comRefVel = self.LoopControlHelper.comVelDesired
          self.comRefAccel = self.LoopControlHelper.comAccDesired
          
          self.zmp_des[0] = self.LoopControlHelper.zmpPosDesired[0]
          self.zmp_des[1] = self.LoopControlHelper.zmpPosDesired[1]
          self.stateType = 0        
          
          if self.counterWalk is None:
            self.counterWalk=1
          
          if self.counterWalk<=maxcounterwalk :
            self.comRefPos[1] = (1-cos(self.counterWalk/maxcounterwalk*np.pi))/2*ktoto
            self.comRefVel[1]= (sin(self.counterWalk/maxcounterwalk*np.pi))/2*ktoto*np.pi/maxcounterwalk*0
            self.comRefAccel[1]= (cos(self.counterWalk/maxcounterwalk*np.pi))/2*ktoto*(np.pi/maxcounterwalk)*(np.pi/maxcounterwalk)*0
            self.zmp_des[1] = -self.comRefAccel[1]*0.78/9.81+self.comRefPos[1]
            self.counterWalk += 1
            self.LoopControlHelper.comPosDesired[1] = self.comRefPos[1]
            self.LoopControlHelper.zmpPosDesired[1] = self.zmp_des[1]
          elif self.counterWalk>maxcounterwalk:
            self.comRefPos[1] = ktoto
            self.comRefVel[1]= 0
            self.comRefAccel[1]= 0
            self.zmp_des[1] = ktoto
            self.LoopControlHelper.comPosDesired[1] = self.comRefPos[1]
            self.LoopControlHelper.zmpPosDesired[1] = self.zmp_des[1]
          
        elif self.IsWalkActiv:          
          self.comRefPos = self.LoopControlHelper.comPosDesired
          self.comRefVel = self.LoopControlHelper.comVelDesired
          self.comRefAccel = self.LoopControlHelper.comAccDesired
          
          self.zmp_des[0] = self.LoopControlHelper.zmpPosDesired[0]
          self.zmp_des[1] = self.LoopControlHelper.zmpPosDesired[1]
          self.stateType = 0        
          
          if self.counterWalk is None:
            self.counterWalk=1
          
          if self.counterWalk<=maxcounterwalk :
            self.comRefPos[1] = (1-cos(self.counterWalk/maxcounterwalk*np.pi))/2*ktoto
            self.comRefVel[1]= (sin(self.counterWalk/maxcounterwalk*np.pi))/2*ktoto*np.pi/maxcounterwalk*0
            self.comRefAccel[1]= (cos(self.counterWalk/maxcounterwalk*np.pi))/2*ktoto*(np.pi/maxcounterwalk)*(np.pi/maxcounterwalk)*0
            self.zmp_des[1] = -self.comRefAccel[1]*0.78/9.81+self.comRefPos[1]
            self.counterWalk += 1
            self.LoopControlHelper.comPosDesired[1] = self.comRefPos[1]
            self.LoopControlHelper.zmpPosDesired[1] = self.zmp_des[1]
          elif self.counterWalk>maxcounterwalk:
            self.comRefPos[1] = ktoto
            self.comRefVel[1]= 0
            self.comRefAccel[1]= 0
            self.zmp_des[1] = ktoto
            self.LoopControlHelper.comPosDesired[1] = self.comRefPos[1]
            self.LoopControlHelper.zmpPosDesired[1] = self.zmp_des[1]
            
        else:
          self.comRefPos = self.LoopControlHelper.comPosDesired
          self.comRefVel = self.LoopControlHelper.comVelDesired
          self.comRefAccel = self.LoopControlHelper.comAccDesired
          
          self.zmp_des[0] = self.LoopControlHelper.zmpPosDesired[0]
          self.zmp_des[1] = self.LoopControlHelper.zmpPosDesired[1]
          self.stateType = 0
          
          
          if self.counterWalk>maxcounterwalk:
            self.comRefPos[1] = ktoto
            self.comRefVel[1]= 0
            self.comRefAccel[1]= 0
            self.zmp_des[1] = ktoto
            self.LoopControlHelper.comPosDesired[1] = self.comRefPos[1]
            self.LoopControlHelper.zmpPosDesired[1] = self.zmp_des[1]
      else:
        ktoto=0.0
        maxcounterwalk=500.0
        
        if self.IsWalkActiv and len(self.zmpcom)>0:
          zmp_com_now = self.zmpcom.pop(0)
        else:
          self.IsWalkActiv=False
        # update state with walking pattern generator      
        if self.IsControlLoopActiv and not self.IsWalkActiv:
  #        self.comRefPos = self.LoopControlHelper.comPosWanted
  #        self.comRefVel = self.LoopControlHelper.comVelWanted
  #        self.comRefAccel = self.LoopControlHelper.comAccWanted
          self.comRefPos = self.LoopControlHelper.comPosDesired+Vector3d(ktoto,0,0)
          
          self.LoopControlHelper.comVelDesired=Vector3d().Zero()
          self.LoopControlHelper.comAccDesire=Vector3d().Zero()
          self.comRefVel = self.LoopControlHelper.comVelDesired
          self.comRefAccel = self.LoopControlHelper.comAccDesired
          
  #        self.zmp_des[0] = self.LoopControlHelper.zmpPosWanted[0]
  #        self.zmp_des[1] = self.LoopControlHelper.zmpPosWanted[1]
#          self.zmp_des[0] = self.LoopControlHelper.zmpPosDesired[0]+ktoto
#          self.zmp_des[1] = self.LoopControlHelper.zmpPosDesired[1]
          self.zmp_des[0] = self.comRefPos[0]
          self.zmp_des[1] = self.comRefPos[1]          
#          self.stateType = 0
            
        elif self.IsControlLoopActiv and self.IsWalkActiv:
          # update state with walking pattern generator
          self.LoopControlHelper.comPosDesired = Vector3d(zmp_com_now[4]+ktoto, zmp_com_now[5], 0.780678)        
          self.LoopControlHelper.comVelDesired = Vector3d(zmp_com_now[7], zmp_com_now[8], zmp_com_now[9])
          self.LoopControlHelper.comAccDesired = Vector3d(zmp_com_now[10], zmp_com_now[11], zmp_com_now[12])
          
          self.LoopControlHelper.previousStateType = self.stateType
          self.LoopControlHelper.stateType=zmp_com_now[0]
          self.LoopControlHelper.zmpPosDesired[0] = zmp_com_now[1]+ktoto
          self.LoopControlHelper.zmpPosDesired[1] = zmp_com_now[2]
          
          self.comRefPos = Vector3d(zmp_com_now[4]+ktoto, zmp_com_now[5], 0.780678)        
          self.comRefVel = Vector3d(zmp_com_now[7], zmp_com_now[8], zmp_com_now[9])
          self.comRefAccel = Vector3d(zmp_com_now[10], zmp_com_now[11], zmp_com_now[12])
          
          self.previousStateType = self.stateType
          self.stateType = zmp_com_now[0] #0=TDS, 1=LSS, 2=RSS
          self.zmp_des[0] = zmp_com_now[1]+ktoto
          self.zmp_des[1] = zmp_com_now[2]
          
        elif self.IsWalkActiv:
          self.LoopControlHelper.comPosDesired = Vector3d(zmp_com_now[4]+ktoto, zmp_com_now[5], 0.780678)        
          self.LoopControlHelper.comVelDesired = Vector3d(zmp_com_now[7], zmp_com_now[8], zmp_com_now[9])
          self.LoopControlHelper.comAccDesired = Vector3d(zmp_com_now[10], zmp_com_now[11], zmp_com_now[12])
          
          self.LoopControlHelper.previousStateType = self.stateType
          self.LoopControlHelper.stateType=zmp_com_now[0]
          self.LoopControlHelper.zmpPosDesired[0] = zmp_com_now[1]+ktoto
          self.LoopControlHelper.zmpPosDesired[1] = zmp_com_now[2]        
          
          # update state with walking pattern generator
          self.comRefPos = Vector3d(zmp_com_now[4]+ktoto, zmp_com_now[5], 0.780678)        
          self.comRefVel = Vector3d(zmp_com_now[7], zmp_com_now[8], zmp_com_now[9])
          self.comRefAccel = Vector3d(zmp_com_now[10], zmp_com_now[11], zmp_com_now[12])
          
          self.previousStateType = self.stateType
          self.stateType = zmp_com_now[0] #0=TDS, 1=LSS, 2=RSS
          self.zmp_des[0] = zmp_com_now[1]+ktoto
          self.zmp_des[1] = zmp_com_now[2]
            
        else:
          self.comRefPos = self.LoopControlHelper.comPosDesired
          
          self.LoopControlHelper.comVelDesired=Vector3d().Zero()
          self.LoopControlHelper.comAccDesire=Vector3d().Zero()
          self.comRefVel = self.LoopControlHelper.comVelDesired
          self.comRefAccel = self.LoopControlHelper.comAccDesired
          
          self.zmp_des[0] = self.LoopControlHelper.zmpPosDesired[0]
          self.zmp_des[1] = self.LoopControlHelper.zmpPosDesired[1]
          self.zmp_des[0] = self.comRefPos[0]
          self.zmp_des[1] = self.comRefPos[1]
#          self.stateType = 0
          if self.counterWalk is None:
            self.counterWalk=1
          if self.counterWalk<=maxcounterwalk :
            self.comRefPos[0] = (1-cos(self.counterWalk/maxcounterwalk*np.pi))/2*ktoto-0.0036526
            self.comRefVel[0]= (sin(self.counterWalk/maxcounterwalk*np.pi))/2*ktoto*np.pi/maxcounterwalk*0
            self.comRefAccel[0]= (cos(self.counterWalk/maxcounterwalk*np.pi))/2*ktoto*(np.pi/maxcounterwalk)*(np.pi/maxcounterwalk)*0
            self.zmp_des[0] = -self.comRefAccel[0]*0.78/9.81+self.comRefPos[0]
            self.counterWalk += 1
            self.LoopControlHelper.comPosDesired[0] = self.comRefPos[0]
            self.LoopControlHelper.zmpPosDesired[0] = self.zmp_des[0]
            self.zmpRPosDesired=Vector3d(self.LoopControlHelper.zmpPosDesired[0],-0.0815817,0)
            self.zmpLPosDesired=Vector3d(self.LoopControlHelper.zmpPosDesired[0],0.0815817,0)
     

      #TODO: very unsure of this part
      if self.IsWalkActiv:
        if ((self.stateType != self.previousStateType) and (self.previousStateType==0)):
          next_pstep=self.pstep.pop(0)
          self.nextStepPos = [next_pstep[0], next_pstep[1]]
          self.next_step_angle=next_pstep[2]
          
          if self.stateType==1:
            self.previousRfootPosFixed[0]=self.LoopControlHelper.footRPosFixed[0]
            self.previousRfootPosFixed[1]=self.LoopControlHelper.footRPosFixed[1]
            
            self.LoopControlHelper.footRPosFixed[0]=next_pstep[0]
            self.LoopControlHelper.footRPosFixed[1]=next_pstep[1]      
            self.LoopControlHelper.footRAnglFixed=next_pstep[2]
          else:
            self.previousLfootPosFixed[0]=self.LoopControlHelper.footLPosFixed[0]
            self.previousLfootPosFixed[1]=self.LoopControlHelper.footLPosFixed[1]
            
            self.LoopControlHelper.footLPosFixed[0]=next_pstep[0]
            self.LoopControlHelper.footLPosFixed[1]=next_pstep[1]
            self.LoopControlHelper.footLAnglFixed=next_pstep[2]

      '''
      '''
      if self.wPG_iters==2:
        self.LoopControlHelper.footRPosFixed=Vector3d(self.zmpcom[0][22],self.zmpcom[0][23],self.zmpcom[0][24])
        self.LoopControlHelper.footLPosFixed=Vector3d(self.zmpcom[0][13],self.zmpcom[0][14],self.zmpcom[0][15])
        self.LoopControlHelper.footRAnglFixed=self.zmpcom[0][36]
        self.LoopControlHelper.footLAnglFixed=self.zmpcom[0][33]

      self.LoopControlHelper.update(rs,self.IsControlLoopActiv)
      
      if self.FootWidthEnlarge:
        maxcounterFootEnlarge=500.0
        if self.counterFootEnlarge is None:
          self.counterFootEnlarge=0
        elif self.counterFootEnlarge<maxcounterFootEnlarge:
          self.counterFootEnlarge += 1
        
        if self.IsWalkActiv or self.IsControlLoopActiv:
          self.FootWidthEnlarge=False
   
#      '''
#      close loop in Qp targets update of COM and ZMP
#      '''
#      if self.IsControlLoopActiv and self.comRefPosInit is None:
#        self.comRefPosInit=self.comRefPos
#        self.comRefVelInit=self.comRefVel
#        self.comRefAccelInit=self.comRefAccel
#        self.zmp_desInit=self.zmp_des
#      elif not self.IsControlLoopActiv:
#        self.comRefPosInit=None
#        self.comRefVelInit=None
#        self.comRefAccelInit=None
#        self.zmp_desInit=None
#        
#      if self.IsControlLoopActiv:
#        self.zmp_desInit[0]=self.LoopControlHelper.zmpPosWantedSatur_[0]
#        self.zmp_desInit[1]=self.LoopControlHelper.zmpPosWantedSatur_[1]
#        
#        self.comRefAccelInit=self.LoopControlHelper.zmpForceWanted_/self.LoopControlHelper.M
#        self.comRefAccelInit[2]=0
#        
#        self.comRefPosInit=self.comRefPos+self.comRefVelInit*1/200+self.comRefAccelInit*(1/200)**2/2
#        
#        self.comRefVelInit=self.comRefVelInit+self.comRefAccelInit*1/200
#        
#        self.comRefPos=self.comRefPosInit
#        self.comRefVel=self.comRefVelInit
#        self.comRefAccel=self.comRefAccelInit
#        self.zmp_des=self.comRefPosInit
#      elif not self.comRefPosInit is None:
#        self.comRefPos=self.comRefPosInit
#        self.comRefVel=self.comRefVelInit*0
#        self.comRefAccel=self.comRefAccelInit*0
#        self.zmp_des=self.comRefPosInit
        
#      if self.wPG_iters==2:
#        self.comRefPosInit=self.comRefPos
#        self.comRefVelInit=self.comRefVel
#        self.comRefAccelInit=self.comRefAccel
#        self.zmp_desInit=self.zmp_des
#      else:
#        self.zmp_desInit[0]=self.LoopControlHelper.zmpPosWantedSatur_[0]
#        self.zmp_desInit[1]=self.LoopControlHelper.zmpPosWantedSatur_[1]
#        
#        self.comRefAccelInit=self.LoopControlHelper.zmpForceWanted_/self.LoopControlHelper.M
#        self.comRefAccelInit[2]=0
#        
#        self.comRefVelInit=self.comRefVelInit+self.comRefAccelInit*1/200
#        self.comRefPosInit=self.comRefPos+self.comRefVelInit*1/200
#        
#        self.comRefPos=self.comRefPosInit
#        self.comRefVel=self.comRefVelInit
#        self.comRefAccel=self.comRefAccelInit
#        self.zmp_des=self.zmp_desInit
        
      comTask.com(self.comRefPos)
      comTaskTr.refVel(toVecX(self.comRefVel))
      comTaskTr.refAccel(toVecX(self.comRefAccel))   

      # prevents a bug caused by the orientation task dimWeight #TODO: fix properly in Tasks self.initAlpha * 
      torsoOriTask.orientation(sva.RotY(0.14)*sva.RotZ((self.last_rotation_angle_rfoot + self.last_rotation_angle_lfoot)/2.))

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
          self.previousRFootPos=self.RFootHelper.bodyState.getPosW()
          
      # right single support
      elif self.stateType==2:
        self.swingFoot = self.lFoot
        print 'state RSS'
        if (self.previousStateType == 0):
          qpsolver.setContacts([c1R])
          qpsolver.update()
          print '------------updating contact state'
          self.last_rotation_angle_lfoot = self.next_step_angle
          self.previousLFootPos=self.LFootHelper.bodyState.getPosW()
          
      '''
      Manage Right Foot tasks
      ''' 
      if self.IsControlLoopActiv and not self.IsWalkActiv:
#        rFootPos=self.LoopControlHelper.footRPosWanted
        rFootPos=self.RFootHelper.bodyState.getPosW()+Vector3d(self.LoopControlHelper.DeltaDisplR_[0],self.LoopControlHelper.DeltaDisplR_[1],self.LoopControlHelper.DeltaDisplR_[2])#+self.LoopControlHelper.DeltaDisplR_
      elif self.IsControlLoopActiv and self.IsWalkActiv:
#          rFootPos=Vector3d(zmp_com_now[22],zmp_com_now[23],zmp_com_now[24])
        rFootPos=self.RFootHelper.bodyState.getPosW()+Vector3d(self.LoopControlHelper.DeltaDisplR_[0],self.LoopControlHelper.DeltaDisplR_[1],self.LoopControlHelper.DeltaDisplR_[2])#+self.LoopControlHelper.DeltaDisplR_
        if not self.IsDebugWalking:
          if self.stateType==1:
            rFootPos=self.previousRFootPos-self.previousRfootPosFixed+Vector3d(zmp_com_now[22],zmp_com_now[23],zmp_com_now[24])
          else:
            rFootPos=self.RFootHelper.bodyState.getPosW()+Vector3d(self.LoopControlHelper.DeltaDisplR_[0],self.LoopControlHelper.DeltaDisplR_[1],self.LoopControlHelper.DeltaDisplR_[2])
      elif self.IsWalkActiv:
#        rFootPos=Vector3d(zmp_com_now[22],zmp_com_now[23],zmp_com_now[24])
##        rFootPos=Vector3d(zmp_com_now[22],zmp_com_now[23],0.093)
##        rFootVel=Vector3d(zmp_com_now[25],zmp_com_now[26],zmp_com_now[27])
##        rFootAcc=Vector3d(zmp_com_now[28],zmp_com_now[29],zmp_com_now[30])
        rFootPos=self.RFootHelper.bodyState.getPosW()#-Vector3d(+0.005,0,0)
        if not self.IsDebugWalking:
          if self.stateType==1:
            rFootPos=self.previousRFootPos-self.previousRfootPosFixed+Vector3d(zmp_com_now[22],zmp_com_now[23],zmp_com_now[24])
          else:
            rFootPos=self.RFootHelper.bodyState.getPosW()
      elif self.FootWidthEnlarge:
        rFootPos=self.LoopControlHelper.footRPosInit+(self.LoopControlHelper.footRPosFixed-self.LoopControlHelper.footRPosInit)*self.counterFootEnlarge/maxcounterFootEnlarge
      else:
#        if self.FootWidthEnlarge:
#          rFootPos=Vector3d(self.zmpcom[0][22],max(self.RFootHelper.bodyState.getPosW()[1]-0.001,self.zmpcom[0][23]),self.zmpcom[0][24])
#        else:
        rFootPos=self.RFootHelper.bodyState.getPosW()
#      rFootVel=Vector3d(0,0,0)
#      rFootAcc=Vector3d(0,0,0)

#      self.RFootHelper.update(rFootPos,rFootVel,rFootAcc)
      rfPosTaskTr.position(rFootPos)
        
      if self.IsControlLoopActiv:
        rfOriTask.orientation(self.LoopControlHelper.DeltafootROri_*self.RFootHelper.bodyState.getOriW())
        
        toto3=self.RFootHelper.bodyState.getOriW()
        self.ROriMarkerMesured=toto3.eulerAngles(0,1,2)
        toto4=self.LoopControlHelper.DeltafootROri_*self.RFootHelper.bodyState.getOriW()
        self.ROriMarkerWanted=toto4.eulerAngles(0,1,2)
      elif self.IsWalkActiv:
#        rfOriTask.orientation(sva.RotX(zmp_com_now[34])*sva.RotY(zmp_com_now[35])*sva.RotZ(zmp_com_now[36]))
        rfOriTask.orientation(self.RFootHelper.bodyState.getOriW())
      elif self.FootWidthEnlarge:
        rfOriTask.orientation(sva.RotZ(self.LoopControlHelper.footRAnglFixed*self.counterFootEnlarge/maxcounterFootEnlarge))
      else:
#        rfOriTask.orientation(sva.RotX(self.LoopControlHelper.footRAnglDesired[0])*sva.RotY(self.LoopControlHelper.footRAnglDesired[1])*sva.RotZ(self.LoopControlHelper.footRAnglDesired[2]))    
        rfOriTask.orientation(self.RFootHelper.bodyState.getOriW())


      '''
      Manage Left Foot tasks
      ''' 
#      lFootPos=Vector3d(zmp_com_now[13],zmp_com_now[14],zmp_com_now[15])
#      lFootVel=Vector3d(zmp_com_now[16],zmp_com_now[17],zmp_com_now[18])
#      lFootAcc=Vector3d(zmp_com_now[19],zmp_com_now[20],zmp_com_now[21])
      if self.IsControlLoopActiv and not self.IsWalkActiv:
#        lFootPos=self.LoopControlHelper.footLPosWanted
        lFootPos=self.LFootHelper.bodyState.getPosW()+Vector3d(self.LoopControlHelper.DeltaDisplL_[0],self.LoopControlHelper.DeltaDisplL_[1],self.LoopControlHelper.DeltaDisplL_[2])#+self.LoopControlHelper.DeltaDisplR_
      elif self.IsControlLoopActiv and self.IsWalkActiv:
#        lFootPos=Vector3d(zmp_com_now[13],zmp_com_now[14],zmp_com_now[15])
        lFootPos=self.LFootHelper.bodyState.getPosW()+Vector3d(self.LoopControlHelper.DeltaDisplL_[0],self.LoopControlHelper.DeltaDisplL_[1],self.LoopControlHelper.DeltaDisplL_[2])#+self.LoopControlHelper.DeltaDisplR_
        if not self.IsDebugWalking:        
          if self.stateType==2:
            lFootPos=self.previousLFootPos-self.previousLfootPosFixed+Vector3d(zmp_com_now[13],zmp_com_now[14],zmp_com_now[15])
          else:
            lFootPos=self.LFootHelper.bodyState.getPosW()+Vector3d(self.LoopControlHelper.DeltaDisplL_[0],self.LoopControlHelper.DeltaDisplL_[1],self.LoopControlHelper.DeltaDisplL_[2])
      elif self.IsWalkActiv:
#        lFootPos=Vector3d(zmp_com_now[13],zmp_com_now[14],zmp_com_now[15])
##        lFootPos=Vector3d(zmp_com_now[13],zmp_com_now[14],0.093)
##        lFootVel=Vector3d(zmp_com_now[16],zmp_com_now[17],zmp_com_now[18])
##        lFootAcc=Vector3d(zmp_com_now[19],zmp_com_now[20],zmp_com_now[21])
        lFootPos=self.LFootHelper.bodyState.getPosW()#-Vector3d(-0.005,0,0)
        if not self.IsDebugWalking:
          if self.stateType==2:
            lFootPos=self.previousLFootPos-self.previousLfootPosFixed+Vector3d(zmp_com_now[13],zmp_com_now[14],zmp_com_now[15])
          else:
            lFootPos=self.LFootHelper.bodyState.getPosW()    
      elif self.FootWidthEnlarge:
        lFootPos=self.LoopControlHelper.footLPosInit+(self.LoopControlHelper.footLPosFixed-self.LoopControlHelper.footLPosInit)*self.counterFootEnlarge/maxcounterFootEnlarge
      else:
#        if self.FootWidthEnlarge:
#         lFootPos=Vector3d(self.zmpcom[0][13],min(self.LFootHelper.bodyState.getPosW()[1]+0.001,self.zmpcom[0][14]),self.zmpcom[0][15])
#        else:
         lFootPos=self.LFootHelper.bodyState.getPosW()
#      lFootVel=Vector3d(0,0,0)
#      lFootAcc=Vector3d(0,0,0)

#      self.LFootHelper.update(lFootPos,lFootVel,lFootAcc)
      lfPosTaskTr.position(lFootPos)
      

      if self.IsControlLoopActiv:
        lfOriTask.orientation(self.LoopControlHelper.DeltafootLOri_*self.LFootHelper.bodyState.getOriW())
        
        toto1=self.LFootHelper.bodyState.getOriW()   
        self.LOriMarkerMesured=toto1.eulerAngles(0,1,2)
        toto2=self.LoopControlHelper.DeltafootLOri_*self.LFootHelper.bodyState.getOriW()
        self.LOriMarkerWanted=toto2.eulerAngles(0,1,2)
      elif self.IsWalkActiv:
#        lfOriTask.orientation(sva.RotX(zmp_com_now[31])*sva.RotY(zmp_com_now[32])*sva.RotZ(zmp_com_now[33]))
        lfOriTask.orientation(self.LFootHelper.bodyState.getOriW())
      elif self.FootWidthEnlarge:
        lfOriTask.orientation(sva.RotZ(self.LoopControlHelper.footLAnglFixed*self.counterFootEnlarge/maxcounterFootEnlarge))
      else:
#        lfOriTask.orientation(sva.RotX(self.LoopControlHelper.footLAnglDesired[0])*sva.RotY(self.LoopControlHelper.footLAnglDesired[1])*sva.RotZ(self.LoopControlHelper.footLAnglDesired[2]))
        lfOriTask.orientation(self.LFootHelper.bodyState.getOriW())
#        lfOriTask.orientation(sva.RotX(self.LoopControlHelper.footLAnglDesired[0])*sva.RotY(self.LoopControlHelper.footLAnglDesired[1]+0.00175*self.wPG_iters)*sva.RotZ(self.LoopControlHelper.footLAnglDesired[2]))

      # used to compare if contact state needs updating
      self.previousStateType = self.stateType
      
#      if self.FootWidthEnlarge==True:
#        if self.LFootHelper.bodyState.getPosW()[1]+0.001>=self.zmpcom[0][14] and self.RFootHelper.bodyState.getPosW()[1]-0.001<=self.zmpcom[0][23]:
#          self.FootWidthEnlarge=False

    else:
      print 'wPG ended with ', self.wPG_iters - 1, ' iterations'
      self.hasEnded = True