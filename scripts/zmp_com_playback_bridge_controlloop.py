#from humoto import RobotParameters
from eigen3 import Vector3d, MatrixXd, Quaterniond
import spacevecalg as sva
import rbdyn as rbd

from tasks_helper import bodyState

import numpy as np

from math import sqrt, cos, sin

from mc_rbdyn import rbdList

import os

#import zmp_com_playback_bridge_controlloop_global as loop_global
from zmp_com_playback_bridge_controlloop_global import intersec_line, projection_convex

'''
This class handles loop control
'''
class loop_control_helper(object):
  def __init__ (self,robot,robotW):
    '''
    store robots
    '''
    self.robot=robot
    self.robotW=robotW
    
    '''
    store robot foot body
    '''
    self.RFootHelper=bodyState(self.robot, 'r_ankle')
    self.LFootHelper=bodyState(self.robot, 'l_ankle')
    
    self.RWFootHelper=bodyState(self.robotW, 'r_ankle')
    self.LWFootHelper=bodyState(self.robotW, 'l_ankle')
    
    '''
    initialaze COM and ZMP desired to Qp's robot
    '''
    self.comPosDesired=rbd.computeCoM(self.robot.mb, self.robot.mbc)
#    self.comVelDesired=rbd.computeCoMVelocity(self.robot.mb, self.robot.mbc)
    self.comVelDesired=Vector3d().Zero()
    self.comAccDesired=Vector3d().Zero()
    self.zmpPosDesired=rbd.computeCentroidalZMP(self.robot.mbc, rbd.computeCoM(self.robot.mb, self.robot.mbc), rbd.computeCoMAcceleration(self.robot.mb, self.robot.mbc), 0.)
    self.zmpRPosDesired=Vector3d(self.zmpPosDesired[0],-0.0815817,0)
    self.zmpLPosDesired=Vector3d(self.zmpPosDesired[0],0.0815817,0)
    
    
#    self.zmpPosDesired=Vector3d(-0.003653,0,0)   
#    self.comPosDesired=Vector3d(-0.003653,0.000473,0.780510)
#    self.comVelDesired=Vector3d().Zero()
#    self.comAccDesired=Vector3d().Zero()
#    self.zmpRPosDesired=Vector3d(-0.003653,-0.0815817,0)
#    self.zmpLPosDesired=Vector3d(-0.003653,0.0815817,0)
    
    '''
    initialaze the Fixed foot position to Qp's robot
    '''
#    self.footRPosFixed=Vector3d(0.000886819314634,-0.0815851525991,0.0936584280382)
#    self.footLPosFixed=Vector3d(0.000886819314634,0.0815851525991,0.0936584280382)   
    self.footRPosFixed=self.RFootHelper.getPosW()
    self.footLPosFixed=self.LFootHelper.getPosW() 
    self.footRAnkED=Vector3d(0.098,-0.011,0.094)
    self.footLAnkED=Vector3d(0.098,+0.011,0.094)
    self.footROrigED=self.footRPosFixed-self.footRAnkED
    self.footLOrigED=self.footLPosFixed-self.footLAnkED
    self.ankToSensor=Vector3d(0.0, 0.0, -0.093)
    
    '''
    initialaze DisplAngle foot to 0
    '''
#    self.footRAnglDesired=Vector3d(-0.002616768556987,-5.897394214936056e-04,-7.090894299247071e-06)
#    self.footLAnglDesired=Vector3d(0.002616315919231,-5.896305532123794e-04,-4.639983147034758e-06)
#    self.footRDisplDesired=Vector3d(5.72156393425283e-06,-2.45748989625317e-05,-4.187044549483934e-04) 
#    self.footLDisplDesired=Vector3d(5.65718083199387e-06,2.56372480591466e-05,-4.186996127954152e-04)
#    self.footRAnglDesired=self.RFootHelper.getOriW().eulerAngles(0,1,2)
#    self.footLAnglDesired=self.RFootHelper.getOriW().eulerAngles(0,1,2)
    self.footRAnglFixed=Vector3d().Zero()
    self.footLAnglFixed=Vector3d().Zero()
    self.footRAnglDesired=Vector3d().Zero()
    self.footLAnglDesired=Vector3d().Zero()
    self.footRDisplDesired=Vector3d().Zero()
    self.footLDisplDesired=Vector3d().Zero()
    
#    '''
#    initialaze DisplAngle foot to 0
##    '''
##    self.footRPosDesired=self.footROrigED+self.footRDisplDesired+sva.RotX(self.footRAnglDesired[0])*sva.RotY(self.footRAnglDesired[1])*sva.RotZ(self.footRAnglDesired[2])*(self.footRAnkED)
##    self.footLPosDesired=self.footLOrigED+self.footLDisplDesired+sva.RotX(self.footLAnglDesired[0])*sva.RotY(self.footLAnglDesired[1])*sva.RotZ(self.footLAnglDesired[2])*(self.footLAnkED)
#    self.footRPosDesired=self.footRPosFixed
#    self.footLPosDesired=self.footLPosFixed
    self.footRPosInit=self.footRPosFixed
    self.footLPosInit=self.footLPosFixed
    self.footRAnglInit=Vector3d().Zero()
    self.footLAnglInit=Vector3d().Zero()
    
    '''
    set sole margin and sole dimension
    '''
    self.sole_margin=0.02
    self.backtoankle=0.098
    self.fronttoankle=0.128
    self.inttoankle=0.054
    self.exttoankle=0.076
    
#    '''
#    set ZMP/COM control loop gains
#    '''
#    self.Tp=0.05 #robot mechanical estimated lags
#    self.M=38
##    self.M=None
#    self.w=sqrt(9.81/0.781739)
##    self.poles=[-13.,-3.,-self.w]
#    self.poles=[-13.*10,-3.*10,-self.w]
#
#    k3=-(sum(self.poles))*self.Tp-1
#    k1=np.prod(self.poles)*self.Tp/self.w**2-(1+k3)
#    k2=-self.Tp*(1+(self.poles[0]*self.poles[1]+self.poles[0]*self.poles[2]+self.poles[2]*self.poles[1])/self.w**2)
#    self.gains=Vector3d(k1,k2,k3)
#    
#    self.gainsIntegral=self.gains+Vector3d(0,0,1)
#    self.gainsProp=self.gains
    
    '''
    set ZMP/COM control loop gains with integrator
    '''
    self.Tp=0.12 #robot mechanical estimated lags
    self.M=38
    self.w=sqrt(9.81/0.781739)
    self.poles=[-4*1,-4*1,-3,-self.w];
    
    kr3=-(sum(self.poles))*self.Tp-1
    ki1=-np.prod(self.poles)*self.Tp/self.w**2
    
    kr2=-self.Tp*(1+(self.poles[0]*self.poles[1]+self.poles[0]*self.poles[2]+self.poles[0]*self.poles[3]+
                    self.poles[1]*self.poles[2]+self.poles[1]*self.poles[3]+
                    self.poles[2]*self.poles[3])/self.w**2)
          
    kr1=-kr3-self.Tp*(1/self.Tp-(self.poles[0]*self.poles[1]*self.poles[3]+self.poles[0]*self.poles[1]*self.poles[3]+
                          self.poles[0]*self.poles[2]*self.poles[3]+self.poles[1]*self.poles[2]*self.poles[3])/self.w**2)
                        
    kl1=kr1
    kl2=kr2
    kl3=kr3+1
    
    self.gainsKr=Vector3d(kr1,kr2,kr3)
    self.gainsKi=Vector3d(ki1,0,0)
    self.gainsKl=Vector3d(kl1,kl2,kl3)
        
    
    '''
    set ZMP/COM control loop gains
    '''
    self.Tgain=0.005
    self.TgainZMP=0.0015
    self.TgainForce=0.005
    self.gainJmat=1/self.Tgain * 1/200.
    self.gainZMPJmat=1/self.TgainZMP * 1/200.
    self.gainForceJmat=1/self.TgainForce * 1/200.
    
    '''
    set flexible sole gain matrix
    '''
#    self.JR=np.loadtxt("/home/jrluser/sharedVM/J_r.txt",delimiter="\t",dtype=float).reshape(6,6).transpose()
#    self.JL=np.loadtxt("/home/jrluser/sharedVM/J_l.txt",delimiter="\t",dtype=float).reshape(6,6).transpose()
    filenameJR = os.path.join(os.path.dirname(__file__), '../trajectoire_zmp/deformables/J_r.txt')
    filenameJL = os.path.join(os.path.dirname(__file__), '../trajectoire_zmp/deformables/J_l.txt')
    self.JR=np.loadtxt(filenameJR,delimiter="\t",dtype=float).reshape(6,6).transpose()
    self.JL=np.loadtxt(filenameJL,delimiter="\t",dtype=float).reshape(6,6).transpose()

    '''
    initalize things
    '''
    self.zmpPosWanted=Vector3d().Zero()
    self.zmpPosWantedSatur=Vector3d().Zero()
    self.comPosMesured=Vector3d().Zero()
    self.comVelMesured=Vector3d().Zero()
    self.comAccMesured=Vector3d().Zero()
    self.zmpPosMesured=Vector3d().Zero()
    self.zmpRPosMesured=Vector3d().Zero()
    self.zmpLPosMesured=Vector3d().Zero()
    self.zmpRPosWantedSatur=Vector3d().Zero()
    self.zmpLPosWantedSatur=Vector3d().Zero()
    self.footRPosMesured=self.RWFootHelper.getPosW()
    self.footLPosMesured=self.LWFootHelper.getPosW()
    
    self.zmpPosWanted_=Vector3d().Zero()
    self.zmpPosWantedSatur_=Vector3d().Zero()
    self.comPosMesured_=Vector3d().Zero()
    self.comVelMesured_=Vector3d().Zero()
#    self.comAccMesured_=Vector3d().Zero()
    self.zmpPosMesured_=Vector3d().Zero()
    self.zmpRPosMesured_=Vector3d().Zero()
    self.zmpLPosMesured_=Vector3d().Zero()
    self.zmpRPosWantedSatur_=Vector3d().Zero()
    self.zmpLPosWantedSatur_=Vector3d().Zero()
    self.footRPosMesured_=self.RWFootHelper.getPosW()
    self.footLPosMesured_=self.LWFootHelper.getPosW()
    
    self.footRForceMesured_=Vector3d().Zero()
    self.footLForceMesured_=Vector3d().Zero()
    self.footRTorquMesured_=Vector3d().Zero()
    self.footLTorquMesured_=Vector3d().Zero()
    
    self.DeltaDisplR_=Vector3d().Zero()
    self.DeltaDisplL_=Vector3d().Zero()
    
    self.DeltaAnglR_=Vector3d().Zero()
    self.DeltaAnglL_=Vector3d().Zero()
    
    self.hist_delta_comVel=[]
    self.hist_delta_comPos=[]
    self.hist_delta_zmpPos=[]
    
    self.zmpRForceWanted_=Vector3d().Zero()
    self.zmpLForceWanted_=Vector3d().Zero()
    
    self.capturePointMesured_=Vector3d().Zero()
    
    self.forceDistrib=0
    
    self.zmpRPosWanted_=Vector3d().Zero()
    self.zmpLPosWanted_=Vector3d().Zero()
    
    self.int_delta_comPos=Vector3d().Zero()
    
    '''
    set the real and Qp robots initial orientation error
    '''
#    q0=rbdList(robotW.mbc.q)[0]
    q0=rbdList(self.robotW.mbc.q)[0]
    quatIMU=Quaterniond(q0[0], q0[1], q0[2], q0[3])
    quatIMU.normalize()
    self.rotInitIMU=quatIMU.toRotationMatrix()
    self.rotInitIMUinv=self.rotInitIMU.inverse()

    '''
    store state Type
    0=TDS, 1=LSS, 2=RSS
    '''    
    self.stateType = 0
    self.previousStateType = None
    
    '''
    set the real and Qp robots initial error
    '''
    self.origCOM=Vector3d(0,1.0,0)
    robotW_q = rbdList(self.robotW.mbc.q)
    self.origQ0=Vector3d(robotW_q[0][4],robotW_q[0][5],robotW_q[0][6])
    
    '''
    initialize the mesured COM position and velocity
    '''
    self.comVelMesured=Vector3d().Zero()
#    self.comPosMesured=Vector3d(-0.003653,0.000473,0.780510)+self.origCOM
#    self.comPosMesured=rbd.computeCoM(self.robot.mb, self.robot.mbc)+self.origCOM
    self.comPosMesured=rbd.computeCoM(self.robotW.mb, self.robotW.mbc)
        
        
  def update(self,rs,IsControlLoopActiv):
    self.update_mesure(rs)
    self.compute_ZMP()
    self.update_align_robots()
    self.update_ZMP_COM_control_layer(IsControlLoopActiv)
    self.update_ZMP_saturation()
    self.update_ZMP_force_distributor()
    self.update_floor_reaction_force_control()
    self.update_foot_control()

  def update_mesure(self,rs):
    '''
    update the foot bodies
    '''
    self.RFootHelper=bodyState(self.robot, 'r_ankle')
    self.LFootHelper=bodyState(self.robot, 'l_ankle')
    
    self.RWFootHelper=bodyState(self.robotW, 'r_ankle')
    self.LWFootHelper=bodyState(self.robotW, 'l_ankle')    
    
    '''
    get the real robot wrench raw measurements
    '''
    for i, name in enumerate(rs.wrench_name): 
      if name=='RightFootForceSensor':
        iR=i
      if name=='LeftFootForceSensor':
        iL=i       
    wrenchR=rs.wrench[iR]
    wrenchL=rs.wrench[iL]
    
    '''
    compute the COM velocity
    
    the robot is considered as a rigid body
    feet in contact are considered fixed
    so the COM velocity is the discrete derivative of the COM position
    minus the foot velocity
    This is to suppress COM velocity drift error
    '''
      
#    if self.stateType==0 and self.footRForceMesured[2]>=20 and self.footLForceMesured[2]>=20:
    if self.stateType==0:
      footVelMean=((self.RWFootHelper.getPosW()-self.footRPosMesured)/0.005+(self.LWFootHelper.getPosW()-self.footLPosMesured)/0.005)/2
#    elif self.stateType==1 or self.footRForceMesured[2]<20:
    elif self.stateType==1:
      footVelMean=(self.LWFootHelper.getPosW()-self.footLPosMesured)/0.005
#    else self.stateType==2 or self.footLForceMesured[2]<20:
    elif self.stateType==2:
      footVelMean=(self.RWFootHelper.getPosW()-self.footRPosMesured)/0.005
    self.comVelMesured=(rbd.computeCoM(self.robotW.mb, self.robotW.mbc)-self.comPosMesured)/0.005-footVelMean 
    
    '''
    update the COM position
    '''    
    self.comPosMesured = rbd.computeCoM(self.robotW.mb, self.robotW.mbc)
    
    '''
    update the feet position and orientation
    '''  
    self.footRPosMesured=self.RWFootHelper.getPosW()
    self.footLPosMesured=self.LWFootHelper.getPosW()
    
    self.footROriMesured=self.RWFootHelper.getOriW()
    self.footLOriMesured=self.LWFootHelper.getOriW()
    

    self.footRAnglMesured=self.RWFootHelper.getOriW().eulerAngles(0,1,2)
    self.footLAnglMesured=self.LWFootHelper.getOriW().eulerAngles(0,1,2)
    
    '''
    update the ankle wrench with respect to the real robot orientation
    ''' 
    if self.M is None:
#      self.M=(sqrt(wrenchR.force.x**2+wrenchR.force.y**2+wrenchR.force.z**2)+
#                  sqrt(wrenchL.force.x**2+wrenchL.force.y**2+wrenchL.force.z**2))/9.81
      self.M=sqrt((wrenchR.force.x+wrenchL.force.x)**2+
              (wrenchR.force.y+wrenchL.force.y)**2+
              (wrenchR.force.z+wrenchL.force.z)**2)/9.81

    self.footRForceMesured=self.footROriMesured.inverse()*Vector3d(wrenchR.force.x,wrenchR.force.y,wrenchR.force.z)
    self.footLForceMesured=self.footLOriMesured.inverse()*Vector3d(wrenchL.force.x,wrenchL.force.y,wrenchL.force.z)
    
    torquRMove=self.ankToSensor.cross(Vector3d(wrenchR.force.x,wrenchR.force.y,wrenchR.force.z))
    torquLMove=self.ankToSensor.cross(Vector3d(wrenchL.force.x,wrenchL.force.y,wrenchL.force.z))
    self.footRTorquMesured=self.footROriMesured.inverse()*Vector3d(wrenchR.torque.x+torquRMove[0],wrenchR.torque.y+torquRMove[1],wrenchR.torque.z+torquRMove[2])
    self.footLTorquMesured=self.footLOriMesured.inverse()*Vector3d(wrenchL.torque.x+torquLMove[0],wrenchL.torque.y+torquLMove[1],wrenchL.torque.z+torquLMove[2])
    
    '''
    update total force mesured
    ''' 
    self.zmpForceMesured=self.footRForceMesured+self.footLForceMesured
        
    '''
    update IMU orientation
    '''
    q0=rbdList(self.robotW.mbc.q)[0]
    self.quatIMU=Quaterniond(q0[0], q0[1], q0[2], q0[3])
    self.quatIMU.normalize()
    self.rotIMU=self.quatIMU.toRotationMatrix()
    self.rotIMUinv=self.rotIMU.inverse()
        
    q0=rbdList(self.robot.mbc.q)[0]
    self.quatIMUQp=Quaterniond(q0[0], q0[1], q0[2], q0[3])
    self.quatIMU.normalize()
    self.rotIMUQp=self.quatIMUQp.toRotationMatrix()
    self.rotIMUQpinv=self.rotIMUQp.inverse()
    
    
  def compute_ZMP(self):
    '''
    compute the ZMP positions under each foot
    '''
#    self.footRForceMesured[2]=max(self.footRForceMesured[2],2)
#    self.footLForceMesured[2]=max(self.footLForceMesured[2],2)
                            
    if self.stateType==0:
      
      if self.footLForceMesured[2]>2:
        self.zmpLPosMesured=self.compute_zmp_under_foot(1)
      else:
        self.zmpLPosMesured=self.footLPosMesured
        self.footLForceMesured=Vector3d().Zero()
        self.footLTorquMesured=Vector3d().Zero()
      
      if self.footRForceMesured[2]>2:
        self.zmpRPosMesured=self.compute_zmp_under_foot(2)
      else:
        self.zmpRPosMesured=self.footRPosMesured
        self.footRForceMesured=Vector3d().Zero()
        self.footRTorquMesured=Vector3d().Zero()
      
#      self.zmpRPosMesured=Vector3d(self.footRPosMesured[0]-self.footRTorquMesured[1]/self.footRForceMesured[2]+self.footRForceMesured[0]/self.footRForceMesured[2]*(-self.footRAnkED[2]),
#                            self.footRPosMesured[1]+self.footRTorquMesured[0]/self.footRForceMesured[2]+self.footRForceMesured[1]/self.footRForceMesured[2]*(-self.footRAnkED[2]),
#                            self.footRPosMesured[2]-(self.footRAnkED[2]))
                            
#      self.zmpLPosMesured=Vector3d(self.footLPosMesured[0]-self.footLTorquMesured[1]/self.footLForceMesured[2]+self.footLForceMesured[0]/self.footLForceMesured[2]*(-self.footLAnkED[2]),
#                            self.footLPosMesured[1]+self.footLTorquMesured[0]/self.footLForceMesured[2]+self.footLForceMesured[1]/self.footLForceMesured[2]*(-self.footLAnkED[2]),
#                            self.footLPosMesured[2]-(self.footLAnkED[2]))
    elif self.stateType==1:
      self.zmpLPosMesured=self.compute_zmp_under_foot(1)
      self.zmpRPosMesured=self.footRPosMesured
#      self.zmpLPosMesured=Vector3d(self.footLPosMesured[0]-self.footLTorquMesured[1]/self.footLForceMesured[2]+self.footLForceMesured[0]/self.footLForceMesured[2]*(-self.footLAnkED[2]),
#                            self.footLPosMesured[1]+self.footLTorquMesured[0]/self.footLForceMesured[2]+self.footLForceMesured[1]/self.footLForceMesured[2]*(-self.footLAnkED[2]),
#                            self.footLPosMesured[2]-(self.footLAnkED[2]))
    else:
      self.zmpRPosMesured=self.compute_zmp_under_foot(2)
      self.zmpLPosMesured=self.footLPosMesured
#      self.zmpRPosMesured=Vector3d(self.footRPosMesured[0]-self.footRTorquMesured[1]/self.footRForceMesured[2]+self.footRForceMesured[0]/self.footRForceMesured[2]*(-self.footRAnkED[2]),
#                            self.footRPosMesured[1]+self.footRTorquMesured[0]/self.footRForceMesured[2]+self.footRForceMesured[1]/self.footRForceMesured[2]*(-self.footRAnkED[2]),
#                            self.footRPosMesured[2]-(self.footRAnkED[2]))

                            
    '''
    compute the ZMP position from ZMP under each foot
    
    It is compute on an horizontal plane under the COM with the theorical 
    COM  height as distance
    '''                 
#    self.zmpPosMesured=self.zmpRPosMesured*self.footRForceMesured[2]/self.zmpForceMesured[2]+self.zmpLPosMesured*self.footLForceMesured[2]/self.zmpForceMesured[2]
#    h=self.comPosMesured[2]-self.comPosDesired[2]
#    self.zmpPosMesured=Vector3d(-self.zmpRPosMesured[2]*self.footRForceMesured[0]/self.zmpForceMesured[2]
#                                +self.zmpRPosMesured[0]*self.footRForceMesured[2]/self.zmpForceMesured[2]
#                                -self.zmpLPosMesured[2]*self.footLForceMesured[0]/self.zmpForceMesured[2]
#                                +self.zmpLPosMesured[0]*self.footLForceMesured[2]/self.zmpForceMesured[2]
#                                +h*self.zmpForceMesured[0]/self.zmpForceMesured[2],
#                                -self.zmpRPosMesured[2]*self.footRForceMesured[1]/self.zmpForceMesured[2]
#                                +self.zmpRPosMesured[1]*self.footRForceMesured[2]/self.zmpForceMesured[2]
#                                -self.zmpLPosMesured[2]*self.footLForceMesured[1]/self.zmpForceMesured[2]
#                                +self.zmpLPosMesured[1]*self.footLForceMesured[2]/self.zmpForceMesured[2]
#                                +h*self.zmpForceMesured[1]/self.zmpForceMesured[2],
#                                  h)
    if self.footLForceMesured[2]<2 and self.footRForceMesured[2]<2:
      self.zmpPosMesured=self.comPosMesured-Vector3d(0,0,0.78)
    else:
      self.zmpPosMesured=self.compute_zmp_under_foot(3)
  
  def update_align_robots(self):
    '''
    Align mesured robot frame with control robot frame and put COM of Qp robot as zero
    '''
#    footRPosMesured_=self.rotInitIMU*self.rotIMUinv*(self.footRPosMesured-self.comPosMesured)
#    footLPosMesured_=self.rotInitIMU*self.rotIMUinv*(self.footLPosMesured-self.comPosMesured)
#    
#    orig=(self.footRPosFixed+self.footLPosFixed)/2-(footRPosMesured_+footLPosMesured_)/2 
#    
#    self.comVelMesured_=self.rotInitIMU*self.rotIMUinv*self.comVelMesured
#    
#    self.footRPosMesured_=footRPosMesured_+orig
#    self.footLPosMesured_=footLPosMesured_+orig
#    self.zmpRPosMesured_=self.rotInitIMU*self.rotIMUinv*(self.zmpRPosMesured-self.comPosMesured)+orig
#    self.zmpLPosMesured_=self.rotInitIMU*self.rotIMUinv*(self.zmpLPosMesured-self.comPosMesured)+orig
#    self.zmpPosMesured_=self.rotInitIMU*self.rotIMUinv*(self.zmpPosMesured-self.comPosMesured)+orig
#    self.comPosMesured_=Vector3d().Zero()+orig
#   
#    self.footROriMesured_=self.rotInitIMU*self.rotIMUinv*self.footROriMesured
#    self.footLOriMesured_=self.rotInitIMU*self.rotIMUinv*self.footLOriMesured
#    self.footRAnglMesured_=self.footROriMesured_.eulerAngles(0,1,2)
#    self.footLAnglMesured_=self.footLOriMesured_.eulerAngles(0,1,2)
#    
#    self.footRForceMesured_=self.rotInitIMU*self.rotIMUinv*self.footRForceMesured
#    self.footLForceMesured_=self.rotInitIMU*self.rotIMUinv*self.footLForceMesured


    angleIMU=self.quatIMU.toRotationMatrix().eulerAngles(0,1,2)
    angleIMUQp=self.quatIMUQp.toRotationMatrix().eulerAngles(0,1,2)

    sagital=Vector3d(1,0,0)
    Ori=self.quatIMU.toRotationMatrix()
    OriYawInv=sva.RotZ(angleIMU[2])
    if sagital.dot(OriYawInv*Ori*sagital)<0:
      OriYawInv=sva.RotZ(angleIMU[2]-np.pi)
    
    OriQp=self.quatIMUQp.toRotationMatrix()
    OriYaw=sva.RotZ(angleIMUQp[2]).inverse()
    if sagital.dot(OriYaw*OriQp*sagital)<0:
      OriYaw=sva.RotZ(angleIMUQp[2]-np.pi).inverse()
      
    OriTotal=OriYaw*OriYawInv
    
#    OriTotal=sva.RotZ(-angleIMUQp[2])*OriYawInv
#    OriTotal=sva.RotZ(-angleIMUQp[2])*sva.RotZ(angleIMU[2])
#    OriTotal=self.rotIMUinv*self.rotIMUQp
    
    footRPosMesured_=OriTotal*(self.footRPosMesured-self.comPosMesured)
    footLPosMesured_=OriTotal*(self.footLPosMesured-self.comPosMesured)
    
#    if self.stateType==0 and self.footRForceMesured[2]>=20 and self.footLForceMesured[2]>=20:
    if self.stateType==0:
      orig=(self.footRPosFixed+self.footLPosFixed)/2-(footRPosMesured_+footLPosMesured_)/2 
#    elif self.stateType==1 or self.footRForceMesured[2]<20:
    elif self.stateType==1:
      orig=self.footLPosFixed-footLPosMesured_
#    elif self.stateType==2 or self.footLForceMesured[2]<20:
    elif self.stateType==2:
      orig=self.footRPosFixed-footRPosMesured_
      
#    orig=(self.footRPosFixed+self.footLPosFixed)/2-(footRPosMesured_+footLPosMesured_)/2  
    orig[2]=0.78
    
    self.comVelMesured_=OriTotal*self.comVelMesured
    
    self.footRPosMesured_=footRPosMesured_+orig
    self.footLPosMesured_=footLPosMesured_+orig
    self.zmpRPosMesured_=OriTotal*(self.zmpRPosMesured-self.comPosMesured)+orig
    self.zmpLPosMesured_=OriTotal*(self.zmpLPosMesured-self.comPosMesured)+orig
    self.zmpPosMesured_=OriTotal*(self.zmpPosMesured-self.comPosMesured)+orig
    self.comPosMesured_=Vector3d().Zero()+orig+Vector3d(-0.00,0,0)
   
    self.footROriMesured_=OriTotal*self.footROriMesured
    self.footLOriMesured_=OriTotal*self.footLOriMesured
    self.footRAnglMesured_=self.footROriMesured_.eulerAngles(0,1,2)
    self.footLAnglMesured_=self.footLOriMesured_.eulerAngles(0,1,2)
    
    self.footRForceMesured_=OriTotal*self.footRForceMesured#+Vector3d(0,0,160-6400*(self.RFootHelper.getPosW()[2]-0.093))
    self.footLForceMesured_=OriTotal*self.footLForceMesured
    
    self.footRTorquMesured_=OriTotal*self.footRTorquMesured
    self.footLTorquMesured_=OriTotal*self.footLTorquMesured
    
    self.capturePointMesured_=self.comPosMesured_+1/self.w*self.comVelMesured_
        
#    print self.footRForceMesured_[2]
#    print Vector3d(0,0,160-6400*(self.RFootHelper.getPosW()[2]-0.093))
#    print (self.RFootHelper.getPosW()[2]-0.093)
    
  def update_ZMP_COM_control_layer(self,IsControlLoopActiv): 

#    if self.stateType==0 and self.footRForceMesured[2]>=20 and self.footLForceMesured[2]>=20:
    if self.stateType==0:
      origFixed=(self.footRPosFixed+self.footLPosFixed)/2
      orig=(self.footRPosMesured_+self.footLPosMesured_)/2
#    elif self.stateType==1 or self.footRForceMesured[2]<20:
    elif self.stateType==1:
      origFixed=self.footLPosFixed
      orig=self.footLPosMesured_
#    elif self.stateType==2 or self.footLForceMesured[2]<20:
    elif self.stateType==2:
      origFixed=self.footRPosFixed
      orig=self.footRPosMesured_
      
#    delta_comVel=self.comVelDesired-self.comVelMesured_
    delta_comPos=(self.comPosDesired-origFixed)-(self.comPosMesured_-orig)
#    delta_zmpPos=(self.zmpPosDesired-origFixed)-(self.zmpPosMesured_-orig)
     
    '''
    only proportional
    '''
#    self.delta_corr_=self.gains[0]*delta_comPos+self.gains[1]*delta_comVel+self.gains[2]*delta_zmpPos
#    self.delta_corr_=delta_comPos_+delta_comVel_+delta_zmpPos_
    
    '''
    integrator
    '''
    if not IsControlLoopActiv:
      self.int_delta_comPos=Vector3d().Zero()
      self.comIntPosList=[]
    
    activeIntWindow=True
    if activeIntWindow:
      if len(self.comIntPosList)>0.4*200:
        self.comIntPosList.pop(0)
      self.comIntPosList.append(delta_comPos)
      
      self.int_delta_comPos=Vector3d().Zero()
      for i in range(len(self.comIntPosList)):
        self.int_delta_comPos+=1/200*self.comIntPosList[i]
    else:
      self.int_delta_comPos=self.int_delta_comPos+1/200.*delta_comPos
      
    '''
    ZMP* computation
    '''
    delta_comVel_=self.gainsKl[1]*self.comVelDesired-self.gainsKr[1]*self.comVelMesured_
    delta_comPos_=self.gainsKi[0]*self.int_delta_comPos+self.gainsKl[0]*(self.comPosDesired-origFixed)-self.gainsKr[0]*(self.comPosMesured_-orig)
    delta_zmpPos_=self.gainsKl[2]*(self.zmpPosDesired-origFixed)-self.gainsKr[2]*(self.zmpPosMesured_-orig)
    
    print self.gainsKr
    print self.gainsKi
    print self.gainsKl


    
#    delta_comVel_=self.gainsIntegral[1]*self.int_comVel-self.gainsProp[1]*(self.comVelMesured_)
#    delta_comPos_=self.gainsIntegral[0]*self.int_comPos-self.gainsProp[0]*((self.comPosMesured_-orig))
#    delta_zmpPos_=self.gainsIntegral[2]*self.int_zmpPos-self.gainsProp[2]*((self.zmpPosMesured_-orig))

    self.delta_corr_=delta_comPos_+delta_comVel_+delta_zmpPos_

    self.zmpPosWanted_=self.delta_corr_+orig
    
    self.zmpPosWanted_[2]=self.zmpPosMesured_[2]

  def update_ZMP_saturation(self):
    '''
    test extension of zmp* support area
    '''
#    '''
#    create convex hull vertices
#    '''
#    [xv,yv,xvR_reduced,yvR_reduced,xvL_reduced,yvL_reduced,xvR,yvR,xvL,yvL]=self.compute_convexhull_vertices(self.footRPosMesured_,self.footLPosMesured_);
#    
##    '''
##    zmp* support area extension
##    '''
##    print xvR
##    print yvR
##    
##    gainHomo=self.Tp*200
##    for i in range(len(xv)):
##      xv[i]=self.zmpPosMesured_[0]+gainHomo*(xv[i]-self.zmpPosMesured_[0])
##      yv[i]=self.zmpPosMesured_[1]+gainHomo*(yv[i]-self.zmpPosMesured_[1])
##      
##    for i in range(len(xvR)):
##      xvR[i]=self.zmpPosMesured_[0]+gainHomo*(xvR[i]-self.zmpPosMesured_[0])
##      yvR[i]=self.zmpPosMesured_[1]+gainHomo*(yvR[i]-self.zmpPosMesured_[1])
##
##      xvL[i]=self.zmpPosMesured_[0]+gainHomo*(xvL[i]-self.zmpPosMesured_[0])
##      yvL[i]=self.zmpPosMesured_[1]+gainHomo*(yvL[i]-self.zmpPosMesured_[1])
##      
##    print xvR
##    print yvR
#    
#    '''
#    compute the orthogonal projection of ZMP on ankle segment
#    '''
#    if self.stateType==0:
#      a=[self.footLPosMesured_[0]-self.footRPosMesured_[0],self.footLPosMesured_[1]-self.footRPosMesured_[1]]
#      if a[0]==0.:
#        a=None
#        a_inv=0
#      elif a[1]==0.:
#        a=0
#        a_inv=None
#      else:
#        a=a[1]/a[0]
#        a_inv=-1/a
#        
#      [xp_ortho,yp_ortho]=intersec_line(a,self.footRPosMesured_[0],self.footRPosMesured_[1],
#                          a_inv,self.zmpPosWanted_[0],self.zmpPosWanted_[1])
#      
#      kn=((self.footLPosMesured_[0]-self.footRPosMesured_[0])*(xp_ortho-self.footRPosMesured_[0])+
#          (self.footLPosMesured_[1]-self.footRPosMesured_[1])*(yp_ortho-self.footRPosMesured_[1]))
#      kd=((self.footRPosMesured_[0]-self.footLPosMesured_[0])**2+
#          (self.footRPosMesured_[1]-self.footLPosMesured_[1])**2)
#      k_2=kn/kd
#    elif self.stateType==1:      
#      k_2=1
#    else:
#      k_2=0
#
#    '''
#    Bring back the ZMP into the Convex hull if necessary
#    '''
#    if k_2<=1 and k_2>=0 and self.stateType==0:
#      #bring the zmp into the convex hull defined  by the 2 feet if
#      #needed
#      #we search the closest point to ZMP projected on the convex
#      #hull edges along the perpendicular to ankle segment
#      [xp_0,yp_0,ii]=projection_convex(self.zmpPosWanted_[0],self.zmpPosWanted_[1],
#                                    xp_ortho,yp_ortho,
#                                    xv,yv)
#    elif k_2>=1:
#      k_2=1
#      #bring the zmp into the convex hull defined by L foot if needed
#      #we search the closest point to ZMP projected on the convex
#      #hull edges in the direction of the ankle
#      [xp_0,yp_0,ii]=projection_convex(self.zmpPosWanted_[0],self.zmpPosWanted_[1],
#                                    self.footLPosMesured_[0],self.footLPosMesured_[1],
#                                    xvL,yvL)
#    else:
#      k_2=0
#      #bring the zmp into the convex hull defined by R foot if needed
#      #we search the closest point to ZMP projected on the convex
#      #hull edges in the direction of the ankle
#      [xp_0,yp_0,ii]=projection_convex(self.zmpPosWanted_[0],self.zmpPosWanted_[1],
#                                    self.footRPosMesured_[0],self.footRPosMesured_[1],
#                                    xvR,yvR)
#    
#    print xp_0
#    print yp_0
#    '''                                 
#    Compute the position of ZMP1&2 with possible saturation
#    '''
#    if k_2<1 and k_2>0:
#      N=(self.footRPosMesured_+self.footLPosMesured_)/2
#      
#      aR=[xp_0-self.footRPosMesured_[0],yp_0-self.footRPosMesured_[1]]
#      if aR[0]==0.:
#        aR=None
##        aR_inv=0
#      elif aR[1]==0.:
#        aR=0
##        aR_inv=None
#      else:
#        aR=aR[1]/aR[0]
##        aR_inv=-1/aR
#        
#      [xp_L,yp_L]=intersec_line(a_inv,self.footLPosMesured_[0],self.footLPosMesured_[1],aR,N[0],N[1])
#      
#      aL=[xp_0-self.footLPosMesured_[0],yp_0-self.footLPosMesured_[1]]
#      if aL[0]==0.:
#        aL=None
##        aL_inv=0
#      elif aL[1]==0.:
#        aL=0
##        aL_inv=None
#      else:
#        aL=aL[1]/aL[0]
##        aL_inv=-1/L
#        
#      [xp_R,yp_R]=intersec_line(a_inv,self.footRPosMesured_[0],self.footRPosMesured_[1],aL,N[0],N[1])
#      
#      print xp_R
#      print xp_L
#      
#      print yp_R
#      print yp_L
#
#      #saturation of ZMPR&L
#      if k_2>0.5:
#        if self.stateType==0:
#          [xp_R,yp_R,ii]=projection_convex(xp_R,yp_R,
#                                          self.footRPosMesured_[0],self.footRPosMesured_[1],
#                                          xv,yv)
#        elif self.stateType==2:
#          [xp_R,yp_R,ii]=projection_convex(xp_R,yp_R,
#                                          self.footRPosMesured_[0],self.footRPosMesured_[1],
#                                          xvR,yvR)
#        xp_L=xp_R-1/k_2*(xp_R-xp_0)
#        yp_L=yp_R-1/k_2*(yp_R-yp_0)
#      else:
#        if self.stateType==0:
#          [xp_L,yp_L,ii]=projection_convex(xp_L,yp_L,
#                                          self.footLPosMesured_[0],self.footLPosMesured_[1],
#                                          xv,yv)
#        elif self.stateType==1:
#          [xp_L,yp_L,ii]=projection_convex(xp_L,yp_L,
#                                          self.footLPosMesured_[0],self.footLPosMesured_[1],
#                                          xvL,yvL)
#        xp_R=1/(1-k_2)*(xp_0-k_2*xp_L)
#        yp_R=1/(1-k_2)*(yp_0-k_2*yp_L)
#        
#      print xp_R
#      print xp_L
#      
#      print yp_R
#      print yp_L
#        
#    elif k_2==1:
#      xp_L=xp_0
#      yp_L=yp_0
#      xp_R=self.footRPosMesured_[0]
#      yp_R=self.footRPosMesured_[1]
##      xp_R=self.zmpRPosMesured_[0]
##      yp_R=self.zmpRPosMesured_[1]
#    else:#k_2==0:
#      xp_R=xp_0
#      yp_R=yp_0
#      xp_L=self.footLPosMesured_[0]
#      yp_L=self.footLPosMesured_[1]
##      xp_L=self.zmpLPosMesured_[0]
##      yp_L=self.zmpLPosMesured_[1]
#      
#        
#    '''                                 
#    ZMPs position into vector form
#    '''
##    print k_2
#    self.zmpPosWantedSatur_=Vector3d(xp_0,yp_0,self.zmpPosMesured_[2])
#    self.zmpRPosWantedSatur_=Vector3d(xp_R,yp_R,self.zmpPosMesured_[2])
#    self.zmpLPosWantedSatur_=Vector3d(xp_L,yp_L,self.zmpPosMesured_[2])
#    self.delta_corrSatur=self.zmpPosWantedSatur_-self.zmpPosDesired
#    self.delta_corrSatur[2]=0
#     
#    self.forceDistrib=k_2
#    print self.forceDistrib

    '''
    create convex hull vertices
    '''
#    [xv,yv,xvR_reduced,yvR_reduced,xvL_reduced,yvL_reduced,xvR,yvR,xvL,yvL]=self.compute_convexhull_vertices(self.footRPosFixed_,self.footRPosFixed);
#    [xv,yv,xvR_reduced,yvR_reduced,xvL_reduced,yvL_reduced,xvR,yvR,xvL,yvL]=self.compute_convexhull_vertices(self.footRPosMesured_,self.footLPosMesured_);
    [xv,yv,xvR_reduced,yvR_reduced,xvL_reduced,yvL_reduced,xvR,yvR,xvL,yvL]=self.compute_convexhull_vertices(self.footRPosMesured_,self.footLPosMesured_,self.footRAnglFixed,self.footLAnglFixed);
    
    '''
    compute the orthogonal projection of ZMP on ankle segment
    '''
    if self.stateType==0:
      a=[self.footLPosMesured_[0]-self.footRPosMesured_[0],self.footLPosMesured_[1]-self.footRPosMesured_[1]]
      if a[0]==0.:
        a=None
        a_inv=0
      elif a[1]==0.:
        a=0
        a_inv=None
      else:
        a=a[1]/a[0]
        a_inv=-1/a
        
      [xp_ortho,yp_ortho]=intersec_line(a,self.footRPosMesured_[0],self.footRPosMesured_[1],
                          a_inv,self.zmpPosWanted_[0],self.zmpPosWanted_[1])
      
      kn=((self.footLPosMesured_[0]-self.footRPosMesured_[0])*(xp_ortho-self.footRPosMesured_[0])+
          (self.footLPosMesured_[1]-self.footRPosMesured_[1])*(yp_ortho-self.footRPosMesured_[1]))
      kd=((self.footRPosMesured_[0]-self.footLPosMesured_[0])**2+
          (self.footRPosMesured_[1]-self.footLPosMesured_[1])**2)
      k_2=kn/kd
#      k_2=min(max(k_2,0.03),0.97)
    elif self.stateType==1:      
      k_2=1
    else:
      k_2=0

    '''
    Bring back the ZMP into the Convex hull if necessary
    '''
    if k_2<=1 and k_2>=0 and self.stateType==0:
      #bring the zmp into the convex hull defined  by the 2 feet if
      #needed
      #we search the closest point to ZMP projected on the convex
      #hull edges along the perpendicular to ankle segment
      [xp_0,yp_0,ii]=projection_convex(self.zmpPosWanted_[0],self.zmpPosWanted_[1],
                                    xp_ortho,yp_ortho,
                                    xv,yv)
    elif k_2>=1:
      k_2=1
      #bring the zmp into the convex hull defined by L foot if needed
      #we search the closest point to ZMP projected on the convex
      #hull edges in the direction of the ankle
      [xp_0,yp_0,ii]=projection_convex(self.zmpPosWanted_[0],self.zmpPosWanted_[1],
                                    self.footLPosMesured_[0],self.footLPosMesured_[1],
                                    xvL,yvL)
    else:
      k_2=0
      #bring the zmp into the convex hull defined by R foot if needed
      #we search the closest point to ZMP projected on the convex
      #hull edges in the direction of the ankle
      [xp_0,yp_0,ii]=projection_convex(self.zmpPosWanted_[0],self.zmpPosWanted_[1],
                                    self.footRPosMesured_[0],self.footRPosMesured_[1],
                                    xvR,yvR)
    
    '''                                 
    Compute the position of ZMP1&2 with possible saturation
    '''
    if k_2<1 and k_2>0:
      N=(self.footRPosMesured_+self.footLPosMesured_)/2
      
      aR=[xp_0-self.footRPosMesured_[0],yp_0-self.footRPosMesured_[1]]
      if aR[0]==0.:
        aR=None
#        aR_inv=0
      elif aR[1]==0.:
        aR=0
#        aR_inv=None
      else:
        aR=aR[1]/aR[0]
#        aR_inv=-1/aR
        
      [xp_L,yp_L]=intersec_line(a_inv,self.footLPosMesured_[0],self.footLPosMesured_[1],aR,N[0],N[1])
      
      aL=[xp_0-self.footLPosMesured_[0],yp_0-self.footLPosMesured_[1]]
      if aL[0]==0.:
        aL=None
#        aL_inv=0
      elif aL[1]==0.:
        aL=0
#        aL_inv=None
      else:
        aL=aL[1]/aL[0]
#        aL_inv=-1/L
        
      [xp_R,yp_R]=intersec_line(a_inv,self.footRPosMesured_[0],self.footRPosMesured_[1],aL,N[0],N[1])

      #saturation of ZMPR&L
      if k_2>0.5:
        [xp_R,yp_R,ii]=projection_convex(xp_R,yp_R,
                                        self.footRPosMesured_[0],self.footRPosMesured_[1],
                                        xvR,yvR)
        xp_L=xp_R-1/k_2*(xp_R-xp_0)
        yp_L=yp_R-1/k_2*(yp_R-yp_0)
      else:
        [xp_L,yp_L,ii]=projection_convex(xp_L,yp_L,
                                        self.footLPosMesured_[0],self.footLPosMesured_[1],
                                        xvL,yvL)
        xp_R=1/(1-k_2)*(xp_0-k_2*xp_L)
        yp_R=1/(1-k_2)*(yp_0-k_2*yp_L)
        
    elif k_2==1:
      xp_L=xp_0
      yp_L=yp_0
      xp_R=self.footRPosMesured_[0]
      yp_R=self.footRPosMesured_[1]
#      xp_R=self.zmpRPosMesured_[0]
#      yp_R=self.zmpRPosMesured_[1]
    else:#k_2==0:
      xp_R=xp_0
      yp_R=yp_0
      xp_L=self.footLPosMesured_[0]
      yp_L=self.footLPosMesured_[1]
#      xp_L=self.zmpLPosMesured_[0]
#      yp_L=self.zmpLPosMesured_[1]
        
    '''                                 
    ZMPs position into vector form
    '''
#    print k_2
    self.zmpPosWantedSatur_=Vector3d(xp_0,yp_0,self.zmpPosMesured_[2])
    self.zmpRPosWantedSatur_=Vector3d(xp_R,yp_R,self.zmpPosMesured_[2])
    self.zmpLPosWantedSatur_=Vector3d(xp_L,yp_L,self.zmpPosMesured_[2])
    self.delta_corrSatur=self.zmpPosWantedSatur_-self.zmpPosDesired
    self.delta_corrSatur[2]=0
    
#    if self.stateType==0:
##      if k_2==1:
##        k_2=0.96
##      elif k_2==0:
##        k_2==0.04
#      k_2=min(max(k_2,0.08),0.92)
    self.forceDistrib=k_2
    print self.forceDistrib
        
  def update_ZMP_force_distributor(self):
    '''
    Compute the ZMP reaction force from inverse pendulum method
    '''
    self.zmpForceWanted_=(self.comPosMesured_-self.zmpPosWantedSatur_)*(self.w)**2*self.M
    self.zmpForceWanted_[2]=self.M*9.81
    
    '''
    The ZMP reaction force is distributed under each foot
    '''
    self.zmpRForceWanted_=(1-self.forceDistrib)*self.zmpForceWanted_
    self.zmpLForceWanted_=self.forceDistrib*self.zmpForceWanted_
    
#    self.zmpRForceWanted_[2]=min(max(self.zmpRForceWanted_[2],1),self.M*9.81-1)
#    self.zmpLForceWanted_[2]=min(max(self.zmpRForceWanted_[2],1),self.M*9.81-1)
    
#    '''
#    Compute the COM motion wanted of the real robot
#    '''
#    self.comAccWanted_=Vector3d(-self.zmpForceWanted_[0]/self.M,-self.zmpForceWanted_[1]/self.M,0)
#    self.comVelWanted_=self.comVelMesured_+self.comAccWanted_*0.005
#    self.comVelWanted_[2]=0
#    self.comPosWanted_=self.comPosMesured_+self.comVelWanted_*0.005
#    self.comPosWanted_[2]=self.zmpPosMesured_[2]
    
    '''
    Bring the zmp wanted under each foot into the plane of the mesured ones
    '''
    if self.forceDistrib==1:
      kR=0
    else:
      kR=(self.zmpRPosMesured_[2]-self.zmpRPosWantedSatur_[2])/self.zmpRForceWanted_[2]
    self.zmpRPosWanted_=Vector3d(kR*self.zmpRForceWanted_[0]+self.zmpRPosWantedSatur_[0],
                                 kR*self.zmpRForceWanted_[1]+self.zmpRPosWantedSatur_[1],
                                 self.zmpRPosMesured_[2])
    if self.forceDistrib==0:
      kL=0
    else:
      kL=(self.zmpLPosMesured_[2]-self.zmpLPosWantedSatur_[2])/self.zmpLForceWanted_[2]
    self.zmpLPosWanted_=Vector3d(kL*self.zmpLForceWanted_[0]+self.zmpLPosWantedSatur_[0],
                                 kL*self.zmpLForceWanted_[1]+self.zmpLPosWantedSatur_[1],
                                 self.zmpLPosMesured_[2])
    
  def update_floor_reaction_force_control(self):
    DeltaFZR=MatrixXd().Zero(6,1)
    DeltaFZL=MatrixXd().Zero(6,1)
    
    ForceTotMesured=Vector3d(0,0,self.footRForceMesured_[2]+self.footLForceMesured_[2])
    normForceTotMesured=ForceTotMesured.norm()
    if normForceTotMesured>4:
      kScaleForce=self.M*9.81/normForceTotMesured

      DeltaFZR[0]=(kScaleForce*self.footRForceMesured_[0]-self.zmpRForceWanted_[0])
      DeltaFZR[1]=(kScaleForce*self.footRForceMesured_[1]-self.zmpRForceWanted_[1])
      DeltaFZR[2]=(kScaleForce*self.footRForceMesured_[2]-self.zmpRForceWanted_[2])
      DeltaFZR[3]=(self.zmpRPosMesured_[0]-self.zmpRPosWanted_[0])
      DeltaFZR[4]=(self.zmpRPosMesured_[1]-self.zmpRPosWanted_[1])
      
      DeltaFZL[0]=(kScaleForce*self.footLForceMesured_[0]-self.zmpLForceWanted_[0])
      DeltaFZL[1]=(kScaleForce*self.footLForceMesured_[1]-self.zmpLForceWanted_[1])
      DeltaFZL[2]=(kScaleForce*self.footLForceMesured_[2]-self.zmpLForceWanted_[2])
      DeltaFZL[3]=(self.zmpLPosMesured_[0]-self.zmpLPosWanted_[0])
      DeltaFZL[4]=(self.zmpLPosMesured_[1]-self.zmpLPosWanted_[1])
      
    else:
      kScaleForce=0
#    print DeltaFZR
#    print DeltaFZL
    
    for i in range(6):
      self.JR[0,i]=0
      self.JL[0,i]=0
      self.JR[1,i]=0
      self.JL[1,i]=0
    for i in range(2):  
      self.JR[2,i]=0
      self.JL[2,i]=0
    for i in range(3):
      self.JR[2,i+3]=0
      self.JL[2,i+3]=0
      self.JR[3,i]=0
      self.JL[3,i]=0
      self.JR[4,i]=0
      self.JL[4,i]=0
      self.JR[5,i]=0
      self.JL[5,i]=0
    
    self.DeltaDisplangleactR_=self.JR.dot(DeltaFZR)
#      self.DeltaDisplangleactR_=np.dot(self.DeltaDisplangleactR_,self.gainJmat)
    self.DeltaDisplangleactR_[0]=self.DeltaDisplangleactR_[0]*self.gainForceJmat
    self.DeltaDisplangleactR_[1]=self.DeltaDisplangleactR_[1]*self.gainForceJmat
    self.DeltaDisplangleactR_[2]=self.DeltaDisplangleactR_[2]*self.gainForceJmat
    self.DeltaDisplangleactR_[3]=self.DeltaDisplangleactR_[3]*self.gainZMPJmat
    self.DeltaDisplangleactR_[4]=self.DeltaDisplangleactR_[4]*self.gainZMPJmat
    self.DeltaDisplangleactR_[5]=self.DeltaDisplangleactR_[5]*self.gainZMPJmat
    
    self.DeltaDisplangleactL_=self.JL.dot(DeltaFZL)
#      self.DeltaDisplangleactL_=np.dot(self.DeltaDisplangleactL_,self.gainJmat)
    self.DeltaDisplangleactL_[0]=self.DeltaDisplangleactL_[0]*self.gainForceJmat
    self.DeltaDisplangleactL_[1]=self.DeltaDisplangleactL_[1]*self.gainForceJmat
    self.DeltaDisplangleactL_[2]=self.DeltaDisplangleactL_[2]*self.gainForceJmat
    self.DeltaDisplangleactL_[3]=self.DeltaDisplangleactL_[3]*self.gainZMPJmat
    self.DeltaDisplangleactL_[4]=self.DeltaDisplangleactL_[4]*self.gainZMPJmat
    self.DeltaDisplangleactL_[5]=self.DeltaDisplangleactL_[5]*self.gainZMPJmat
    
    '''
    Rescale angle rotation with ground pressure
    '''
    if normForceTotMesured>2:
      if self.stateType==0 and self.footRForceMesured_[2]*kScaleForce>=15 and self.footLForceMesured_[2]*kScaleForce>=15:
        kR=min(max((self.footRForceMesured_[2]*kScaleForce/(self.M*9.81/2)),0.5),1.5)
        self.DeltaDisplangleactR_[3]=self.DeltaDisplangleactR_[3]*kR
        self.DeltaDisplangleactR_[4]=self.DeltaDisplangleactR_[4]*kR
        self.DeltaDisplangleactR_[5]=self.DeltaDisplangleactR_[5]*kR
        
        kL=min(max((self.footLForceMesured_[2]*kScaleForce/(self.M*9.81/2)),0.5),1.5)
        self.DeltaDisplangleactL_[3]=self.DeltaDisplangleactL_[3]*kL
        self.DeltaDisplangleactL_[4]=self.DeltaDisplangleactL_[4]*kL
        self.DeltaDisplangleactL_[5]=self.DeltaDisplangleactL_[5]*kL
      elif self.stateType==0:
        kR=min(max((self.footRForceMesured_[2]*kScaleForce/(self.M*9.81/2)),0.5),0.9)
        self.DeltaDisplangleactR_[3]=self.DeltaDisplangleactR_[3]*kR
        self.DeltaDisplangleactR_[4]=self.DeltaDisplangleactR_[4]*kR
        self.DeltaDisplangleactR_[5]=self.DeltaDisplangleactR_[5]*kR
        
        kL=min(max((self.footLForceMesured_[2]*kScaleForce/(self.M*9.81/2)),0.5),0.9)
        self.DeltaDisplangleactL_[3]=self.DeltaDisplangleactL_[3]*kL
        self.DeltaDisplangleactL_[4]=self.DeltaDisplangleactL_[4]*kL
        self.DeltaDisplangleactL_[5]=self.DeltaDisplangleactL_[5]*kL
      else:
        kR=min(max((self.footRForceMesured_[2]*kScaleForce/(self.M*9.81/2)),0.5),0.9)
        self.DeltaDisplangleactR_[3]=self.DeltaDisplangleactR_[3]*kR
        self.DeltaDisplangleactR_[4]=self.DeltaDisplangleactR_[4]*kR
        self.DeltaDisplangleactR_[5]=self.DeltaDisplangleactR_[5]*kR
        
        kL=min(max((self.footLForceMesured_[2]*kScaleForce/(self.M*9.81/2)),0.5),0.9)
        self.DeltaDisplangleactL_[3]=self.DeltaDisplangleactL_[3]*kL
        self.DeltaDisplangleactL_[4]=self.DeltaDisplangleactL_[4]*kL
        self.DeltaDisplangleactL_[5]=self.DeltaDisplangleactL_[5]*kL

          
  def update_foot_control(self):
    
#    self.DeltaDisplR_=Vector3d(self.DeltaDisplangleactR_[0],self.DeltaDisplangleactR_[1],self.DeltaDisplangleactR_[2])
#    self.DeltaDisplL_=Vector3d(self.DeltaDisplangleactL_[0],self.DeltaDisplangleactL_[1],self.DeltaDisplangleactL_[2])   
    self.DeltaDisplR_=Vector3d(self.DeltaDisplangleactR_[0]*0,self.DeltaDisplangleactR_[1]*0,self.DeltaDisplangleactR_[2])
    self.DeltaDisplL_=Vector3d(self.DeltaDisplangleactL_[0]*0,self.DeltaDisplangleactL_[1]*0,self.DeltaDisplangleactL_[2])
    
    self.DeltaAnglR_=Vector3d(self.DeltaDisplangleactR_[3],self.DeltaDisplangleactR_[4],self.DeltaDisplangleactR_[5])
    self.DeltaAnglL_=Vector3d(self.DeltaDisplangleactL_[3],self.DeltaDisplangleactL_[4],self.DeltaDisplangleactL_[5])
    
    self.DeltafootROri_=sva.RotX(self.DeltaAnglR_[0])*sva.RotY(self.DeltaAnglR_[1])*sva.RotZ(self.DeltaAnglR_[2])
    self.DeltafootLOri_=sva.RotX(self.DeltaAnglL_[0])*sva.RotY(self.DeltaAnglL_[1])*sva.RotZ(self.DeltaAnglL_[2])

    if self.stateType==1 and self.footRForceMesured_.norm()<10:
      self.DeltafootROri_=sva.RotX(0)
      self.DeltaDisplR_=Vector3d.Zero()
    self.footROriWanted_=self.DeltafootROri_*self.footROriMesured_
      
    if self.stateType==2 and self.footLForceMesured_.norm()<10:
      self.DeltafootLOri_=sva.RotX(0)
      self.DeltaDisplL_=Vector3d.Zero()
    self.footLOriWanted_=self.DeltafootLOri_*self.footLOriMesured_
    
    
#    self.DeltafootROri_=sva.RotX(0)
#    self.DeltafootLOri_=sva.RotX(0)
    
#    print self.DeltaDisplR_
#    print self.DeltaDisplL_
    
#    self.footRPosDesired=self.footRPosDesired+Vector3d(0,0,self.DeltaDisplR_[2])#+self.DeltaDisplR_
#    self.footLPosDesired=self.footLPosDesired+Vector3d(0,0,self.DeltaDisplL_[2])#+self.DeltaDisplL_
    
#    self.DeltafootRPos_=self.footROriWanted_*self.footRAnkED-self.footROriMesured_*self.footRAnkED+self.DeltaDisplR_
#    self.DeltafootLPos_=self.footLOriWanted_*self.footLAnkED-self.footLOriMesured_*self.footLAnkED+self.DeltaDisplL_
#
#    self.footRPosWanted=self.footRPosMesured_+self.DeltafootRPos_
#    self.footLPosWanted=self.footLPosMesured_+self.DeltafootLPos_
  
  def compute_zmp_under_foot(self,foot):
          
      if foot==2:
        normal=Vector3d(0,0,-0.093)
        normalRot=self.footROriMesured.inverse()*normal
        
#        normal=Vector3d(0,0,-0.78+(self.comPosMesured[2]-self.footRPosMesured[2]))
#        normal=Vector3d(0,0,-0.093)
#        normalRot=normal
        
        A=self.footRPosMesured+normalRot
        normalScalA=-normalRot.dot(A)
        
        xFo=self.footRForceMesured[0]
        yFo=self.footRForceMesured[1]
        zFo=self.footRForceMesured[2]
        
        xMo=self.footRTorquMesured[0]
        yMo=self.footRTorquMesured[1]
        zMo=self.footRTorquMesured[2]
              
        xank=self.footRPosMesured[0]
        yank=self.footRPosMesured[1]
        zank=self.footRPosMesured[2]
      elif foot==1:
        normal=Vector3d(0,0,-0.093)
        normalRot=self.footLOriMesured.inverse()*normal
      
#        normal=Vector3d(0,0,-0.78+(self.comPosMesured[2]-self.footLPosMesured[2]))
#        normal=Vector3d(0,0,-0.093)
#        normalRot=normal
        
        A=self.footLPosMesured+normalRot
        normalScalA=-normalRot.dot(A)
        
        xFo=self.footLForceMesured[0]
        yFo=self.footLForceMesured[1]
        zFo=self.footLForceMesured[2]
        
        xMo=self.footLTorquMesured[0]
        yMo=self.footLTorquMesured[1]
        zMo=self.footLTorquMesured[2]
              
        xank=self.footLPosMesured[0]
        yank=self.footLPosMesured[1]
        zank=self.footLPosMesured[2]
        
      elif foot==3:
        normal=Vector3d(0,0,-0.78)
#        normalRot=self.rotIMU*normal
        normalRot=normal
        
        A=self.comPosMesured+normalRot
        normalScalA=-normalRot.dot(A)
        
        xFo=self.footLForceMesured[0]+self.footRForceMesured[0]
        yFo=self.footLForceMesured[1]+self.footRForceMesured[1]
        zFo=self.footLForceMesured[2]+self.footRForceMesured[2]
        
        vectR=-self.comPosMesured.cross(self.footRForceMesured)+self.footRPosMesured.cross(self.footRForceMesured)
        vectL=-self.comPosMesured.cross(self.footLForceMesured)+self.footLPosMesured.cross(self.footLForceMesured)
        xMo=self.footRTorquMesured[0]+self.footLTorquMesured[0]+vectR[0]+vectL[0]
        yMo=self.footRTorquMesured[1]+self.footLTorquMesured[1]+vectR[1]+vectL[1]
        zMo=self.footRTorquMesured[2]+self.footLTorquMesured[2]+vectR[2]+vectL[2]
        
        xank=self.comPosMesured[0]
        yank=self.comPosMesured[1]
        zank=self.comPosMesured[2]
        
      
      a=normalRot[0]
      b=normalRot[1]
      c=normalRot[2]
      d=normalScalA
         
      xz=-(b*xFo*xMo+b*yFo*yMo+d*xFo*zFo+b*xFo*yank*zFo-b*xank*yFo*zFo+c*yMo*zFo+c*xFo*zank*zFo-c*xank*(zFo**2))/(zFo*(a*xFo+b*yFo+c*zFo))
      yz=-(-a*xFo*xMo-a*yFo*yMo-c*xMo*zFo-a*xFo*yank*zFo+d*yFo*zFo+a*xank*yFo*zFo+c*yFo*zank*zFo-c*yank*(zFo**2))/(zFo*(a*xFo+b*yFo+c*zFo))
      zz=-(b*xMo-a*yMo-a*xFo*zank-b*yFo*zank+d*zFo+a*xank*zFo+b*yank*zFo)/(a*xFo+b*yFo+c*zFo)
      
      return Vector3d(xz,yz,zz)    
  
  def compute_convexhull_vertices(self,footR,footL,footRAngl,footLAngl):

    ##foot R and L borders, used in SSP
#    xvL = [footL[0]+(self.fronttoankle-self.sole_margin),
#            footL[0]+(self.fronttoankle-self.sole_margin),
#            footL[0]-(self.backtoankle-self.sole_margin),
#            footL[0]-(self.backtoankle-self.sole_margin)
#            ]
#    yvL = [footL[1]-(self.inttoankle-self.sole_margin),
#            footL[1]+(self.exttoankle-self.sole_margin),
#            footL[1]+(self.exttoankle-self.sole_margin),
#            footL[1]-(self.inttoankle-self.sole_margin)
#            ]
    xvL = [footL[0]+(self.fronttoankle-self.sole_margin)*cos(footLAngl)-sin(footLAngl)*(-(self.inttoankle-self.sole_margin)),
            footL[0]+(self.fronttoankle-self.sole_margin)*cos(footLAngl)-sin(footLAngl)*(+(self.exttoankle-self.sole_margin)),
            footL[0]-(self.backtoankle-self.sole_margin)*cos(footLAngl)-sin(footLAngl)*(+(self.exttoankle-self.sole_margin)),
            footL[0]-(self.backtoankle-self.sole_margin)*cos(footLAngl)-sin(footLAngl)*(-(self.inttoankle-self.sole_margin))
            ]
    yvL = [footL[1]-(self.inttoankle-self.sole_margin)*cos(footLAngl)+sin(footLAngl)*(+(self.fronttoankle-self.sole_margin)),
            footL[1]+(self.exttoankle-self.sole_margin)*cos(footLAngl)+sin(footLAngl)*(+(self.fronttoankle-self.sole_margin)),
            footL[1]+(self.exttoankle-self.sole_margin)*cos(footLAngl)+sin(footLAngl)*(-(self.backtoankle-self.sole_margin)),
            footL[1]-(self.inttoankle-self.sole_margin)*cos(footLAngl)+sin(footLAngl)*(-(self.backtoankle-self.sole_margin))
            ]
    xvL.append(xvL[0])
    yvL.append(yvL[0])
    
#    xvL = [+(self.fronttoankle-self.sole_margin),
#            +(self.fronttoankle-self.sole_margin),
#            -(self.backtoankle-self.sole_margin),
#            -(self.backtoankle-self.sole_margin)
#            ]
#    yvL = [-(self.inttoankle-self.sole_margin),
#            +(self.exttoankle-self.sole_margin),
#            +(self.exttoankle-self.sole_margin),
#            -(self.inttoankle-self.sole_margin)
#            ]
#    xvL.append(xvL[0])
#    yvL.append(yvL[0])
#    zvL=np.ones(len(xvL))*(-0.093)
#    
#    for i in range(len(xvL)):
#      toto=self.footLOriMesured.inverse()*Vector3d(xvL[i],yvL[i],zvL[i])
#      xvL[i]=toto[0]+self.footLPosMesured[0]
#      yvL[i]=toto[1]+self.footLPosMesured[1]
#      zvL[i]=toto[2]+self.footLPosMesured[2]
    
    
    
    xvR = [footR[0]-(self.backtoankle-self.sole_margin)*cos(footRAngl)-sin(footLAngl)*(+(self.inttoankle-self.sole_margin)),
            footR[0]-(self.backtoankle-self.sole_margin)*cos(footRAngl)-sin(footLAngl)*(-(self.exttoankle-self.sole_margin)),
            footR[0]+(self.fronttoankle-self.sole_margin)*cos(footRAngl)-sin(footLAngl)*(-(self.exttoankle-self.sole_margin)),
            footR[0]+(self.fronttoankle-self.sole_margin)*cos(footRAngl)-sin(footLAngl)*(+(self.inttoankle-self.sole_margin)),
            ];
    yvR = [footR[1]+(self.inttoankle-self.sole_margin)*cos(footRAngl)+sin(footLAngl)*(-(self.backtoankle-self.sole_margin)),
            footR[1]-(self.exttoankle-self.sole_margin)*cos(footRAngl)+sin(footLAngl)*(-(self.backtoankle-self.sole_margin)),
            footR[1]-(self.exttoankle-self.sole_margin)*cos(footRAngl)+sin(footLAngl)*(+(self.fronttoankle-self.sole_margin)),
            footR[1]+(self.inttoankle-self.sole_margin)*cos(footRAngl)+sin(footLAngl)*(+(self.fronttoankle-self.sole_margin)),
            ];
    xvR.append(xvR[0])
    yvR.append(yvR[0])

#    xvR = [-(self.backtoankle-self.sole_margin),
#            -(self.backtoankle-self.sole_margin),
#            +(self.fronttoankle-self.sole_margin),
#            +(self.fronttoankle-self.sole_margin),
#            ]
#    yvR = [+(self.inttoankle-self.sole_margin),
#            -(self.exttoankle-self.sole_margin),
#            -(self.exttoankle-self.sole_margin),
#            +(self.inttoankle-self.sole_margin),
#            ]
#    xvR.append(xvR[0])
#    yvR.append(yvR[0])
#    zvR=np.ones(len(xvR))*(-0.093)
#    
#    for i in range(len(xvR)):
#      toto=self.footROriMesured.inverse()*Vector3d(xvR[i],yvR[i],zvR[i])
#      xvR[i]=toto[0]+self.footRPosMesured[0]
#      yvR[i]=toto[1]+self.footRPosMesured[1]
#      zvR[i]=toto[2]+self.footRPosMesured[2]
    
    ##foot R and L intersection with perpendicular to ankle segment.
    a=[footL[0]-footR[0],footL[1]-footR[1]]
    a_ortho=[-a[1],a[0]]
  
    [xvL1,yvL1,vL1]=projection_convex(footL[0]+10*a_ortho[0],footL[1]+10*a_ortho[1],
                                    footL[0],footL[1],
                                    xvL,yvL)
    [xvL2,yvL2,vL2]=projection_convex(footL[0]-10*a_ortho[0],footL[1]-10*a_ortho[1],
                                    footL[0],footL[1],
                                    xvL,yvL)
                                
    [xvR1,yvR1,vR1]=projection_convex(footR[0]+10*a_ortho[0],footR[1]+10*a_ortho[1],
                                    footR[0],footR[1],
                                    xvR,yvR)
    [xvR2,yvR2,vR2]=projection_convex(footR[0]-10*a_ortho[0],footR[1]-10*a_ortho[1],
                                    footR[0],footR[1],
                                    xvR,yvR)
    
    if vL1>vL2:
        xuL=xvL2
        xlL=xvL1
    
        yuL=yvL2
        ylL=yvL1
    else:
        xuL=xvL1
        xlL=xvL2
    
        yuL=yvL1
        ylL=yvL2
    
    if vL1==1 or vL2==1:
      xvL_reduced={2:[xuL]+xvL[1:-3]+[xlL],
                      3:[xuL]+xvL[1:-2]+[xlL],
                      4:[xuL]+xvL[1:-1]+[xlL]}[vL1*vL2]
      yvL_reduced={2:[yuL]+yvL[1:-3]+[ylL],
                      3:[yuL]+yvL[1:-2]+[ylL],
                      4:[yuL]+yvL[1:-1]+[ylL]}[vL1*vL2]
                      
    elif vL1==2 or vL2==2:
      xvL_reduced={6:[xuL]+xvL[2:-2]+[xlL],
                      8:[xuL]+xvL[2:-1]+[xlL]}[vL1*vL2]
      yvL_reduced={6:[yuL]+yvL[2:-2]+[ylL],
                      8:[yuL]+yvL[2:-1]+[ylL]}[vL1*vL2]
    else:
        xvL_reduced=[xuL]+xvL[3:-1]+[xlL]
        yvL_reduced=[yuL]+yvL[3:-1]+[ylL]
    
    
    if vR1<vR2:
        xuR=xvR2
        xlR=xvR1
    
        yuR=yvR2
        ylR=yvR1
    else:
        xuR=xvR1
        xlR=xvR2
    
        yuR=yvR1
        ylR=yvR2
  
    
    
    if vR1==1 or vR2==1:
      xvR_reduced={2:[xuR]+xvR[1:-3]+[xlR],
                      3:[xuR]+xvR[1:-2]+[xlR],
                      4:[xuR]+xvR[1:-1]+[xlR]}[vR1*vR2]
      yvR_reduced={2:[yuR]+yvR[1:-3]+[ylR],
                      3:[yuR]+yvR[1:-2]+[ylR],
                      4:[yuR]+yvR[1:-1]+[ylR]}[vR1*vR2]
                      
    elif vR1==2 or vR2==2:
      xvR_reduced={6:[xuR]+xvR[2:-2]+[xlR],
                      8:[xuR]+xvR[2:-1]+[xlR]}[vR1*vR2]
      yvR_reduced={6:[yuR]+yvR[2:-2]+[ylR],
                      8:[yuR]+yvR[2:-1]+[ylR]}[vR1*vR2]
    else:
        xvR_reduced=[xuR]+xvR[3:-1]+[xlR]
        yvR_reduced=[yuR]+yvR[3:-1]+[ylR]
        
    xv=[xlR,xuR,xuL,xlL];
    yv=[ylR,yuR,yuL,ylL];
    xv.append(xv[0])
    yv.append(yv[0])
    
    return [xv,yv,xvR_reduced,yvR_reduced,xvL_reduced,yvL_reduced,xvR,yvR,xvL,yvL]