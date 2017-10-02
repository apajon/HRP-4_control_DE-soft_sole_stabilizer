#! /usr/bin/env python
import tasks
from eigen3 import Vector3d, VectorXd
import spacevecalg as sva

# Helper function for using only a selection of joints in the task
def jointsSelector(robots, index, hl, jointsName):
  r = robots.robots[index]
  jointsId = map(r.jointIdByName, jointsName)
  return tasks.qp.JointsSelector(robots.mbs, index, hl, jointsId)

# Helper function for creating a position set point task
def positionTask(robots, robot_index, bodyName, pos, stiff, w, ef=Vector3d.Zero(), jointsName=[]):
  posTask = tasks.qp.PositionTask(robots.mbs, robot_index,
                                  robots.robots[robot_index].bodyIdByName(bodyName),
                                  pos, ef)
  if len(jointsName) > 0:
    posTaskSel = jointsSelector(robots, robot_index, posTask, jointsName)
    posTaskSp = tasks.qp.SetPointTask(robots.mbs, robot_index, posTaskSel, stiff, w)
    return posTask, posTaskSp, posTaskSel
  posTaskSp = tasks.qp.SetPointTask(robots.mbs, robot_index, posTask, stiff, w)
  return posTask, posTaskSp

# Helper function for creating a position tracking task #TODO: deprecate this
def positionTrackingTask(robots, robot_index, bodyName, pos, gainPos, gainVel, w, ef=Vector3d.Zero()):
  posTask = tasks.qp.PositionTask(robots.mbs, robot_index,
                                  robots.robots[robot_index].bodyIdByName(bodyName),
                                  pos, ef)
  posTaskTr = tasks.qp.TrackingTask(robots.mbs, robot_index, posTask, gainPos, gainVel, w)
  return posTask, posTaskTr

# Helper function for creating a position trajectory tracking task
def positionTrajectoryTask(robots, robot_index, bodyName, pos, gainPos, gainVel, w, ef=Vector3d.Zero()):
  posTask = tasks.qp.PositionTask(robots.mbs, robot_index,
                                  robots.robots[robot_index].bodyIdByName(bodyName),
                                  pos, ef)
  posTaskTr = tasks.qp.TrajectoryTask(robots.mbs, robot_index, posTask, gainPos, gainVel, w)
  return posTask, posTaskTr

# Helper function for creating orientation task in the surface frame
def surfOrientationTask(robots, robot_index, bodyName, ori, stiff, w, X_b_s, jointsName=[]):
  oriTask = tasks.qp.SurfaceOrientationTask(robots.mbs, robot_index,
                                            robots.robots[robot_index].bodyIdByName(bodyName),
                                            ori, X_b_s)
  if len(jointsName) > 0:
    oriTaskSel = jointsSelector(robots, robot_index, oriTask, jointsName)
    oriTaskSp = tasks.qp.SetPointTask(robots.mbs, robot_index, oriTaskSel, stiff, w)
    return oriTask, oriTaskSp, oriTaskSel

  oriTaskSp = tasks.qp.SetPointTask(robots.mbs, robot_index, oriTask, stiff, w)
  return oriTask, oriTaskSp

# Helper function for creating orientation task
def orientationTask(robots, robot_index, bodyName, ori, stiff, w, jointsName=[]):
  oriTask = tasks.qp.OrientationTask(robots.mbs, robot_index,
                                     robots.robots[robot_index].bodyIdByName(bodyName),
                                     ori)
  if len(jointsName) > 0:
    oriTaskSel = jointsSelector(robots, robot_index, oriTask, jointsName)
    oriTaskSp = tasks.qp.SetPointTask(robots.mbs, robot_index, oriTaskSel, stiff, w)
    return oriTask, oriTaskSp, oriTaskSel

  oriTaskSp = tasks.qp.SetPointTask(robots.mbs, robot_index, oriTask, stiff, w)
  return oriTask, oriTaskSp

# Helper function for creating an orientation tracking task #TODO: deprecate this
def orientationTrackingTask(robots, robot_index, bodyName, ori, gainPos, gainVel, w):
  oriTask = tasks.qp.OrientationTask(robots.mbs, robot_index,
                                     robots.robots[robot_index].bodyIdByName(bodyName),
                                     ori)
  oriTaskTr = tasks.qp.TrackingTask(robots.mbs, robot_index, oriTask, gainPos, gainVel, w)
  return oriTask, oriTaskTr

# Helper function for creating an orientation trajectory tracking task
def orientationTrajectoryTask(robots, robot_index, bodyName, ori, gainPos, gainVel, w):
  oriTask = tasks.qp.OrientationTask(robots.mbs, robot_index,
                                     robots.robots[robot_index].bodyIdByName(bodyName),
                                     ori)
  oriTaskTr = tasks.qp.TrajectoryTask(robots.mbs, robot_index, oriTask, gainPos, gainVel, w)
  return oriTask, oriTaskTr

# Helper function for creating a Center of Mass task
def comTask(robots, index, com, stiff, w):
  comTask = tasks.qp.CoMTask(robots.mbs, index, com)
  comTaskSp = tasks.qp.SetPointTask(robots.mbs, index, comTask, stiff, w)
  return comTask, comTaskSp

# Helper function for creating a Center of Mass tracking task #TODO: deprecate this
def comTrackingTask(robots, robot_index, com, gainPos, gainVel, w):
  comTask = tasks.qp.CoMTask(robots.mbs, robot_index, com)
  comTaskTr = tasks.qp.TrackingTask(robots.mbs, robot_index, comTask,
                                    gainPos, gainVel, w)
  return comTask, comTaskTr

# Helper function for creating a Center of Mass tracking task
def comTrajectoryTask(robots, robot_index, com, gainPos, gainVel, w):
  comTask = tasks.qp.CoMTask(robots.mbs, robot_index, com)
  comTaskTr = tasks.qp.TrajectoryTask(robots.mbs, robot_index, comTask,
                                      gainPos, gainVel, w)
  return comTask, comTaskTr

'''
helper function to convert from Eigen Vector3d to VectorXd
'''
def toVecX(vec3):
  vecX = VectorXd(3)
  vecX[0] = vec3[0]
  vecX[1] = vec3[1]
  vecX[2] = vec3[2]
  return vecX

'''
Helper functions for tracking tasks
Allow to obtain the model state expressed in the world frame
'''
class surfaceState(object):
  def __init__(self, robot, surface):
    self.robot = robot
    self.surface = surface
    self.X_b_s = self.surface.X_b_s
    self.bodyIdx = self.robot.bodyIndexByName(self.surface.bodyName)

  def getPosW(self):
    return (self.X_b_s*list(self.robot.mbc.bodyPosW)[self.bodyIdx]).translation()

  def getOriW(self):
    return (self.X_b_s*list(self.robot.mbc.bodyPosW)[self.bodyIdx]).rotation()

  def getVelW(self):
    X_0_b = list(self.robot.mbc.bodyPosW)[self.bodyIdx]
    X_Np_w = sva.PTransformd(X_0_b.rotation().transpose(), self.X_b_s.translation())
    return (X_Np_w*(list(self.robot.mbc.bodyVelB)[self.bodyIdx])).linear()

  def getAngVelW(self):
    X_0_b = list(self.robot.mbc.bodyPosW)[self.bodyIdx]
    X_Np_w = sva.PTransformd(X_0_b.rotation().transpose(), self.X_b_s.translation())
    return (X_Np_w*(list(self.robot.mbc.bodyVelB)[self.bodyIdx])).angular()

class bodyState(object):
  def __init__(self, robot, bodyName):
    self.robot = robot
    self.bodyIdx = self.robot.bodyIndexByName(bodyName)

  def getPosW(self):
    return (list(self.robot.mbc.bodyPosW)[self.bodyIdx]).translation()

  def getOriW(self):
    return (list(self.robot.mbc.bodyPosW)[self.bodyIdx]).rotation()

  def getVelW(self):
    return list(self.robot.mbc.bodyVelW)[self.bodyIdx].linear()

  def getAngVelW(self):
    return list(self.robot.mbc.bodyVelW)[self.bodyIdx].angular()
