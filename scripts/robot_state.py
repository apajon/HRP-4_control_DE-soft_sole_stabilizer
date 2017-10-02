# TODO: this is a copy of robot_state from mc_control, need to handle the import better
from eigen3 import Quaterniond
import spacevecalg as sva
import rbdyn as rbd

from mc_rbdyn import rbdList


def qFromRobotState(robot, robot_state):
  q = rbdList(robot.mbc.q)
  fillQFromRobotState(robot, q, robot_state)
  return q


def fillQFromRobotState(robot, q, robot_state):
  for n, p in zip(robot_state.joint_name, robot_state.joint_position):
    if robot.hasJoint(n):
      index = robot.jointIndexByName(n)
      q[index][0] = p


def XAccFromRobotState(robot_state):
  E_0_acc = Quaterniond(robot_state.imu_orientation.w,
                        robot_state.imu_orientation.x,
                        robot_state.imu_orientation.y,
                        robot_state.imu_orientation.z).inverse()

  return sva.PTransform(E_0_acc)



def fillMbcFromRobotState(robot, mbc, robot_state):
  q = qFromRobotState(robot, robot_state)
  q[0] = list(robot.mb.joint(0).zeroParam())
  X_0_acc_esti = XAccFromRobotState(robot_state)
  mbc.q = q
  rbd.forwardKinematics(robot.mb, mbc)

  bodyPosW = list(mbc.bodyPosW)
  X_0_b0_old = bodyPosW[0]
  accelBody = robot.accelerometerBody
  X_0_acc = bodyPosW[robot.bodyIndexByName(accelBody)]
  X_b0_acc = X_0_acc*X_0_b0_old.inv()
  X_0_b0_new = X_b0_acc.inv()*X_0_acc_esti
  quat_ff = Quaterniond(X_0_b0_new.rotation()).inverse()

  q[0][0:4] = [quat_ff.w(), quat_ff.x(), quat_ff.y(), quat_ff.z()]
  mbc.q = q


class RobotStateReader(object):
  def __init__(self, robot):
    self.robot = robot

    # we create the sub multibody from freeflier to Accelerometer
    accelBody = robot.accelerometerBody
    accelId = robot.bodyIdByName(accelBody)
    self.accelMb = rbd.Jacobian(robot.mb, accelId).subMultiBody(robot.mb)
    self.accelMbc = rbd.MultiBodyConfig(self.accelMb)
    self.accelMbc.zero(self.accelMb)
    self.accelQ = rbdList(self.accelMbc.q)
    self.accelJoints = [(i, j.name()) for i, j in enumerate(self.accelMb.joints()) if j.dof() == 1]


  def fillQ(self, q, robot_state):
    # apply robot_state q to accelMb
    for i, jn in self.accelJoints:
      index = robot_state.joint_name.index(jn)
      self.accelQ[i][0] = robot_state.joint_position[index]
    self.accelQ[0] = list(self.accelMb.joint(0).zeroParam())
    self.accelMbc.q = self.accelQ
    rbd.forwardKinematics(self.accelMb, self.accelMbc)

    # compute accelerometer to ff transform
    X_0_acc_esti = XAccFromRobotState(robot_state)
    accelBodyPosW = list(self.accelMbc.bodyPosW)
    X_0_b0_old = accelBodyPosW[0]
    X_0_acc = accelBodyPosW[-1]
    X_b0_acc = X_0_acc*X_0_b0_old.inv()
    X_0_b0_new = X_b0_acc.inv()*X_0_acc_esti
    quat_ff = Quaterniond(X_0_b0_new.rotation()).inverse()

    fillQFromRobotState(self.robot, q, robot_state)
    q[0][0:4] = [quat_ff.w(), quat_ff.x(), quat_ff.y(), quat_ff.z()]
    return q
