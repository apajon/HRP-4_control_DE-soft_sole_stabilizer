#! /usr/bin/env python
import rospy
from tcp_ros_bridge_msgs.msg import Stabilizer as StMsg
from mc_ros_utils import transform
import rbdyn as rbd
import spacevecalg as sva

class stabilizerMsg(object):
  def __init__(self, robot):
    self.robot = robot
    # create and initialize the ROS message and publisher
    self.stabMsg = StMsg()
    self.stabMsg.activated = True
    self.stabMsg.zmp.x = 0.
    self.stabMsg.zmp.y = 0.
    self.stabMsg.zmp.z = 0.
    self.rosPub = rospy.Publisher('/robot/controls/stabilizer', StMsg, queue_size=None)

    self.X_0_baselink = None
    self.zmp_world = None
    self.X_0_zmp = None
    self.X_zmp = None

  def publishZMPDesired(self, curTime, zmp_desired):
    # fill the stabilizer message properly
    self.stabMsg.header.stamp = curTime
    self.X_0_baselink = list(self.robot.mbc.bodyPosW)[0]
    self.stabMsg.base_link = transform.toTransform(self.X_0_baselink)
    self.X_0_zmp = sva.PTransform(zmp_desired)
    self.X_zmp = self.X_0_zmp * self.X_0_baselink.inv()
    self.stabMsg.zmp.x = self.X_zmp.translation().x()
    self.stabMsg.zmp.y = self.X_zmp.translation().y()
    self.stabMsg.zmp.z = self.X_zmp.translation().z()

    self.rosPub.publish(self.stabMsg)

  def publish(self, curTime, comPosition, comAccel):
    # fill the stabilizer message properly
    self.stabMsg.header.stamp = curTime
    self.X_0_baselink = list(self.robot.mbc.bodyPosW)[0]
    self.stabMsg.base_link = transform.toTransform(self.X_0_baselink)
    self.zmp_world = rbd.computeCentroidalZMP(self.robot.mbc, comPosition, comAccel, 0.)
    self.X_0_zmp = sva.PTransform(self.zmp_world)
    self.X_zmp = self.X_0_zmp * self.X_0_baselink.inv()
    self.stabMsg.zmp.x = self.X_zmp.translation().x()
    self.stabMsg.zmp.y = self.X_zmp.translation().y()
    self.stabMsg.zmp.z = self.X_zmp.translation().z()

    self.rosPub.publish(self.stabMsg)