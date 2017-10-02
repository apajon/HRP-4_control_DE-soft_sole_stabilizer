import rospy
from visualization_msgs.msg import Marker, MarkerArray
import rbdyn as rbd
import numpy as np

# visualization for important points of a walking pattern generator
class WalkPGMarkers(object):
  def __init__(self, topic_name):
    self.ZMP_actual_marker = Marker()
    self.ZMP_target_marker = Marker()
    self.CoM_actual_marker = Marker()
    self.CoM_target_marker = Marker()
    self.foot_target_marker = Marker()
    self.CapturePoint_marker = Marker()

    self.markerList = [self.ZMP_actual_marker,
                       self.ZMP_target_marker,
                       self.CoM_actual_marker,
                       self.CoM_target_marker,
                       self.foot_target_marker,
                       self.CapturePoint_marker]

    # Initialize marker settings
    self.init_colors()
    self.markerID_count = 0
    self.colors = [self.red, self.green, self.red, self.green, self.blue, self.yellow]
    self.frames = ['map', 'map', 'map', 'map', 'map', 'map']
    map(self.init_marker, self.markerList, self.colors, self.frames)

    self.ros_publisher = rospy.Publisher(topic_name, MarkerArray)
    self.mArray = MarkerArray()

  def init_marker(self, marker, color, frame):
    marker.header.frame_id = frame
    marker.id = self.markerID_count
    self.markerID_count += 1
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD

    marker.pose.position.x = 0.0
    marker.pose.position.y = 0.0
    marker.pose.position.z = 0.0

    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0

    marker.scale.x = .05
    marker.scale.y = .05
    marker.scale.z = .05

    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]

  def fill(self, ZMP_actual, ZMP_target, CoM_actual, CoM_target, foot_target, CapturePoint):
    map(self.fillPosition, self.markerList, \
        [ZMP_actual, ZMP_target, CoM_actual, CoM_target, foot_target, CapturePoint])

  '''
  helper method that computes the data (if needed) and fills the markers
  Note: make sure rbd.forwardAcceleration is called before this method
  '''
  def computeAndFill(self, mb, mbc, zmp_des, com_des, footStep_des):
    # update Center of Mass state
    comPos = rbd.computeCoM(mb, mbc)
    comVel = rbd.computeCoMVelocity(mb, mbc)
    comAcc = rbd.computeCoMAcceleration(mb, mbc)

    # angular frequency of the Linearized Inverted Pendulum Model
    omega = np.sqrt(9.81/comPos[2])

    #  Zero Moment Point and Capture Point
    zmp = rbd.computeCentroidalZMP(mbc, comPos, comAcc, 0.)
    capturePoint = comPos + (comVel/omega)
    capturePoint[2] = 0.0
    
    delta_corr=zmp

    # fill the markers
    self.fill(zmp, zmp_des,
              comPos, com_des,
              footStep_des,
              capturePoint,
              delta_corr)

  def fillPosition(self, marker, position):
    marker.pose.position.x = position[0]
    marker.pose.position.y = position[1]
    marker.pose.position.z = position[2]

  def publish(self):
    self.mArray.markers[:] = []
    map(self.mArray.markers.append, self.markerList)
    self.ros_publisher.publish(self.mArray)

  def init_colors(self):
    self.red = [1.0, 0.0, 0.0]
    self.green = [0.0, 1.0, 0.0]
    self.blue= [0.0, 0.0, 1.0]
    self.yellow = [1.0, 0.5, 0.0]