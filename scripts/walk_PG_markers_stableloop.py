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
    self.zmpPosWanted_marker = Marker()
    self.zmpPosWantedSatur_marker = Marker()
    self.comPosMesured_marker = Marker()
    self.comVelMesured_marker = Marker()
    self.comAccMesured_marker = Marker()
    self.zmpPosDesired_marker = Marker()
    self.zmpPosMesured_marker = Marker()
    self.zmpRPosMesured_marker = Marker()
    self.zmpLPosMesured_marker = Marker()
    self.zmpRPosDesired_marker = Marker()
    self.zmpLPosDesired_marker = Marker()
    self.zmpRPosWantedSatur_marker = Marker()
    self.zmpLPosWantedSatur_marker = Marker()
    self.footRPosMesured_marker_ = Marker()
    self.footLPosMesured_marker_ = Marker()
    self.ROriMarkerMesured_marker = Marker()
    self.LOriMarkerMesured_marker = Marker()
    self.ROriMarkerWanted_marker = Marker()
    self.LOriMarkerWanted_marker = Marker()
    self.zmpPosMesured_marker_ = Marker()
    self.zmpRPosMesured_marker_ = Marker()
    self.zmpLPosMesured_marker_ = Marker()
    self.comPosMesured_marker_ = Marker()
    self.DeltaDisplR_marker_ = Marker()
    self.DeltaDisplL_marker_ = Marker()
    self.comPosDesired_marker = Marker()
    self.footRForceMesured_marker_ = Marker()
    self.footLForceMesured_marker_ = Marker()
    self.DeltaAnglR_marker_ = Marker()
    self.DeltaAnglL_marker_ = Marker()
    self.zmpRForceWanted_marker_ = Marker()
    self.zmpLForceWanted_marker_ = Marker()
    self.comVelMesured_marker_ = Marker()
    self.comVelDesired_marker = Marker()
    self.capturePointMesured_marker_ = Marker()
    self.zmpRPosWanted_marker_ = Marker()
    self.zmpLPosWanted_marker_ = Marker()
    self.footRTorquMesured_marker_ = Marker()
    self.footLTorquMesured_marker_ = Marker()
    self.int_delta_comPos_marker = Marker()

    self.markerList = [self.ZMP_actual_marker,
                       self.ZMP_target_marker,
                       self.CoM_actual_marker,
                       self.CoM_target_marker,
                       self.foot_target_marker,
                       self.CapturePoint_marker,
                       self.zmpPosWanted_marker,
                       self.zmpPosWantedSatur_marker,
                       self.comPosMesured_marker,
                       self.comVelMesured_marker,
                       self.comAccMesured_marker,
                       self.zmpPosDesired_marker,
                       self.zmpPosMesured_marker,
                       self.zmpRPosMesured_marker,
                       self.zmpLPosMesured_marker,
                       self.zmpRPosDesired_marker,
                       self.zmpLPosDesired_marker,
                       self.zmpRPosWantedSatur_marker,
                       self.zmpLPosWantedSatur_marker,
                       self.footRPosMesured_marker_,
                       self.footLPosMesured_marker_,
                       self.ROriMarkerMesured_marker,
                       self.LOriMarkerMesured_marker,
                       self.ROriMarkerWanted_marker,
                       self.LOriMarkerWanted_marker,
                       self.zmpPosMesured_marker_,
                       self.zmpRPosMesured_marker_,
                       self.zmpLPosMesured_marker_,
                       self.comPosMesured_marker_,
                       self.DeltaDisplR_marker_,
                       self.DeltaDisplL_marker_,
                       self.comPosDesired_marker,
                       self.footRForceMesured_marker_,
                       self.footLForceMesured_marker_,
                       self.DeltaAnglR_marker_,
                       self.DeltaAnglL_marker_,
                       self.zmpRForceWanted_marker_,
                       self.zmpLForceWanted_marker_,
                       self.comVelMesured_marker_,
                       self.comVelDesired_marker,
                       self.capturePointMesured_marker_,
                       self.zmpRPosWanted_marker_,
                       self.zmpLPosWanted_marker_,
                       self.footRTorquMesured_marker_,
                       self.footLTorquMesured_marker_,
                       self.int_delta_comPos_marker]
                       
    self.markerNameList = ['ZMP_actual',
                       'ZMP_target',
                       'CoM_actual',
                       'CoM_target',
                       'foot_target',
                       'CapturePoint',
                       'zmpPosWanted',
                       'zmpPosWantedSatur',
                       'comPosMesured',
                       'comVelMesured',
                       'comAccMesured',
                       'zmpPosDesired',
                       'zmpPosMesured',
                       'zmpRPosMesured',
                       'zmpLPosMesured',
                       'zmpRPosDesired',
                       'zmpLPosDesired',
                       'zmpRPosWantedSatur',
                       'zmpLPosWantedSatur',
                       'footRPosMesured_',
                       'footLPosMesured_',
                       'ROriMarkerMesured',
                       'LOriMarkerMesured',
                       'ROriMarkerWanted',
                       'LOriMarkerWanted',
                       'zmpPosMesured_',
                       'zmpRPosMesured_',
                       'zmpLPosMesured_',
                       'comPosMesured_',
                       'DeltaDisplR_',
                       'DeltaDisplL_',
                       'comPosDesired',
                       'footRForceMesured_',
                       'footLForceMesured_',
                       'DeltaAnglR_',
                       'DeltaAnglL_',
                       'zmpRForceWanted_',
                       'zmpLForceWanted_',
                       'comVelMesured_',
                       'comVelDesired',
                       'capturePointMesured_',
                       'zmpRPosWanted_',
                       'zmpLPosWanted_',
                       'footRTorquMesured_',
                       'footLTorquMesured_',
                       'int_delta_comPos']

    # Initialize marker settings
    self.init_colors()
    self.markerID_count = 0
    self.colors = [self.red, self.green, self.red, self.green, self.blue, self.yellow]
    self.frames = ['map', 'map', 'map', 'map', 'map', 'map']
    for i in xrange(6,len(self.markerList)):  
      self.colors.append(self.blue)      
      self.frames.append('map')
    self.colors[6]=self.red
    self.colors[7]=self.red
    self.colors[11]=self.green
    self.colors[15]=self.green
    self.colors[16]=self.green
    self.colors[17]=self.yellow
    self.colors[18]=self.yellow
    map(self.init_marker, self.markerList, self.colors, self.frames, self.markerNameList)

    self.ros_publisher = rospy.Publisher(topic_name, MarkerArray)
    self.mArray = MarkerArray()

  def init_marker(self, marker, color, frame, markerName):
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
    
    if self.markerID_count>6:
      marker.color.a= 0
    else:
      marker.color.a = 1.0
    
    if self.markerID_count in [1,2,5,6]:
      marker.color.a = 0.0
    if self.markerID_count in [7, 8, 9, 12, 13, 14, 15, 16, 17, 18, 19, 26, 27, 28]:
      marker.color.a = 1.0
      
    marker.text=markerName

  def fill(self, ZMP_actual, ZMP_target, CoM_actual, CoM_target, foot_target, CapturePoint,
           zmpPosWanted, zmpPosWantedSatur, comPosMesured, comVelMesured, comAccMesured,
           zmpPosDesired, zmpPosMesured, 
           zmpRPosMesured, zmpLPosMesured, zmpRPosDesired, zmpLPosDesired, zmpRPosWantedSatur, zmpLPosWantedSatur,
           footRPosMesured_, footLPosMesured_,
           ROriMarkerMesured,LOriMarkerMesured,ROriMarkerWanted,LOriMarkerWanted,
           zmpPosMesured_,zmpRPosMesured_, zmpLPosMesured_, comPosMesured_,
           DeltaDisplR_marker_, DeltaDisplL_marker_,
           comPosDesired,
           footRForceMesured_,footLForceMesured_,
           DeltaAnglR_,DeltaAnglL_,
           zmpRForceWanted_,zmpLForceWanted_,
           comVelMesured_,comVelDesired,
           capturePointMesured_,
           zmpRPosWanted_,zmpLPosWanted_,
           footRTorquMesured_,footLTorquMesured_,
           int_delta_comPos):
    map(self.fillPosition, self.markerList, \
        [ZMP_actual, ZMP_target, CoM_actual, CoM_target, foot_target, CapturePoint, 
         zmpPosWanted, zmpPosWantedSatur, comPosMesured, comVelMesured, comAccMesured,
         zmpPosDesired, zmpPosMesured, 
         zmpRPosMesured, zmpLPosMesured, zmpRPosDesired, zmpLPosDesired, zmpRPosWantedSatur, zmpLPosWantedSatur,
         footRPosMesured_, footLPosMesured_,
         ROriMarkerMesured,LOriMarkerMesured,ROriMarkerWanted,LOriMarkerWanted,
         zmpPosMesured_,zmpRPosMesured_, zmpLPosMesured_, comPosMesured_,
         DeltaDisplR_marker_, DeltaDisplL_marker_,
         comPosDesired,
         footRForceMesured_,footLForceMesured_,
         DeltaAnglR_,DeltaAnglL_,
         zmpRForceWanted_,zmpLForceWanted_,
         comVelMesured_,comVelDesired,
         capturePointMesured_,
         zmpRPosWanted_,zmpLPosWanted_,
         footRTorquMesured_,footLTorquMesured_,
         int_delta_comPos])

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