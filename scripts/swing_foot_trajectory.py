from eigen3 import Vector3d
import spacevecalg as sva

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from tasks_helper import toVecX, surfaceState

# class that generates a 3rd order polynomial trajectory
class trajectory_poly3(object):
  def __init__(self, init_pos, init_vel, final_pos, final_vel, total_time):
    # trajectory parameters
    self.init_pos = init_pos
    self.init_vel = init_vel
    self.final_pos = final_pos
    self.final_vel = final_vel
    self.total_time = total_time

    # polynomial coefficients
    self.computePolynomial()

  def getPosition(self, current_time):
    position = self.poly_a*current_time**3 + self.poly_b*current_time**2 + self.poly_c*current_time + self.poly_d;
    return position

  def getVelocity(self, current_time):
    velocity = 3*self.poly_a*current_time**2 + 2*self.poly_b*current_time + self.poly_c;
    return velocity

  def getAcceleration(self, current_time):
    acceleration = 6*self.poly_a*current_time + 2*self.poly_b;
    return acceleration

  def computePolynomial(self):
    self.poly_a = (self.init_vel*self.total_time + 2*self.init_pos + self.final_vel*self.total_time - 2*self.final_pos) / self.total_time**3
    self.poly_b = -(2*self.init_vel*self.total_time + 3*self.init_pos + self.final_vel*self.total_time - 3*self.final_pos) / self.total_time**2
    self.poly_c = self.init_vel
    self.poly_d = self.init_pos

  def setNewFinalPosition(self, final_pos):
    self.final_pos = final_pos
    self.computePolynomial()

# class that generates the complete swing foot trajectory
class swing_foot_traj(object):
  def __init__(self, init_pos_x, init_pos_y, final_pos_x, final_pos_y, \
               init_theta, final_theta, step_height, total_time):
    self.TrajX = trajectory_poly3(init_pos_x, 0., final_pos_x, 0., total_time)
    self.TrajY = trajectory_poly3(init_pos_y, 0., final_pos_y, 0., total_time)

    self.TrajZ_up = trajectory_poly3(0., 0., step_height, 0., total_time/2.)
    self.TrajZ_down = trajectory_poly3(step_height, 0., 0., 0., total_time/2.)

    # TODO: maybe another type of trajectory can be better
    self.TrajTheta = trajectory_poly3(init_theta, 0., final_theta, 0., total_time)

    self.total_time = total_time
    self.half_total_time = self.total_time/2.

  def getTranslation(self, current_time):
    if current_time <= self.half_total_time:
      z = self.TrajZ_up.getPosition(current_time)
    else:
      z = self.TrajZ_down.getPosition(current_time - self.half_total_time)
    x = self.TrajX.getPosition(current_time)
    y = self.TrajY.getPosition(current_time)
    return Vector3d(x,y,z)

  def getVelocity(self, current_time):
    if current_time <= self.half_total_time:
      z = self.TrajZ_up.getVelocity(current_time)
    else:
      z = self.TrajZ_down.getVelocity(current_time - self.half_total_time)
    x = self.TrajX.getVelocity(current_time)
    y = self.TrajY.getVelocity(current_time)
    return Vector3d(x,y,z)

  def getAcceleration(self, current_time):
    if current_time <= self.half_total_time:
      z = self.TrajZ_up.getAcceleration(current_time)
    else:
      z = self.TrajZ_down.getAcceleration(current_time - self.half_total_time)
    x = self.TrajX.getAcceleration(current_time)
    y = self.TrajY.getAcceleration(current_time)
    return Vector3d(x,y,z)

  def getOrientation(self, current_time):
    thetaZ = self.TrajTheta.getPosition(current_time)
    return sva.RotZ(thetaZ)

# class to help the abstraction with JorisQP Tasks
class swing_foot_task_helper(object):
  def __init__(self, swingFootPosTask, swingFootOriTask,
               final_pos_x, final_pos_y,
               init_theta, final_theta,
               step_height, total_time, time_step=5e-3):

    self.swingFootPosTask = swingFootPosTask
    self.swingFootOriTask = swingFootOriTask

    # set initial position
    pos = swingFootPosTask.position()
    self.sft = swing_foot_traj(pos[0], pos[1], final_pos_x, final_pos_y,
                               init_theta, final_theta, step_height, total_time)

    # handles time stepping
    self.total_time = total_time
    self.time_step = time_step
    self.current_time = 0.
    self.isDone = False

  def update(self):
    if not self.isDone:
      self.swingFootPosTask.position(self.sft.getTranslation(self.current_time))
      self.swingFootOriTask.orientation(self.sft.getOrientation(self.current_time))
      if (self.current_time > self.total_time):
        self.isDone = True
      else:
        self.current_time += self.time_step

  def setNewFootLandingPosition(self, final_pos_x, final_pos_y):
    self.sft.TrajX.setNewFinalPosition(final_pos_x)
    self.sft.TrajY.setNewFinalPosition(final_pos_y)

# class to help the abstraction with JorisQP Tasks, utilizes the tracking task
class swing_foot_tracktask_helper(object):
  def __init__(self, robot, footSurface,
               swingFootPosTask, swingFootPosTaskTr, swingFootOriTask,
               final_pos_x, final_pos_y,
               init_theta, final_theta,
               step_height, total_time, time_step=5e-3):

    self.swingFootPosTask = swingFootPosTask
    self.swingFootPosTaskTr = swingFootPosTaskTr
    self.swingFootOriTask = swingFootOriTask

    self.surfState = surfaceState(robot, footSurface)
    pos = self.surfState.getPosW()
    # set initial position
    self.sft = swing_foot_traj(pos[0], pos[1], final_pos_x, final_pos_y,
                               init_theta, final_theta, step_height, total_time)

    # handles time stepping
    self.total_time = total_time
    self.time_step = time_step
    self.current_time = 0.
    self.isDone = False

  def update(self, robot, footSurface):
    if not self.isDone:
      self.swingFootPosTask.position(self.sft.getTranslation(self.current_time))
      self.swingFootPosTaskTr.refVel(toVecX(self.sft.getVelocity(self.current_time)))
      self.swingFootPosTaskTr.refAccel(toVecX(self.sft.getAcceleration(self.current_time)))

      # TODO: orientation tracking?
      self.swingFootOriTask.orientation(self.sft.getOrientation(self.current_time))
      if (self.current_time > self.total_time):
        self.isDone = True
      else:
        self.current_time += self.time_step

  def setNewFootLandingPosition(self, final_pos_x, final_pos_y):
    self.sft.TrajX.setNewFinalPosition(final_pos_x)
    self.sft.TrajY.setNewFinalPosition(final_pos_y)

# class to visualize the planned swing foot trajectory in RViZ, #TODO: visualize rotations
class swing_foot_visualizer(object):
  def __init__(self, topic_name):
    self.mArray = MarkerArray()
    self.ros_publisher = rospy.Publisher(topic_name, MarkerArray)
    self.traj = None

  # create a set of markers that symbolize the trajectory at each projected time instance
  def createFromTrajectory(self, swing_foot_traj, numMarkers):
    self.mArray.markers[:] = []
    timeStep = swing_foot_traj.total_time / numMarkers
    for mID in range(0, numMarkers):
      marker = Marker()
      marker.header.frame_id = 'map'
      marker.id = mID
      marker.type = marker.SPHERE
      marker.action = marker.ADD

      marker.pose.orientation.x = 0.0
      marker.pose.orientation.y = 0.0
      marker.pose.orientation.z = 0.0
      marker.pose.orientation.w = 1.0

      marker.scale.x = .015
      marker.scale.y = .015
      marker.scale.z = .015

      marker.color.r = 0.0
      marker.color.g = 0.0
      marker.color.b = 1.0
      marker.color.a = 1.0

      # set marker position to trajectory position at the timeStep
      time = mID*timeStep
      marker.pose.position.x = swing_foot_traj.TrajX.getPosition(time)
      marker.pose.position.y = swing_foot_traj.TrajY.getPosition(time)
      if mID < (numMarkers/2):
        marker.pose.position.z = swing_foot_traj.TrajZ_up.getPosition(time)
      else:
        time = (mID - (numMarkers/2))*timeStep
        marker.pose.position.z = swing_foot_traj.TrajZ_down.getPosition(time)

      self.mArray.markers.append(marker)

  def publish(self):
    self.ros_publisher.publish(self.mArray)
