import humoto
from swing_foot_trajectory import swing_foot_tracktask_helper, swing_foot_visualizer
from hri_scenario.srv import setCoMVelDes, stopWalk

from eigen3 import Vector2d, Vector3d, Matrix3d
import spacevecalg as sva

import numpy as np
import rospy
import copy
from tasks_helper import toVecX

'''
This class handles bridging with the HuMoTo library for walking tasks
'''
class HumotoBridge(object):
  def __init__(self, robot, rFoot, lFoot, timeStep):
    self.robot = robot
    # foot surfaces
    self.rFoot = rFoot
    self.lFoot = lFoot
    self.swingFoot = None

    # starting pose of the walking pattern generator expressed in the QP world frame
    self.X_0_humoto0 = sva.interpolate(rFoot.X_0_s(robot), lFoot.X_0_s(robot))
    self.X_0_trans = self.X_0_humoto0.translation()
    # TODO: this is an approximation, find a better way
    self.X_0_angle = -sva.rotationError(self.X_0_humoto0.rotation(), Matrix3d.Identity(), 1e-7)[2]
    print 'init trans is ', self.X_0_trans
    print 'init angle is ', self.X_0_angle

    # humoto wPG parameters
    self.walk_options = humoto.WalkOptions()
    self.robot_params = humoto.RobotParameters()
    self.mpc_params = humoto.MPCParameters()

    # time synch handling
    self.timeStep = timeStep
    self.walk_phase_tick = 0
    self.recomputeParams()

    # initializations for the walk FSM
    self.swingFootMover = None
    self.wPG_iters = 1
    self.stateType = None
    self.previousStateType = None

    # foot pose
    self.nextStepPos = [0., 0.]
    self.last_rotation_angle_lfoot = copy.copy(self.X_0_angle)
    self.last_rotation_angle_rfoot = copy.copy(self.X_0_angle)

    # visualizer
    self.foot_traj_viz = swing_foot_visualizer('swing_foot_trajectory')

    # data for stabilizer
    self.zmp_des = Vector2d.Zero()

    # CoM Target tracking variables
    self.comRefPos = Vector3d().Zero()
    self.comRefVel = Vector3d().Zero()
    self.comRefAccel = Vector3d().Zero()

    # ROS services to control the walk
    # TODO: safety to make sure the class is not recreated otherwise ROS crashes with launching the same service
    self.comVelServ = rospy.Service('com_velocity_des', setCoMVelDes, self.setCoMVelocity)
    self.stopServ = rospy.Service('stop_walk', stopWalk, self.stopWalk)

    # allows other code to check contact changes
    self.isContactChange = False

  '''
  used to transform the humoto {0} frame to the QP's {0} frame
  # TODO: cleaner way to handle all 3
  '''
  def transToWorld(self, x, y):
    ctheta = np.cos(self.X_0_angle)# TODO: maybe move outside if still needed
    stheta = np.sin(self.X_0_angle)
    new_x = x*ctheta - y*stheta + self.X_0_trans[0]
    new_y = x*stheta + y*ctheta + self.X_0_trans[1]
    return new_x, new_y

  def motionToWorld(self, x, y):
    ctheta = np.cos(self.X_0_angle)# TODO: maybe move outside if still needed
    stheta = np.sin(self.X_0_angle)
    new_x = x*ctheta - y*stheta
    new_y = x*stheta + y*ctheta
    return new_x, new_y

  def rotToWorld(self, angle):
    return angle + self.X_0_angle

  '''
  recompute internal parms that are dependent on humoto params
  this needs to be called if humoto parameters were updated
  '''
  def recomputeParams(self):
    self.walk_phase_tick_max = self.mpc_params.sampling_time_ms_/(self.timeStep*1000)

    # The time (0.5*self.mpc_params.sampling_time_ms_) is given to allow the tracking task to converge
    # Since the landing position is updated at every wPG iteration, a max of sampling_time_ms_ is allowed
    self.swing_time = (self.walk_options.ss_duration_ms_ - (0.5*self.mpc_params.sampling_time_ms_))/ 1000.

  '''
  creates the Walking Pattern Generator
  params changes aren't taken into account anymore after this
  '''
  def createWPG(self, comTarget, graspContacts):
    self.comRefPos = comTarget
    self.comRefVel = Vector3d().Zero()
    self.comRefAccel = Vector3d().Zero()
    self.graspContacts = graspContacts
    self.robot_params.com_height_ = comTarget[2]
    self.wPG = humoto.pybindWPG01(self.robot_params, self.walk_options, self.mpc_params)

  '''
  handles rosservice request to change the CoM velocity
  '''
  def setCoMVelocity(self, req):
    self.wPG.setCoMVelocity(Vector2d(req.CoMVelX, req.CoMVelY))
    self.wPG.setThetaIncrement(req.theta_inc)
    return True

  '''
  handles rosservice request to stop the walking pattern generator
  '''
  def stopWalk(self, req):
    if req.stopWalk:
      if not self.wPG.getStatus():
        self.wPG.stopWalking()
        print 'stopping the walk--------------------------------------'
        self.comVelServ.shutdown('walk stopped, stopping com velocity service')
        self.stopServ.shutdown('walk stopped, stopping the walk stop service')
        return True
      else:
        print 'walk already stopped'
        return False
    else:
      return False

  '''
  one iteration of the Walking Pattern Generator
  '''
  def callWPG(self, qpsolver, comTask, comTaskTr, torsoOriTask,
              rfPosTask, rfPosTaskTr, lfPosTask, lfPosTaskTr,
              rfOriTask, lfOriTask, c1L, c1R):
    self.isContactChange = False
    if self.walk_phase_tick > 1:
      self.walk_phase_tick -= 1

      # Euler integration considering constant acceleration
      self.comRefPos += self.comRefVel*self.timeStep + 0.5*self.timeStep*self.timeStep*self.comRefAccel
      self.comRefVel += self.comRefAccel*self.timeStep

      comTask.com(self.comRefPos)
      comTaskTr.refVel(toVecX(self.comRefVel))
      comTaskTr.refAccel(toVecX(self.comRefAccel))

#      TODO: ZMP des goes out of footstep bounds.
      self.zmp_des += self.zmp_vel*self.timeStep

    else:
      if not self.wPG.getStatus():
        print '\n ========== wPG iteration ', self.wPG_iters
        self.wPG.iterateOnce()
        self.wPG_iters += 1

        # update state with walking pattern generator result
        cx_state = self.wPG.getComStateX()
        cy_state = self.wPG.getComStateY()

        posx, posy = self.transToWorld(cx_state[0], cy_state[0])
        velx, vely = self.motionToWorld(cx_state[1], cy_state[1])
        accx, accy = self.motionToWorld(cx_state[2], cy_state[2])

        self.comRefPos = Vector3d(posx, posy, self.robot_params.com_height_)
        self.comRefVel = Vector3d(velx, vely, 0.)
        self.comRefAccel = Vector3d(accx, accy, 0.)

        comTask.com(self.comRefPos)
        comTaskTr.refVel(toVecX(self.comRefVel))
        comTaskTr.refAccel(toVecX(self.comRefAccel))

        wpgCop = self.wPG.getCoPState()
        zmp_desX, zmp_desY = self.transToWorld(wpgCop[0], wpgCop[1])
        self.zmp_des = Vector2d(zmp_desX, zmp_desY)

        wpgCopVel = self.wPG.getCoPVelocityState()
        zmp_velx, zmp_vely = self.motionToWorld(wpgCopVel[0], wpgCopVel[1])
        self.zmp_vel = Vector2d(zmp_velx, zmp_vely)

        self.stateType = self.wPG.getStateType()

        # prevents a bug caused by the orientation task dimWeight #TODO: fix properly in Tasks
        torsoOriTask.orientation(sva.RotZ((self.last_rotation_angle_rfoot + self.last_rotation_angle_lfoot)/2.))

#          print 'angle: ', self.wPG.getRotationAngle()
        # Properly sets the last swing foot landing position,
        # TODO: last foot rotation is still incremented, maybe it should not?
        if self.wPG.getIsLastSS():
          if self.stateType == humoto.STATE_RSS:
            finalLandingPos = (self.rFoot.X_b_s*list(self.robot.mbc.bodyPosW)[self.robot.bodyIndexByName(self.rFoot.bodyName)]).translation() + \
              sva.RotZ(self.last_rotation_angle_rfoot).transpose() *  Vector3d(0., self.robot_params.feet_dist_default_, 0)
            self.nextStepPos[0] = finalLandingPos[0]
            self.nextStepPos[1] = finalLandingPos[1]
          elif self.stateType == humoto.STATE_LSS:
            finalLandingPos = (self.lFoot.X_b_s*list(self.robot.mbc.bodyPosW)[self.robot.bodyIndexByName(self.lFoot.bodyName)]).translation() + \
              sva.RotZ(self.last_rotation_angle_lfoot).transpose() * Vector3d(0., -self.robot_params.feet_dist_default_, 0)
            self.nextStepPos[0] = finalLandingPos[0]
            self.nextStepPos[1] = finalLandingPos[1]
          else: # final TDS and DS states
            pass
        else:
          wpgsteps = self.wPG.getNextSupportPosition()
          posX, posY = self.transToWorld(wpgsteps[0], wpgsteps[1])
          self.nextStepPos = [posX, posY]


        # double support
        if self.stateType == humoto.STATE_DS:
          print 'state DS'
          self.walk_phase_tick = self.walk_phase_tick_max
          if not((self.previousStateType == humoto.STATE_DS) or
                 (self.previousStateType == humoto.STATE_TDS)):
            qpsolver.setContacts(self.graspContacts + [c1L, c1R])
            qpsolver.update()
            self.isContactChange = True
            print '------------updating contact state'

        # transitional DS
        if self.stateType == humoto.STATE_TDS:
          print 'state TDS'
          self.walk_phase_tick = self.walk_phase_tick_max
          if not((self.previousStateType == humoto.STATE_DS) or
                 (self.previousStateType == humoto.STATE_TDS)):
            qpsolver.setContacts(self.graspContacts + [c1L, c1R])
            qpsolver.update()
            self.isContactChange = True
            print '------------updating contact state'

        # left single support
        elif self.stateType == humoto.STATE_LSS:
          self.swingFoot = self.rFoot
          print 'state LSS'
          self.walk_phase_tick = self.walk_phase_tick_max
          if not(self.previousStateType == humoto.STATE_LSS):
            qpsolver.setContacts(self.graspContacts + [c1L])
            qpsolver.update()
            self.isContactChange = True
            print '------------updating contact state'
            wpgangle = self.rotToWorld(self.wPG.getRotationAngle())
            self.swingFootMover = swing_foot_tracktask_helper(self.robot, self.swingFoot,
              rfPosTask, rfPosTaskTr, rfOriTask,
              self.nextStepPos[0], self.nextStepPos[1],
              self.last_rotation_angle_rfoot, wpgangle,
              self.robot_params.step_height_, self.swing_time, self.timeStep)
          else:
            self.swingFootMover.setNewFootLandingPosition(self.nextStepPos[0], self.nextStepPos[1])
            self.last_rotation_angle_rfoot = self.rotToWorld(self.wPG.getRotationAngle())
          self.foot_traj_viz.createFromTrajectory(self.swingFootMover.sft, 10)
          self.foot_traj_viz.publish()

        # right single support
        elif self.stateType == humoto.STATE_RSS:
          self.swingFoot = self.lFoot
          print 'state RSS'
          self.walk_phase_tick = self.walk_phase_tick_max
          if not(self.previousStateType == humoto.STATE_RSS):
            qpsolver.setContacts(self.graspContacts + [c1R])
            qpsolver.update()
            self.isContactChange = True
            print '------------updating contact state'
            wpgangle = self.rotToWorld(self.wPG.getRotationAngle())
            self.swingFootMover = swing_foot_tracktask_helper(self.robot, self.swingFoot,
              lfPosTask, lfPosTaskTr, lfOriTask,
              self.nextStepPos[0], self.nextStepPos[1],
              self.last_rotation_angle_lfoot, wpgangle,
              self.robot_params.step_height_, self.swing_time, self.timeStep)
          else:
            self.swingFootMover.setNewFootLandingPosition(self.nextStepPos[0], self.nextStepPos[1])
            self.last_rotation_angle_lfoot = self.rotToWorld(self.wPG.getRotationAngle())
          self.foot_traj_viz.createFromTrajectory(self.swingFootMover.sft, 10)
          self.foot_traj_viz.publish()

        # used to compare if contact state needs updating
        self.previousStateType = self.stateType

      else:
        print 'wPG ended with ', self.wPG_iters - 1, ' iterartions'

    if self.swingFootMover is not None:
      self.swingFootMover.update(self.robot, self.swingFoot)
