import numpy as np

from selfdrive.controls.lib.pid import PIController
from selfdrive.controls.lib.drive_helpers import get_steer_max
from cereal import car
from cereal import log
from selfdrive.kegman_conf import kegman_conf
from common.numpy_fast import interp

import common.log as  trace1
import common.MoveAvg as  moveavg1

from selfdrive.config import Conversions as CV


MAX_SPEED = 255.0

class LatControlPID():
  def __init__(self, CP):
    self.kegman = kegman_conf(CP)
    self.deadzone = float(self.kegman.conf['deadzone'])
    self.pid = PIController((CP.lateralTuning.pid.kpBP, CP.lateralTuning.pid.kpV),
                            (CP.lateralTuning.pid.kiBP, CP.lateralTuning.pid.kiV),
                            k_f=CP.lateralTuning.pid.kf, pos_limit=1.0, sat_limit=CP.steerLimitTimer)
    self.angle_steers_des = 0.
    self.mpc_frame = 500

    self.BP0 = 4
    self.steer_Kf1 = [0.00003,0.00003]    
    self.steer_Ki1 = [0.02,0.03]
    self.steer_Kp1 = [0.18,0.20]

    self.steer_Kf2 = [0.00005,0.00005]
    self.steer_Ki2 = [0.04,0.05]
    self.steer_Kp2 = [0.20,0.25]

    self.pid_change_flag = 0
    self.pre_pid_change_flag = 0
    self.pid_BP0_time = 0

    self.movAvg = moveavg1.MoveAvg()
    self.v_curvature = 256
    self.path_x = np.arange(192)


  
  def calc_va(self, sm, v_ego ):
    md = sm['model']    
    if len(md.path.poly):
      path = list(md.path.poly)

      self.l_poly = np.array(md.leftLane.poly)
      self.r_poly = np.array(md.rightLane.poly)
      self.p_poly = np.array(md.path.poly)


      # Curvature of polynomial https://en.wikipedia.org/wiki/Curvature#Curvature_of_the_graph_of_a_function
      # y = a x^3 + b x^2 + c x + d, y' = 3 a x^2 + 2 b x + c, y'' = 6 a x + 2 b
      # k = y'' / (1 + y'^2)^1.5
      # TODO: compute max speed without using a list of points and without numpy
      y_p = 3 * path[0] * self.path_x**2 + 2 * path[1] * self.path_x + path[2]
      y_pp = 6 * path[0] * self.path_x + 2 * path[1]
      curv = y_pp / (1. + y_p**2)**1.5

      a_y_max = 2.975 - v_ego * 0.0375  # ~1.85 @ 75mph, ~2.6 @ 25mph
      v_curvature = np.sqrt(a_y_max / np.clip(np.abs(curv), 1e-4, None))
      model_speed = np.min(v_curvature)
      model_speed = max(30.0 * CV.KPH_TO_MS, model_speed) # Don't slow down below 20mph

      model_speed = model_speed * CV.MS_TO_KPH
      if model_speed > MAX_SPEED:
          model_speed = MAX_SPEED
    else:
      model_speed = MAX_SPEED

    #following = lead_1.status and lead_1.dRel < 45.0 and lead_1.vLeadK > v_ego and lead_1.aLeadK > 0.0

    #following = CS.lead_distance < 100.0
    #accel_limits = [float(x) for x in calc_cruise_accel_limits(v_ego, following)]
    #jerk_limits = [min(-0.1, accel_limits[0]), max(0.1, accel_limits[1])]  # TODO: make a separate lookup for jerk tuning
    #accel_limits_turns = limit_accel_in_turns(v_ego, CS.angle_steers, accel_limits, self.steerRatio, self.wheelbase )

    model_speed = self.movAvg.get_min( model_speed, 10 )
    return model_speed

  def update_state( self, sm, CS ):
    self.v_curvature = self.calc_va( sm, CS.vEgo )



  def reset(self):
    self.pid.reset()
    
  def live_tune(self, CP, path_plan, v_ego):
    self.mpc_frame += 1
    if self.mpc_frame % 600 == 0:
      # live tuning through /data/openpilot/tune.py overrides interface.py settings
      self.kegman = kegman_conf()
      if self.kegman.conf['tuneGernby'] == "1":
        self.steerKf = float(self.kegman.conf['Kf'])

        self.BP0 = float(self.kegman.conf['sR_BP0'])
        self.steer_Kp1 = [ float(self.kegman.conf['Kp']), float(self.kegman.conf['sR_Kp']) ]
        self.steer_Ki1 = [ float(self.kegman.conf['Ki']), float(self.kegman.conf['sR_Ki']) ]
        self.steer_Kf1 = [ float(self.kegman.conf['Kf']), float(self.kegman.conf['sR_Kf']) ]

        self.steer_Kp2 = [ float(self.kegman.conf['Kp2']), float(self.kegman.conf['sR_Kp2']) ]
        self.steer_Ki2 = [ float(self.kegman.conf['Ki2']), float(self.kegman.conf['sR_Ki2']) ]
        self.steer_Kf2 = [ float(self.kegman.conf['Kf2']), float(self.kegman.conf['sR_Kf2']) ]        

        self.deadzone = float(self.kegman.conf['deadzone'])
        self.mpc_frame = 0 
        if not self.pid_change_flag:
          self.pid_change_flag = 1


    kBP0 = 0
    if self.pid_change_flag == 0:
      pass
    elif abs(path_plan.angleSteers) > self.BP0  or self.v_curvature < 200:
      kBP0 = 1
      self.pid_change_flag = 2

      ##
      self.pid_BP0_time = 300
    elif self.pid_BP0_time:
      kBP0 = 1
      self.pid_BP0_time -= 1
    else:
      kBP0 = 0
      self.pid_change_flag = 3


    self.steerKpV = [ float(self.steer_Kp1[ kBP0 ]), float(self.steer_Kp2[ kBP0 ]) ]
    self.steerKiV = [ float(self.steer_Ki1[ kBP0 ]), float(self.steer_Ki2[ kBP0 ]) ]

    xp = CP.lateralTuning.pid.kpBP
    fp = [float(self.steer_Kf1[ kBP0 ]), float(self.steer_Kf2[ kBP0 ]) ]
    self.steerKf = interp( v_ego,  xp, fp )

    if self.pid_change_flag != self.pre_pid_change_flag:
      self.pre_pid_change_flag = self.pid_change_flag
      self.pid = PIController((CP.lateralTuning.pid.kpBP, self.steerKpV),
                              (CP.lateralTuning.pid.kiBP, self.steerKiV),
                               k_f=self.steerKf, pos_limit=1.0)



        
    

  def update(self, active, v_ego, angle_steers, angle_steers_rate, eps_torque, steer_override, rate_limited, CP, path_plan):

    self.live_tune(CP, path_plan, v_ego)
 
    pid_log = log.ControlsState.LateralPIDState.new_message()
    pid_log.steerAngle = float(angle_steers)
    pid_log.steerRate = float(angle_steers_rate)



    if v_ego < 0.3 or not active:
      output_steer = 0.0
      pid_log.active = False
      #self.angle_steers_des = 0.0
      self.pid.reset()
      self.angle_steers_des = path_plan.angleSteers
    else:
      self.angle_steers_des = path_plan.angleSteers

      

      steers_max = get_steer_max(CP, v_ego)
      self.pid.pos_limit = steers_max
      self.pid.neg_limit = -steers_max
      steer_feedforward = self.angle_steers_des   # feedforward desired angle


      if CP.steerControlType == car.CarParams.SteerControlType.torque:
        # TODO: feedforward something based on path_plan.rateSteers
        steer_feedforward -= path_plan.angleOffset   # subtract the offset, since it does not contribute to resistive torque
        steer_feedforward *= v_ego**2  # proportional to realigning tire momentum (~ lateral accel)
      
      if abs(self.angle_steers_des) > self.BP0:
        deadzone = 0
      else:
        deadzone = self.deadzone

      check_saturation = (v_ego > 10) and not rate_limited and not steer_override
      output_steer = self.pid.update(self.angle_steers_des, angle_steers, check_saturation=check_saturation, override=steer_override,
                                     feedforward=steer_feedforward, speed=v_ego, deadzone=deadzone)
      pid_log.active = True
      pid_log.p = self.pid.p
      pid_log.i = self.pid.i
      pid_log.f = self.pid.f
      pid_log.output = output_steer
      pid_log.saturated = bool(self.pid.saturated)


    return output_steer, float(self.angle_steers_des), pid_log