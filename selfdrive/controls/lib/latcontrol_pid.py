from selfdrive.controls.lib.pid import PIController
from selfdrive.controls.lib.drive_helpers import get_steer_max
from cereal import car
from cereal import log
from selfdrive.kegman_conf import kegman_conf
from common.numpy_fast import interp
import common.log as  trace1

from selfdrive.config import Conversions as CV

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
    self.steer_Kp = [0.18,0.25]
    self.steer_Ki = [0.01,0.05]

    self.steerKf = [ 0.000005 ]
    self.pid_change_flag = 0
    self.pre_pid_change_flag = 0


    self.movAvg = moveavg1.MoveAvg()

  def reset(self):
    self.pid.reset()
    
  def live_tune(self, CP, path_plan):
    self.mpc_frame += 1
    if self.mpc_frame % 600 == 0:
      # live tuning through /data/openpilot/tune.py overrides interface.py settings
      self.kegman = kegman_conf()
      if self.kegman.conf['tuneGernby'] == "1":
        self.steerKf = float(self.kegman.conf['Kf'])

        self.BP0 = float(self.kegman.conf['sR_BP0'])
        self.steer_Kp = [ float(self.kegman.conf['Kp']), float(self.kegman.conf['sR_Kp']) ]
        self.steer_Ki = [ float(self.kegman.conf['Ki']), float(self.kegman.conf['sR_Ki']) ]

        self.deadzone = float(self.kegman.conf['deadzone'])
        self.mpc_frame = 0 
        self.pid_change_flag = 1


    if self.pid_change_flag == 0:
      pass
    elif abs(path_plan.angleSteers) > self.BP0:
      self.steerKpV = [ self.steer_Kp[1] ]
      self.steerKiV = [ self.steer_Ki[1] ]
      self.pid_change_flag = 1
    else:
      self.steerKpV = [ self.steer_Kp[0] ]
      self.steerKiV = [ self.steer_Ki[0] ]
      self.pid_change_flag = 2

    if self.pid_change_flag != self.pre_pid_change_flag:
      self.pre_pid_change_flag = self.pid_change_flag
      self.pid = PIController((CP.lateralTuning.pid.kpBP, self.steerKpV),
                          (CP.lateralTuning.pid.kiBP, self.steerKiV),
                          k_f=self.steerKf, pos_limit=1.0)



        
    

  def update(self, active, v_ego, angle_steers, angle_steers_rate, eps_torque, steer_override, rate_limited, CP, path_plan):

    self.live_tune(CP, path_plan)
 
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

    delta = self.angle_steers_des - path_plan.angleSteers
    #trace1.printf( 'pid steer:{:.1f} dst:{:.1f} delta={:.1f}'.format( self.angle_steers_des, path_plan.angleSteers ) )

    return output_steer, float(self.angle_steers_des), pid_log
