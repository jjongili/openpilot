import os
import math
import common.log as trace1


from common.realtime import sec_since_boot, DT_MDL
from selfdrive.swaglog import cloudlog
from selfdrive.controls.lib.lateral_mpc import libmpc_py
from selfdrive.controls.lib.drive_helpers import MPC_COST_LAT
from selfdrive.controls.lib.lane_planner import LanePlanner
from selfdrive.kegman_conf import kegman_conf
from selfdrive.config import Conversions as CV
from common.params import Params
from common.numpy_fast import interp
from cereal import log

import cereal.messaging as messaging

import common.MoveAvg as  moveavg1


LaneChangeState = log.PathPlan.LaneChangeState
LaneChangeDirection = log.PathPlan.LaneChangeDirection
LaneChangeBSM = log.PathPlan.LaneChangeBSM

LOG_MPC = os.environ.get('LOG_MPC', True)

tracePP = trace1.Loger("pathPlanner")

DESIRES = {
  LaneChangeDirection.none: {
    LaneChangeState.off: log.PathPlan.Desire.none,
    LaneChangeState.preLaneChange: log.PathPlan.Desire.none,
    LaneChangeState.laneChangeStarting: log.PathPlan.Desire.none,
    LaneChangeState.laneChangeFinishing: log.PathPlan.Desire.none,
  },
  LaneChangeDirection.left: {
    LaneChangeState.off: log.PathPlan.Desire.none,
    LaneChangeState.preLaneChange: log.PathPlan.Desire.none,
    LaneChangeState.laneChangeStarting: log.PathPlan.Desire.laneChangeLeft,
    LaneChangeState.laneChangeFinishing: log.PathPlan.Desire.laneChangeLeft,
  },
  LaneChangeDirection.right: {
    LaneChangeState.off: log.PathPlan.Desire.none,
    LaneChangeState.preLaneChange: log.PathPlan.Desire.none,
    LaneChangeState.laneChangeStarting: log.PathPlan.Desire.laneChangeRight,
    LaneChangeState.laneChangeFinishing: log.PathPlan.Desire.laneChangeRight,
  },
}

def calc_states_after_delay(states, v_ego, steer_angle, curvature_factor, steer_ratio, delay):
  states[0].x = v_ego * delay
  states[0].psi = v_ego * curvature_factor * math.radians(steer_angle) / steer_ratio * delay
  return states


class PathPlanner():
  def __init__(self, CP):
      self.LP = LanePlanner()

      self.last_cloudlog_t = 0
      self.steer_rate_cost = CP.steerRateCost

      self.setup_mpc()
      self.solution_invalid_cnt = 0
      self.lane_change_enabled = Params().get('LaneChangeEnabled') == b'1'
      self.path_offset_i = 0.0

      self.mpc_frame = 400
      self.lane_change_ll_prob = 1.0
      self.sR_delay_counter = 0
      self.steerRatio_new = 0.0
      self.steerAngle_new = 0.0
      self.sR_time = 1
      self.nCommand = 0

      kegman = kegman_conf(CP)
      if kegman.conf['steerRatio'] == "-1":
        self.steerRatio = CP.steerRatio
      else:
        self.steerRatio = float(kegman.conf['steerRatio'])

      if kegman.conf['steerRateCost'] == "-1":
        self.steerRateCost = CP.steerRateCost
      else:
        self.steerRateCost = float(kegman.conf['steerRateCost'])

      self.sR = [float(kegman.conf['steerRatio']), (float(kegman.conf['steerRatio']) + float(kegman.conf['sR_boost']))]
      self.sRBP = [float(kegman.conf['sR_BP0']), float(kegman.conf['sR_BP1'])]

      self.steerRateCost_prev = self.steerRateCost
      self.setup_mpc()

      self.lane_change_state = LaneChangeState.off
      self.lane_change_direction = LaneChangeDirection.none
      self.lane_change_timer1 = 0
      self.lane_change_timer2 = 0
      self.lane_change_timer3 = 0
      self.lane_change_BSM = LaneChangeBSM.off

      self.movAvg = moveavg1.MoveAvg()



  def limit_ctrl(self, value, limit, offset ):
      p_limit = offset + limit
      m_limit = offset - limit
      if value > p_limit:
          value = p_limit
      elif  value < m_limit:
          value = m_limit
      return value

  def setup_mpc(self):
      self.libmpc = libmpc_py.libmpc
      self.libmpc.init(MPC_COST_LAT.PATH, MPC_COST_LAT.LANE, MPC_COST_LAT.HEADING, self.steer_rate_cost)

      self.mpc_solution = libmpc_py.ffi.new("log_t *")
      self.cur_state = libmpc_py.ffi.new("state_t *")
      self.cur_state[0].x = 0.0
      self.cur_state[0].y = 0.0
      self.cur_state[0].psi = 0.0
      self.cur_state[0].delta = 0.0

      self.angle_steers_des = 0.0
      self.angle_steers_des_mpc = 0.0
      self.angle_steers_des_prev = 0.0
      self.angle_steers_des_time = 0.0


  def lane_change_fun( self, sm, lca_left, lca_right, lane_change_prob ):
      if self.nCommand == 0:
          self.lane_change_timer1 = 0
          self.lane_change_timer2 = 0
          self.lane_change_timer3 = 0
          self.lane_change_timer4 = 0
          self.lane_change_state = LaneChangeState.off
          self.lane_change_direction = LaneChangeDirection.none
          self.nCommand=1

      elif self.nCommand == 1:
          one_blinker = sm['carState'].leftBlinker != sm['carState'].rightBlinker
          if not one_blinker:
              pass
          elif sm['carState'].leftBlinker and not lca_left:
              self.lane_change_direction = LaneChangeDirection.left
          elif sm['carState'].rightBlinker  and not lca_right:
              self.lane_change_direction = LaneChangeDirection.right
          else:
              self.lane_change_direction = LaneChangeDirection.none

          self.lane_change_state = LaneChangeState.off
          if self.lane_change_direction != LaneChangeDirection.none:
              self.lane_change_BSM = LaneChangeBSM.off
              self.lane_change_state = LaneChangeState.preLaneChange
              self.nCommand=2

      elif self.nCommand == 2:   # preLaneChange
          torque_applied = False        
          if not sm['carState'].steeringPressed:
              pass
          elif self.lane_change_direction == LaneChangeDirection.left:
              if lca_left:  # BSM
                self.lane_change_BSM = LaneChangeBSM.left
                self.nCommand=5  # cancel
              else:            
                torque_applied = sm['carState'].steeringTorque > 0
          elif self.lane_change_direction == LaneChangeDirection.right:
              if lca_right:  # BSM
                self.lane_change_BSM = LaneChangeBSM.right
                self.nCommand=5   # cancel         
              else:
                torque_applied = sm['carState'].steeringTorque < 0

          if torque_applied:
              self.lane_change_timer2 = 0
              self.lane_change_ll_prob = 1.0
              self.lane_change_state = LaneChangeState.laneChangeStarting
              self.nCommand=3

      elif self.nCommand == 3:   # laneChangeStarting
          cancel_applied = False
          if self.lane_change_direction == LaneChangeDirection.left:
            if sm['carState'].rightBlinker:
              cancel_applied = True
            else:
              cancel_applied = sm['carState'].steeringTorque < -50
          elif self.lane_change_direction == LaneChangeDirection.right:
            if sm['carState'].leftBlinker:
              cancel_applied = True
            else:
              cancel_applied = sm['carState'].steeringTorque > 50

          self.lane_change_timer2 += 1
          if cancel_applied:
            self.nCommand=5  # cancel
          else:
            # fade out lanelines over 1s
            self.lane_change_ll_prob = max(self.lane_change_ll_prob - DT_MDL, 0.0)
            # 98% certainty
            if lane_change_prob < 0.02 and self.lane_change_ll_prob < 0.01 or self.lane_change_timer2 > 200:              
              self.lane_change_state = LaneChangeState.laneChangeFinishing
              self.nCommand=4


      elif self.nCommand == 4:   # laneChangeFinishing
          if sm['carState'].leftBlinker or sm['carState'].rightBlinker:
            pass
          else:
            # fade in laneline over 1s
            self.lane_change_ll_prob = min(self.lane_change_ll_prob + DT_MDL, 1.0)
            if self.lane_change_ll_prob > 0.99:
              self.lane_change_state = LaneChangeState.off
              self.nCommand=0

      elif self.nCommand == 5:  # cancel
          self.lane_change_timer4 += 1
          if self.lane_change_timer4 > 500:
              self.nCommand = 0


          self.lane_change_state = LaneChangeState.off
          self.lane_change_direction = LaneChangeDirection.none
          one_blinker = sm['carState'].leftBlinker != sm['carState'].rightBlinker
          if not one_blinker:
              self.nCommand = 0





  def update(self, sm, pm, CP, VM):
    v_ego = sm['carState'].vEgo
    angle_steers = sm['carState'].steeringAngle
    active = sm['controlsState'].active

    angle_offset = sm['liveParameters'].angleOffset

    lca_left = sm['carState'].lcaLeft
    lca_right = sm['carState'].lcaRight

    v_ego_kph = v_ego * CV.MS_TO_KPH

    lean_offset = 0

    # Run MPC
    self.angle_steers_des_prev = self.angle_steers_des_mpc
    VM.update_params(sm['liveParameters'].stiffnessFactor, sm['liveParameters'].steerRatio)
    curvature_factor = VM.curvature_factor(v_ego)

    # Get steerRatio and steerRateCost from kegman.json every x seconds
    self.mpc_frame += 1
    if self.mpc_frame % 500 == 0:
      # live tuning through /data/openpilot/tune.py overrides interface.py settings
      kegman = kegman_conf()
      if kegman.conf['tuneGernby'] == "1":
        self.steerRateCost = float(kegman.conf['steerRateCost'])
        if self.steerRateCost != self.steerRateCost_prev:
          self.setup_mpc()
          self.steerRateCost_prev = self.steerRateCost

        self.sR = [float(kegman.conf['steerRatio']), (float(kegman.conf['steerRatio']) + float(kegman.conf['sR_boost']))]
        self.sRBP = [float(kegman.conf['sR_BP0']), float(kegman.conf['sR_BP1'])]
        self.sR_time = int(float(kegman.conf['sR_time']) * 100.)

      self.mpc_frame = 0


    abs_angle_steers = abs(angle_steers)
    if v_ego_kph < 10:
      self.steerRatio = self.sR[0] * 0.8
    elif self.lane_change_state != LaneChangeState.off:
      self.steerRatio = self.sR[0]
      self.steerAngle_new = 0
    elif v_ego_kph > 40:  # 11.111:
      # boost steerRatio by boost amount if desired steer angle is high
      self.steerRatio_new = interp( abs_angle_steers, self.sRBP, self.sR)

      self.sR_delay_counter += 1
      delta_angle = abs_angle_steers - self.steerAngle_new
      if delta_angle > 2.0 and self.sR_delay_counter > 5:
          self.sR_delay_counter += 20

      if self.sR_delay_counter < self.sR_time:
        if self.steerRatio_new > self.steerRatio:
          self.steerRatio = self.steerRatio_new
          self.steerAngle_new = abs_angle_steers
      else:
        self.steerRatio = (self.steerRatio_new + self.steerRatio) * 0.5
        self.sR_delay_counter = 0
        self.steerAngle_new = 0
    else:
      self.steerRatio = self.sR[0]
      self.steerAngle_new = 0


    #print("steerRatio = ", self.steerRatio)

    self.LP.parse_model(sm['model'])

    # Lane change logic
    below_lane_change_speed = v_ego_kph < 60

    if (not active) or below_lane_change_speed or (self.lane_change_timer1 > 10.0):  # 5 sec
        self.nCommand = 0 
        self.lane_change_state = LaneChangeState.off
        self.lane_change_direction = LaneChangeDirection.none
    else:
        lane_change_prob = self.LP.l_lane_change_prob + self.LP.r_lane_change_prob
        self.lane_change_fun( sm, lca_left, lca_right, lane_change_prob )
        
        if self.lane_change_state in [LaneChangeState.off, LaneChangeState.preLaneChange]:
          self.lane_change_timer1 = 0
        else:
          self.lane_change_timer1 += 0.01

    if self.lane_change_BSM != LaneChangeBSM.off:
      self.lane_change_timer3 += 1
      if self.lane_change_timer3 > 100:
          self.lane_change_timer3 = 0
          self.lane_change_BSM = LaneChangeBSM.off


    desire = DESIRES[self.lane_change_direction][self.lane_change_state]

    # Turn off lanes during lane change
    if desire == log.PathPlan.Desire.laneChangeRight or desire == log.PathPlan.Desire.laneChangeLeft:
      self.LP.l_prob = 0 #self.lane_change_ll_prob
      self.LP.r_prob = 0 #self.lane_change_ll_prob
      self.libmpc.init_weights(MPC_COST_LAT.PATH / 10.0, MPC_COST_LAT.LANE, MPC_COST_LAT.HEADING, self.steer_rate_cost)
    else:
      self.libmpc.init_weights(MPC_COST_LAT.PATH, MPC_COST_LAT.LANE, MPC_COST_LAT.HEADING, self.steer_rate_cost)



    # 차량이 있을 경우 약간 이동하기.
    if lca_left and lca_right:
      lean_offset = 0
    elif lca_left:
      lean_offset = -0.005
    elif lca_right:
      lean_offset = 0.005

    self.LP.update_d_poly( lean_offset )


    # TODO: Check for active, override, and saturation
    # if active:
    #   self.path_offset_i += self.LP.d_poly[3] / (60.0 * 20.0)
    #   self.path_offset_i = clip(self.path_offset_i, -0.5,  0.5)
    #   self.LP.d_poly[3] += self.path_offset_i
    # else:
    #   self.path_offset_i = 0.0

    # account for actuation delay
    self.cur_state = calc_states_after_delay(self.cur_state, v_ego, angle_steers - angle_offset, curvature_factor, self.steerRatio, CP.steerActuatorDelay)

    v_ego_mpc = max(v_ego, 5.0)  # avoid mpc roughness due to low speed
    self.libmpc.run_mpc(self.cur_state, self.mpc_solution,
                        list(self.LP.l_poly), list(self.LP.r_poly), list(self.LP.d_poly),
                        self.LP.l_prob, self.LP.r_prob, curvature_factor, v_ego_mpc, self.LP.lane_width)

    # reset to current steer angle if not active or overriding
    if active:
      delta_desired = self.mpc_solution[0].delta[1]
      rate_desired = math.degrees(self.mpc_solution[0].rate[0] * self.steerRatio)
    else:
      delta_desired = math.radians(angle_steers - angle_offset) / self.steerRatio
      rate_desired = 0.0

    self.cur_state[0].delta = delta_desired
    old_angle_steers_des = self.angle_steers_des_mpc
    org_angle_steers_des = float(math.degrees(delta_desired * self.steerRatio) + angle_offset)
    self.angle_steers_des_mpc = org_angle_steers_des

    if v_ego_kph < 40:
        xp = [0,5,20,40]
        fp2 = [0.2,0.5,1,1.5]
        limit_steers = interp( v_ego_kph, xp, fp2 )
        angle_steers_des = self.limit_ctrl( org_angle_steers_des, limit_steers, angle_steers )
        if v_ego_kph < 10:
            self.angle_steers_des_mpc = self.movAvg.get_data( angle_steers_des, 5 )
        else:
            self.angle_steers_des_mpc = angle_steers_des
    elif self.lane_change_state != LaneChangeState.off:
        self.angle_steers_des_mpc = self.limit_ctrl( self.angle_steers_des_mpc, 3, angle_steers )
    else:
        self.angle_steers_des_mpc = self.limit_ctrl( self.angle_steers_des_mpc, 5, angle_steers )


    #if active:
    #  log_str = 'v_ego={:.1f} cur_steer={:.1f} dst_steer={:.1f} org={:.1f}'.format( v_ego_kph, angle_steers, self.angle_steers_des_mpc, org_angle_steers_des )
    #  tracePP.add( log_str )

    #  Check for infeasable MPC solution
    mpc_nans = any(math.isnan(x) for x in self.mpc_solution[0].delta)
    t = sec_since_boot()
    if mpc_nans:
      self.libmpc.init(MPC_COST_LAT.PATH, MPC_COST_LAT.LANE, MPC_COST_LAT.HEADING, self.steerRateCost)
      self.cur_state[0].delta = math.radians(angle_steers - angle_offset) / self.steerRatio

      if t > self.last_cloudlog_t + 5.0:
        self.last_cloudlog_t = t
        cloudlog.warning("Lateral mpc - nan: True")

    if self.mpc_solution[0].cost > 20000. or mpc_nans:   # TODO: find a better way to detect when MPC did not converge
      self.solution_invalid_cnt += 1
    else:
      self.solution_invalid_cnt = 0
    plan_solution_valid = self.solution_invalid_cnt < 2

    plan_send = messaging.new_message()
    plan_send.init('pathPlan')
    plan_send.valid = sm.all_alive_and_valid(service_list=['carState', 'controlsState', 'liveParameters', 'model'])
    plan_send.pathPlan.laneWidth = float(self.LP.lane_width)
    plan_send.pathPlan.dPoly = [float(x) for x in self.LP.d_poly]
    plan_send.pathPlan.lPoly = [float(x) for x in self.LP.l_poly]
    plan_send.pathPlan.lProb = float(self.LP.l_prob)
    plan_send.pathPlan.rPoly = [float(x) for x in self.LP.r_poly]
    plan_send.pathPlan.rProb = float(self.LP.r_prob)

    plan_send.pathPlan.angleSteers = float(self.angle_steers_des_mpc)
    plan_send.pathPlan.rateSteers = float(rate_desired)
    plan_send.pathPlan.angleOffset = float(sm['liveParameters'].angleOffsetAverage)
    plan_send.pathPlan.mpcSolutionValid = bool(plan_solution_valid)
    plan_send.pathPlan.paramsValid = bool(sm['liveParameters'].valid)
    plan_send.pathPlan.sensorValid = bool(sm['liveParameters'].sensorValid)
    plan_send.pathPlan.posenetValid = bool(sm['liveParameters'].posenetValid)

    plan_send.pathPlan.desire = desire
    plan_send.pathPlan.laneChangeState = self.lane_change_state
    plan_send.pathPlan.laneChangeDirection = self.lane_change_direction
    plan_send.pathPlan.laneChangeBSM = self.lane_change_BSM

    pm.send('pathPlan', plan_send)




    if LOG_MPC:
      dat = messaging.new_message()
      dat.init('liveMpc')
      dat.liveMpc.x = list(self.mpc_solution[0].x)
      dat.liveMpc.y = list(self.mpc_solution[0].y)
      dat.liveMpc.psi = list(self.mpc_solution[0].psi)
      dat.liveMpc.delta = list(self.mpc_solution[0].delta)
      dat.liveMpc.cost = self.mpc_solution[0].cost
      pm.send('liveMpc', dat)
