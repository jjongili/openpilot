import math
import numpy as np

from cereal import log
import cereal.messaging as messaging


from cereal import log
import cereal.messaging as messaging
from selfdrive.config import Conversions as CV
from selfdrive.controls.lib.planner import calc_cruise_accel_limits
from selfdrive.controls.lib.speed_smoother import speed_smoother
from selfdrive.controls.lib.long_mpc import LongitudinalMpc



from selfdrive.car.hyundai.values import Buttons, SteerLimitParams
from common.numpy_fast import clip, interp

from selfdrive.config import RADAR_TO_CAMERA

import common.log as trace1
import common.CTime1000 as tm
import common.MoveAvg as  moveavg1

MAX_SPEED = 255.0

LON_MPC_STEP = 0.2  # first step is 0.2s
MAX_SPEED_ERROR = 2.0
AWARENESS_DECEL = -0.2     # car smoothly decel at .2m/s^2 when user is distracted

# lookup tables VS speed to determine min and max accels in cruise
# make sure these accelerations are smaller than mpc limits
_A_CRUISE_MIN_V  = [-1.0, -.8, -.67, -.5, -.30]
_A_CRUISE_MIN_BP = [   0., 5.,  10., 20.,  40.]

# need fast accel at very low speed for stop and go
# make sure these accelerations are smaller than mpc limits
_A_CRUISE_MAX_V = [1.2, 1.2, 0.65, .4]
_A_CRUISE_MAX_V_FOLLOWING = [1.6, 1.6, 0.65, .4]
_A_CRUISE_MAX_BP = [0.,  6.4, 22.5, 40.]

# Lookup table for turns
_A_TOTAL_MAX_V = [1.7, 3.2]
_A_TOTAL_MAX_BP = [20., 40.]

# 75th percentile
SPEED_PERCENTILE_IDX = 7


SC = trace1.Loger("spd")


def limit_accel_in_turns(v_ego, angle_steers, a_target, steerRatio , wheelbase):
  """
  This function returns a limited long acceleration allowed, depending on the existing lateral acceleration
  this should avoid accelerating when losing the target in turns
  """

  a_total_max = interp(v_ego, _A_TOTAL_MAX_BP, _A_TOTAL_MAX_V)
  a_y = v_ego**2 * angle_steers * CV.DEG_TO_RAD / (steerRatio * wheelbase)
  a_x_allowed = math.sqrt(max(a_total_max**2 - a_y**2, 0.))

  return [a_target[0], min(a_target[1], a_x_allowed)]


class SpdController():
  def __init__(self):
    self.long_control_state = 0  # initialized to off
    self.long_active_timer = 0
    self.long_wait_timer = 0
    self.long_curv_timer = 0
    self.long_dst_speed = 0


    self.v_acc_start = 0.0
    self.a_acc_start = 0.0
    self.path_x = np.arange(192)

    self.traceSC = trace1.Loger("SPD_CTRL")

    self.wheelbase = 2.845
    self.steerRatio = 12.5  #12.5

    self.v_model = 0
    self.a_model = 0
    self.v_cruise = 0
    self.a_cruise = 0

    self.l_poly = []
    self.r_poly = []

    self.movAvg = moveavg1.MoveAvg()   
    self.Timer1 = tm.CTime1000("SPD")
    self.time_no_lean = 0

  def reset(self):
    self.long_active_timer = 0
    self.v_model = 0
    self.a_model = 0
    self.v_cruise = 0
    self.a_cruise = 0    


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
      model_speed = max(30.0 * CV.MPH_TO_MS, model_speed) # Don't slow down below 20mph

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


  def get_lead(self, sm, CS ):
    lead_msg = sm['model'].lead
    if lead_msg.prob > 0.5:
      dRel = float(lead_msg.dist - RADAR_TO_CAMERA)
      yRel = float(lead_msg.relY)
      vRel = float(lead_msg.relVel)
      vLead = float(CS.v_ego + lead_msg.relVel)
    else:
      dRel = 150
      yRel = 0
      vRel = 0

      #vRel = vRel * CV.MS_TO_KPH

    return dRel, yRel, vRel

  def get_tm_speed( self, CS, set_time ):
    time = set_time

    if CS.VSetDis > CS.clu_Vanz:
      set_speed = int(CS.clu_Vanz)
    else:
      set_speed = int(CS.VSetDis)

    set_speed -= 1

    return time, set_speed

  def update_lead(self, CS ):
    set_speed = CS.cruise_set_speed_kph
    cur_speed = CS.clu_Vanz
    long_wait_timer_cmd = 500    
    dst_lead_distance = 110


    if CS.cruise_set_mode != 2:
      return  long_wait_timer_cmd, set_speed

    #dRel, yRel, vRel = self.get_lead( sm, CS )

    dRel = CS.lead_distance
    vRel = CS.lead_objspd

    spd_to_lead = (cur_speed*0.8)
    if dst_lead_distance > spd_to_lead:
       dst_lead_distance = spd_to_lead

    if dst_lead_distance < 30:
      dst_lead_distance = 30

    if dRel != 150:
      self.time_no_lean = 0

    d_delta = dRel - dst_lead_distance
    lead_objspd = vRel
    # 1. 거리 유지.
    if d_delta < 0:
      
      if lead_objspd >= 0:
        set_speed = int(CS.VSetDis)
      elif lead_objspd < -10:
        long_wait_timer_cmd, set_speed = self.get_tm_speed( CS, 10 )
      elif lead_objspd < -5:
        long_wait_timer_cmd, set_speed = self.get_tm_speed( CS, 50 )
      elif lead_objspd < 0:
        long_wait_timer_cmd, set_speed = self.get_tm_speed( CS, 100 )
    else:
      if lead_objspd < -10:
        long_wait_timer_cmd, set_speed = self.get_tm_speed( CS, 50 )   
      elif lead_objspd < -5:
        long_wait_timer_cmd, set_speed = self.get_tm_speed( CS, 90 )        
      elif lead_objspd < 0:
        long_wait_timer_cmd, set_speed = self.get_tm_speed( CS, 150 )           
      else:
        set_speed = cur_speed + 2
        if CS.VSetDis > set_speed:
          set_speed = CS.VSetDis + 1

        if dRel == 150:
          self.time_no_lean += 1
          if self.time_no_lean < 150:
            long_wait_timer_cmd = 50
          else:
            long_wait_timer_cmd = 20
        elif d_delta < 5:
          long_wait_timer_cmd = 50
        elif d_delta < 10:
          long_wait_timer_cmd = 30
        elif d_delta < 20:
          long_wait_timer_cmd = 20
        else:
          long_wait_timer_cmd = 10

    return  long_wait_timer_cmd, set_speed


  def update(self, v_ego_kph, CS, sm, actuators ):
    btn_type = Buttons.NONE
    #lead_1 = sm['radarState'].leadOne
    long_wait_timer_cmd = 500
    set_speed = CS.cruise_set_speed_kph

    long_wait_timer_cmd, set_speed = self.update_lead( CS )

    model_speed = self.calc_va( sm, CS.v_ego )

    #xp = [0,5,20,40]
    #fp2 = [2,3,4,5]
    #limit_steers = interp( v_ego_kph, xp, fp2 )


    # 2. 커브 감속.
    cuv_dst_speed = set_speed
    if CS.cruise_set_speed_kph >= 70:
      if model_speed < 80:
        cuv_dst_speed = CS.cruise_set_speed_kph - 15
        if long_wait_timer_cmd > 20:
          long_wait_timer_cmd = 20
      elif model_speed < 120:  # 6도
        cuv_dst_speed = CS.cruise_set_speed_kph - 10
        if long_wait_timer_cmd > 100:
          long_wait_timer_cmd = 100
      elif model_speed < 160:  #  3 도
        cuv_dst_speed = CS.cruise_set_speed_kph - 5
        if long_wait_timer_cmd > 150:
          long_wait_timer_cmd = 150

      if set_speed > cuv_dst_speed:
        set_speed = cuv_dst_speed

    if  set_speed > CS.cruise_set_speed_kph:
        set_speed = CS.cruise_set_speed_kph
    

    target_set_speed = set_speed
    delta = int(set_speed) - int(CS.VSetDis)
    if abs(delta) <= 1:
      long_wait_timer_cmd = 200

    if self.long_wait_timer:
      self.long_wait_timer -= 1      
      if self.long_wait_timer > long_wait_timer_cmd:
        self.long_wait_timer = long_wait_timer_cmd
    elif delta <= -1:
      set_speed = CS.VSetDis - 1
      btn_type = Buttons.SET_DECEL
      self.long_wait_timer = long_wait_timer_cmd
      self.long_dst_speed = set_speed   
    elif  delta >= 1 and model_speed > 200:
      set_speed = CS.VSetDis + 1
      btn_type = Buttons.RES_ACCEL
      self.long_wait_timer = long_wait_timer_cmd
      self.long_dst_speed = set_speed 


    if CS.cruise_set_mode == 0:
       btn_type = Buttons.NONE

    tm_sample = self.Timer1.sampleTime()
    str3 = 'curvature={:3.0f} dest={:3.0f}/{:3.0f} heart={:.0f} '.format( model_speed,  target_set_speed, self.long_wait_timer,  tm_sample )
    trace1.printf2(  str3 )
    #SC.add( str3 )

    #if CS.pcm_acc_status and CS.AVM_Popup_Msg == 1 and CS.VSetDis > 30  and CS.lead_distance < 90:
      #str2 = 'btn={:.0f} btn_type={}  v{:.5f} a{:.5f}  v{:.5f} a{:.5f}'.format(  CS.AVM_View, btn_type, self.v_model, self.a_model, self.v_cruise, self.a_cruise )
     # self.traceSC.add( 'v_ego={:.1f} angle={:.1f}  {} {} {}'.format( v_ego_kph, CS.angle_steers, str1, str2, str3 )  ) 

    return btn_type, set_speed, model_speed
    #return btn_type, set_speed, model_speed