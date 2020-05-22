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

    self.SC = trace1.Loger("spd")

    

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

  def get_tm_speed( self, CS, set_time, add_val ):
    time = set_time

    delta_speed = CS.VSetDis - CS.clu_Vanz

    set_speed = int(CS.VSetDis) + add_val

    if add_val > 0:  # 증가
      if delta_speed > 5:
        time = 250
    else:
      if delta_speed < -5:
        time = 250

    return time, set_speed

  def update_lead(self, CS,  dRel, yRel, vRel ):
    lead_set_speed = CS.cruise_set_speed_kph
    lead_wait_cmd = 300

    if CS.cruise_set_mode != 2:
      return  lead_wait_cmd, lead_set_speed

    #dRel, yRel, vRel = self.get_lead( sm, CS )
    if CS.lead_distance != 150:
      dRel = CS.lead_distance
      vRel = CS.lead_objspd

    dst_lead_distance = (CS.clu_Vanz*0.6)


#    if dst_lead_distance < 30:
#      dst_lead_distance = 30

    if dRel != 150:
      self.time_no_lean = 0
      d_delta = dRel - dst_lead_distance
      lead_objspd = vRel  # 선행차량 상대속도.
    else:
      d_delta = 0
      lead_objspd = 0

    # 가속이후 속도 설정.
    if CS.driverAcc_time:
      lead_set_speed = CS.clu_Vanz
      lead_wait_cmd = 25
      str3 = 'driver acc speed={:3.0f} time={:3.0f}'.format( lead_set_speed, lead_wait_cmd )
    # 1. 거리 유지.
    elif d_delta < 0:
    # 선행 차량이 가까이 있으면.
      if lead_objspd >= 0:
        lead_set_speed = int(CS.VSetDis)
      elif lead_objspd < -10:
        lead_wait_cmd, lead_set_speed = self.get_tm_speed( CS, 50, -1 )
      elif lead_objspd < -5:
        lead_wait_cmd, lead_set_speed = self.get_tm_speed( CS, 90, -1 )
      elif lead_objspd < 0:
        lead_wait_cmd, lead_set_speed = self.get_tm_speed( CS, 150, -1 )
      elif CS.VSetDis > (CS.clu_Vanz + 5):
        lead_wait_cmd = 100
        lead_set_speed = CS.VSetDis - 1 # CS.clu_Vanz + 5
        if lead_set_speed < 30:
            lead_set_speed = 30

      str3 = '<{:.0f} obj={:.0f} speed={:3.0f} time={:3.0f}'.format( d_delta, lead_objspd, lead_set_speed, lead_wait_cmd )
      self.SC.add(  str3 )
    # 선행차량이 멀리 있으면.
    elif lead_objspd < -10:  
        lead_wait_cmd, lead_set_speed = self.get_tm_speed( CS, 50, -1 )
        str3 = '{:.0f} speed={:.0f} time={:3.0f}'.format( lead_objspd, lead_set_speed, lead_wait_cmd )
        self.SC.add(  str3 )         
    elif lead_objspd < -5:
      lead_wait_cmd, lead_set_speed = self.get_tm_speed( CS, 75, -1 )        
      str3 = '{:.0f} speed={:.0f} time={:3.0f}'.format( lead_objspd, lead_set_speed, lead_wait_cmd )
      self.SC.add(  str3 )         
    elif lead_objspd < -1:
      lead_wait_cmd, lead_set_speed = self.get_tm_speed( CS, 100, -1 )           
      str3 = '{:.0f} speed={:.0f} time={:3.0f}'.format( lead_objspd, lead_set_speed, lead_wait_cmd )
      self.SC.add(  str3 )         
    elif CS.cruise_set_speed_kph > CS.clu_Vanz:
      
      # 선행 차량이 가속하고 있으면.
      if dRel == 150:
        self.time_no_lean += 1
        if self.time_no_lean < 50:
          lead_wait_cmd, lead_set_speed = self.get_tm_speed( CS, 75, 1 )
        else:
          lead_wait_cmd, lead_set_speed = self.get_tm_speed( CS, 50, 1 )
      elif lead_objspd < 3 or d_delta < 5:
        lead_set_speed = int(CS.VSetDis)
      elif lead_objspd < 5:
        lead_wait_cmd, lead_set_speed = self.get_tm_speed( CS, 50, 1 )
      elif lead_objspd < 10:
        lead_wait_cmd, lead_set_speed = self.get_tm_speed( CS, 35, 1 )
      else:
        lead_wait_cmd, lead_set_speed = self.get_tm_speed( CS, 25, 1 )

      if lead_wait_cmd != 300:
        str3 = 'acc speed={:3.0f} time={:3.0f}'.format( lead_set_speed, lead_wait_cmd )
        self.SC.add(  str3 )

    return  lead_wait_cmd, lead_set_speed


  def update_curv(self, CS, sm, model_speed ):
    wait_time_cmd = 0
    set_speed = CS.cruise_set_speed_kph

    #model_speed = self.calc_va( sm, CS.v_ego )
    # 2. 커브 감속.
    if CS.cruise_set_speed_kph >= 60:
      if model_speed < 60:
        set_speed = CS.cruise_set_speed_kph - 20
        wait_time_cmd = 50
      elif model_speed < 90:
        set_speed = CS.cruise_set_speed_kph - 10
        wait_time_cmd = 75
      elif model_speed < 140:
        set_speed = CS.cruise_set_speed_kph - 5
        wait_time_cmd = 100


    return wait_time_cmd, set_speed

  def update(self, v_ego_kph, CS, sm, actuators, dRel, yRel, vRel, model_speed ):
    btn_type = Buttons.NONE
    #lead_1 = sm['radarState'].leadOne
    long_wait_cmd = 250
    set_speed = CS.cruise_set_speed_kph

    lead_wait_cmd, lead_set_speed = self.update_lead( CS,  dRel, yRel, vRel )  #선행 차량 거리유지
    curv_wait_cmd, curv_set_speed = self.update_curv( CS, sm, model_speed )  # 커브 감속.

    if curv_wait_cmd != 0:
      if lead_set_speed > curv_set_speed:
        set_speed = curv_set_speed
        long_wait_cmd = curv_wait_cmd   
      else:
        set_speed = lead_set_speed
        long_wait_cmd = lead_wait_cmd      
    else: 
      set_speed = lead_set_speed
      long_wait_cmd = lead_wait_cmd


    if  set_speed > CS.cruise_set_speed_kph:
        set_speed = CS.cruise_set_speed_kph
    elif set_speed < 30:
        set_speed = 30



    # control process
    target_set_speed = set_speed
    delta = int(set_speed) - int(CS.VSetDis)
    if abs(CS.cruise_set_speed_kph - CS.VSetDis) <= 1:
      long_wait_cmd = 100

    if self.long_wait_timer:
      self.long_wait_timer -= 1      
      if self.long_wait_timer > long_wait_cmd:
        self.long_wait_timer = long_wait_cmd
    elif delta <= -1:
      set_speed = CS.VSetDis - 1
      btn_type = Buttons.SET_DECEL
      self.long_wait_timer = long_wait_cmd
    elif  delta >= 1 and (model_speed > 150 or CS.clu_Vanz < 60):
      set_speed = CS.VSetDis + 1
      btn_type = Buttons.RES_ACCEL
      self.long_wait_timer = long_wait_cmd



    if CS.cruise_set_mode == 0:
       btn_type = Buttons.NONE
       self.long_wait_timer = 0

    tm_sample = self.Timer1.sampleTime()


    str3 = 'curvature={:3.0f} dest={:3.0f}/{:3.0f} heart={:.0f} '.format( model_speed, target_set_speed, self.long_wait_timer,  tm_sample )
    trace1.printf2(  str3 )

    return btn_type, set_speed
