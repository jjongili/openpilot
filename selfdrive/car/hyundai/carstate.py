from cereal import car
from selfdrive.car.hyundai.values import DBC, STEER_THRESHOLD, FEATURES
from opendbc.can.parser import CANParser
from selfdrive.config import Conversions as CV
from common.kalman.simple_kalman import KF1D
from common.realtime import DT_CTRL

import common.log as trace1

GearShifter = car.CarState.GearShifter

def get_can_parser(CP):

  signals = [
    # sig_name, sig_address, default
    ("WHL_SPD_FL", "WHL_SPD11", 0),
    ("WHL_SPD_FR", "WHL_SPD11", 0),
    ("WHL_SPD_RL", "WHL_SPD11", 0),
    ("WHL_SPD_RR", "WHL_SPD11", 0),

    ("YAW_RATE", "ESP12", 0),

    ("CF_Gway_DrvSeatBeltInd", "CGW4", 1),

    ("CF_Gway_DrvSeatBeltSw", "CGW1", 0),
    ("CF_Gway_TSigLHSw", "CGW1", 0),
    ("CF_Gway_TurnSigLh", "CGW1", 0),
    ("CF_Gway_TSigRHSw", "CGW1", 0),
    ("CF_Gway_TurnSigRh", "CGW1", 0),
    ("CF_Gway_ParkBrakeSw", "CGW1", 0),

    ("BRAKE_ACT", "EMS12", 0),
    ("PV_AV_CAN", "EMS12", 0),
    ("TPS", "EMS12", 0),

    ("CYL_PRES", "ESP12", 0),

    ("CF_Clu_CruiseSwState", "CLU11", 0),
    ("CF_Clu_CruiseSwMain", "CLU11", 0),
    ("CF_Clu_SldMainSW", "CLU11", 0),
    ("CF_Clu_ParityBit1", "CLU11", 0),
    ("CF_Clu_VanzDecimal" , "CLU11", 0),
    ("CF_Clu_Vanz", "CLU11", 0),
    ("CF_Clu_SPEED_UNIT", "CLU11", 0),
    ("CF_Clu_DetentOut", "CLU11", 0),
    ("CF_Clu_RheostatLevel", "CLU11", 0),
    ("CF_Clu_CluInfo", "CLU11", 0),
    ("CF_Clu_AmpInfo", "CLU11", 0),
    ("CF_Clu_AliveCnt1", "CLU11", 0),

    ("ACCEnable", "TCS13", 0),
    ("ACC_REQ", "TCS13", 0),
    ("BrakeLight", "TCS13", 0),
    ("DriverBraking", "TCS13", 0),
    ("DriverOverride", "TCS13", 0),

    ("ESC_Off_Step", "TCS15", 0),

    ("CF_Lvr_GearInf", "LVR11", 0),        #Transmission Gear (0 = N or P, 1-8 = Fwd, 14 = Rev)

    ("CF_Lca_Stat", "LCA11", 0),
    ("CF_Lca_IndLeft", "LCA11", 0),
    ("CF_Lca_IndRight", "LCA11", 0),
  ]

  checks = [
    # address, frequency
    ("TCS15", 10),
    ("CLU11", 50),
    ("ESP12", 100),
    ("CGW1", 10),
    ("CGW4", 5),
    ("WHL_SPD11", 50),
  ]
  if not CP.mdpsBus:
    signals += [
      ("CR_Mdps_StrColTq", "MDPS12", 0),
      ("CF_Mdps_Def", "MDPS12", 0),
      ("CF_Mdps_ToiActive", "MDPS12", 0),
      ("CF_Mdps_ToiUnavail", "MDPS12", 0),
      ("CF_Mdps_MsgCount2", "MDPS12", 0),
      ("CF_Mdps_Chksum2", "MDPS12", 0),
      ("CF_Mdps_ToiFlt", "MDPS12", 0),
      ("CF_Mdps_SErr", "MDPS12", 0),
      ("CR_Mdps_StrTq", "MDPS12", 0),
      ("CF_Mdps_FailStat", "MDPS12", 0),
      ("CR_Mdps_OutTq", "MDPS12", 0)
    ]
    checks += [
      ("MDPS12", 50)
    ]
  if not CP.sasBus:
    signals += [
      ("SAS_Angle", "SAS11", 0),
      ("SAS_Speed", "SAS11", 0),
    ]
    checks += [
      ("SAS11", 100)
    ]
  if CP.sccBus == -1:
    signals += [
      ("CRUISE_LAMP_M", "EMS16", 0),
      ("CF_Lvr_CruiseSet", "LVR12", 0),
    ]
  elif not CP.sccBus:
    signals += [
      ("MainMode_ACC", "SCC11", 0),
      ("VSetDis", "SCC11", 0),
      ("SCCInfoDisplay", "SCC11", 0),
      ("ACC_ObjDist", "SCC11", 0),
      ("ACC_ObjRelSpd", "SCC11", 0),
       ("TauGapSet", "SCC11", 0),
      ("Navi_SCC_Camera_Act", "SCC11", 0),

      ("ACCMode", "SCC12", 0),
      ("CF_VSM_Prefill", "SCC12", 0),
      ("CF_VSM_DecCmdAct", "SCC12", 0),
      ("CF_VSM_HBACmd", "SCC12", 0),
      ("CF_VSM_Warn", "SCC12", 0),
      ("CF_VSM_Stat", "SCC12", 0),
      ("CF_VSM_BeltCmd", "SCC12", 0),
      ("ACCFailInfo", "SCC12", 0),
      ("ACCMode", "SCC12", 0),
      ("StopReq", "SCC12", 0),
      ("CR_VSM_DecCmd", "SCC12", 0),
      ("aReqMax", "SCC12", 0),
      ("TakeOverReq", "SCC12", 0),
      ("PreFill", "SCC12", 0),
      ("aReqMin", "SCC12", 0),
      ("CF_VSM_ConfMode", "SCC12", 0),
      ("AEB_Failinfo", "SCC12", 0),
      ("AEB_Status", "SCC12", 0),
      ("AEB_CmdAct", "SCC12", 0),
      ("AEB_StopReq", "SCC12", 0),
      ("CR_VSM_Alive", "SCC12", 0),
      ("CR_VSM_ChkSum", "SCC12", 0),
    ]
    checks += [
      ("SCC11", 50),
      ("SCC12", 50),
    ]
  if CP.carFingerprint in FEATURES["use_cluster_gears"]:
    signals += [
      ("CF_Clu_InhibitD", "CLU15", 0),
      ("CF_Clu_InhibitP", "CLU15", 0),
      ("CF_Clu_InhibitN", "CLU15", 0),
      ("CF_Clu_InhibitR", "CLU15", 0),
    ]
  elif CP.carFingerprint in FEATURES["use_tcu_gears"]:
    signals += [
      ("CUR_GR", "TCU12",0),
    ]
  elif CP.carFingerprint in FEATURES["use_elect_gears"]:
    signals += [
      ("Elect_Gear_Shifter", "ELECT_GEAR", 0),
	]
  else:
    signals += [
      ("CF_Lvr_Gear","LVR12",0),
    ]
  return CANParser(DBC[CP.carFingerprint]['pt'], signals, checks, 0)

def get_can2_parser(CP):
  signals = []
  checks = []
  if CP.mdpsBus == 1:
    signals += [
      ("CR_Mdps_StrColTq", "MDPS12", 0),
      ("CF_Mdps_Def", "MDPS12", 0),
      ("CF_Mdps_ToiActive", "MDPS12", 0),
      ("CF_Mdps_ToiUnavail", "MDPS12", 0),
      ("CF_Mdps_MsgCount2", "MDPS12", 0),
      ("CF_Mdps_Chksum2", "MDPS12", 0),
      ("CF_Mdps_ToiFlt", "MDPS12", 0),
      ("CF_Mdps_SErr", "MDPS12", 0),
      ("CR_Mdps_StrTq", "MDPS12", 0),
      ("CF_Mdps_FailStat", "MDPS12", 0),
      ("CR_Mdps_OutTq", "MDPS12", 0)
    ]
    checks += [
      ("MDPS12", 50)
    ]
  if CP.sasBus == 1:
    signals += [
      ("SAS_Angle", "SAS11", 0),
      ("SAS_Speed", "SAS11", 0),
    ]
    checks += [
      ("SAS11", 100)
    ]
  if CP.sccBus == 1:
    signals += [
      ("MainMode_ACC", "SCC11", 0),
      ("VSetDis", "SCC11", 0),
      ("SCCInfoDisplay", "SCC11", 0),
      ("ACC_ObjDist", "SCC11", 0),
      ("ACC_ObjRelSpd", "SCC11", 0),
      ("TauGapSet", "SCC11", 0),
      ("Navi_SCC_Camera_Act", "SCC11", 0),

      ("ACCMode", "SCC12", 0),
      ("CF_VSM_Prefill", "SCC12", 0),
      ("CF_VSM_DecCmdAct", "SCC12", 0),
      ("CF_VSM_HBACmd", "SCC12", 0),
      ("CF_VSM_Warn", "SCC12", 0),
      ("CF_VSM_Stat", "SCC12", 0),
      ("CF_VSM_BeltCmd", "SCC12", 0),
      ("ACCFailInfo", "SCC12", 0),
      ("ACCMode", "SCC12", 0),
      ("StopReq", "SCC12", 0),
      ("CR_VSM_DecCmd", "SCC12", 0),
      ("aReqMax", "SCC12", 0),
      ("TakeOverReq", "SCC12", 0),
      ("PreFill", "SCC12", 0),
      ("aReqMin", "SCC12", 0),
      ("CF_VSM_ConfMode", "SCC12", 0),
      ("AEB_Failinfo", "SCC12", 0),
      ("AEB_Status", "SCC12", 0),
      ("AEB_CmdAct", "SCC12", 0),
      ("AEB_StopReq", "SCC12", 0),
      ("CR_VSM_Alive", "SCC12", 0),
      ("CR_VSM_ChkSum", "SCC12", 0)
    ]
    checks += [
      ("SCC11", 50),
      ("SCC12", 50),
    ]
  return CANParser(DBC[CP.carFingerprint]['pt'], signals, checks, 1)

def get_camera_parser(CP):

  signals = [
    # sig_name, sig_address, default
    ("CF_Lkas_Bca_R", "LKAS11", 0),
    ("CF_Lkas_LdwsSysState", "LKAS11", 0),
    ("CF_Lkas_SysWarning", "LKAS11", 0),
    ("CF_Lkas_LdwsLHWarning", "LKAS11", 0),
    ("CF_Lkas_LdwsRHWarning", "LKAS11", 0),
    ("CF_Lkas_HbaLamp", "LKAS11", 0),
    ("CF_Lkas_FcwBasReq", "LKAS11", 0),
    ("CF_Lkas_ToiFlt", "LKAS11", 0),
    ("CF_Lkas_HbaSysState", "LKAS11", 0),
    ("CF_Lkas_FcwOpt", "LKAS11", 0),
    ("CF_Lkas_HbaOpt", "LKAS11", 0),
    ("CF_Lkas_FcwSysState", "LKAS11", 0),
    ("CF_Lkas_FcwCollisionWarning", "LKAS11", 0),
    ("CF_Lkas_MsgCount", "LKAS11", 0),
    ("CF_Lkas_FusionState", "LKAS11", 0),
    ("CF_Lkas_FcwOpt_USM", "LKAS11", 0),
    ("CF_Lkas_LdwsOpt_USM", "LKAS11", 0)
  ]

  checks = []
  if CP.sccBus == 2:
    signals += [
      ("MainMode_ACC", "SCC11", 0),
      ("VSetDis", "SCC11", 0),
      ("SCCInfoDisplay", "SCC11", 0),
      ("ACC_ObjDist", "SCC11", 0),
      ("ACC_ObjRelSpd", "SCC11", 0),
      ("TauGapSet", "SCC11", 0),
      ("Navi_SCC_Camera_Act", "SCC11", 0),

      ("ACCMode", "SCC12", 0),
      ("CF_VSM_Prefill", "SCC12", 0),
      ("CF_VSM_DecCmdAct", "SCC12", 0),
      ("CF_VSM_HBACmd", "SCC12", 0),
      ("CF_VSM_Warn", "SCC12", 0),
      ("CF_VSM_Stat", "SCC12", 0),
      ("CF_VSM_BeltCmd", "SCC12", 0),
      ("ACCFailInfo", "SCC12", 0),
      ("ACCMode", "SCC12", 0),
      ("StopReq", "SCC12", 0),
      ("CR_VSM_DecCmd", "SCC12", 0),
      ("aReqMax", "SCC12", 0),
      ("TakeOverReq", "SCC12", 0),
      ("PreFill", "SCC12", 0),
      ("aReqMin", "SCC12", 0),
      ("CF_VSM_ConfMode", "SCC12", 0),
      ("AEB_Failinfo", "SCC12", 0),
      ("AEB_Status", "SCC12", 0),
      ("AEB_CmdAct", "SCC12", 0),
      ("AEB_StopReq", "SCC12", 0),
      ("CR_VSM_Alive", "SCC12", 0),
      ("CR_VSM_ChkSum", "SCC12", 0),
    ]
    checks += [
      ("SCC11", 50),
      ("SCC12", 50),
    ]
  return CANParser(DBC[CP.carFingerprint]['pt'], signals, checks, 2)




def get_AVM_parser(CP):

  signals = [
    # sig_name, sig_address, default
    ("AVM_View", "AVM_HU_PE_00", 0),
    ("AVM_ParkingAssist_BtnSts", "AVM_HU_PE_00", 0),
    ("AVM_Display_Message", "AVM_HU_PE_00", 0),
    ("AVM_Popup_Msg", "AVM_HU_PE_00", 0),
    ("AVM_Ready", "AVM_HU_PE_00", 0),
    ("AVM_ParkingAssist_Step", "AVM_HU_PE_00", 0),
    ("AVM_FrontBtn_Type", "AVM_HU_PE_00", 0),
    ("AVM_Option", "AVM_HU_PE_00", 0),
    ("AVM_HU_FrontViewPointOpt", "AVM_HU_PE_00", 0),
    ("AVM_HU_RearView_Option", "AVM_HU_PE_00", 0),
    ("AVM_HU_FrontView_Option", "AVM_HU_PE_00", 0),
    ("AVM_Version", "AVM_HU_PE_00", 0),
  ]

  checks = []

  checks += [
    ("AVM_HU_PE_00", 50),
  ]
  return CANParser(DBC[CP.carFingerprint]['pt'], signals, checks, 0)

class CarState():

  def __init__(self, CP):
    self.CP = CP
    # initialize can parser
    self.car_fingerprint = CP.carFingerprint
    self.left_blinker_on = 0
    self.left_blinker_flash = 0
    self.right_blinker_on = 0
    self.right_blinker_flash = 0
    self.lca_left = 0
    self.lca_right = 0
    self.no_radar = CP.sccBus == -1
    self.mdps_bus = CP.mdpsBus
    self.sas_bus = CP.sasBus
    self.scc_bus = CP.sccBus


    self.blinker_timer = 0
    self.blinker_status = 0

    self.lkas_LdwsSysState = 0
    self.lkas_LdwsLHWarning = 0
    self.lkas_LdwsRHWarning = 0

    self.clu_CruiseSwState = 0


    self.cruise_set_speed = 0
    self.cruise_set_speed_kph = 0
    self.curise_set_first = 0
    self.prev_clu_CruiseSwState = 0

    self.prev_VSetDis = 30

    self.cruise_set_mode = 0

    # Q = np.matrix([[10.0, 0.0], [0.0, 100.0]])
    # R = 1e3
    self.v_ego_kf = KF1D(x0=[[0.0], [0.0]],
                         A=[[1.0, DT_CTRL], [0.0, 1.0]],
                         C=[1.0, 0.0],
                         K=[[0.12287673], [0.29666309]])

  def update_speed_kf(self, v_ego_raw):
    if abs(v_ego_raw - self.v_ego_kf.x[0][0]) > 2.0:  # Prevent large accelerations when car starts at non zero speed
      self.v_ego_kf.x = [[v_ego_raw], [0.0]]

    v_ego_x = self.v_ego_kf.update(v_ego_raw)
    return float(v_ego_x[0]), float(v_ego_x[1])
	

  def update_cruiseSW(self ):
    cruise_set_speed_kph = self.cruise_set_speed_kph
    delta_vsetdis = 0
    if self.pcm_acc_status:
      delta_vsetdis = abs(self.VSetDis - self.prev_VSetDis)
      if self.prev_clu_CruiseSwState != self.clu_CruiseSwState:
        if self.clu_CruiseSwState:
          self.prev_VSetDis = int(self.VSetDis)
        elif self.prev_clu_CruiseSwState == 1:   # up
          if self.curise_set_first:
            self.curise_set_first = 0
            cruise_set_speed_kph =  int(self.clu_Vanz)
          elif delta_vsetdis > 5:
            cruise_set_speed_kph = self.VSetDis
          else:
            cruise_set_speed_kph += 1
        elif self.prev_clu_CruiseSwState == 2:  # dn
          if self.curise_set_first:
            self.curise_set_first = 0
            cruise_set_speed_kph =  int(self.clu_Vanz)
          elif delta_vsetdis > 5:
            cruise_set_speed_kph =  int(self.VSetDis)
          else:
            cruise_set_speed_kph -= 1

        self.prev_clu_CruiseSwState = self.clu_CruiseSwState
      elif self.clu_CruiseSwState and delta_vsetdis > 0:
        cruise_set_speed_kph =  int(self.VSetDis)

    else:
      self.curise_set_first = 1
      cruise_set_speed_kph = self.VSetDis

      if self.prev_clu_CruiseSwState != self.clu_CruiseSwState:
        if self.clu_CruiseSwState == 4:
          self.cruise_set_mode += 1
          if self.cruise_set_mode > 2:
            self.cruise_set_mode = 0
        self.prev_clu_CruiseSwState = self.clu_CruiseSwState
      
    trace1.cruise_set_mode = self.cruise_set_mode

    if cruise_set_speed_kph < 30:
      cruise_set_speed_kph = 30

    return cruise_set_speed_kph



  def update(self, cp, cp2, cp_cam, cp_avm ):


    cp_mdps = cp2 if self.mdps_bus else cp
    cp_sas = cp2 if self.sas_bus else cp
    cp_scc = cp2 if self.scc_bus == 1 else cp_cam if self.scc_bus == 2 else cp

    # update prevs, update must run once per Loop
    self.prev_left_blinker_on = self.left_blinker_on
    self.prev_right_blinker_on = self.right_blinker_on
    self.prev_left_blinker_flash = self.left_blinker_flash
    self.prev_right_blinker_flash = self.right_blinker_flash

    self.door_all_closed = True
    self.seatbelt = cp.vl["CGW1"]['CF_Gway_DrvSeatBeltSw']

    self.brake_pressed = cp.vl["TCS13"]['DriverBraking']
    self.esp_disabled = cp.vl["TCS15"]['ESC_Off_Step']
    self.park_brake = cp.vl["CGW1"]['CF_Gway_ParkBrakeSw']

    self.Navi_HDA = cp.vl["SCC11"]['Navi_SCC_Camera_Act']   #  Cam Area 2,  Highway 1, normal 0


    # AVM
    self.AVM_View = cp_avm.vl["AVM_HU_PE_00"]["AVM_View"]
    self.AVM_ParkAssist_btn = cp_avm.vl["AVM_HU_PE_00"]["AVM_ParkingAssist_BtnSts"]
    self.AVM_Disp_Msg = cp_avm.vl["AVM_HU_PE_00"]["AVM_Display_Message"]
    self.AVM_Popup_Msg = cp_avm.vl["AVM_HU_PE_00"]["AVM_Popup_Msg"]
    self.AVM_Ready= cp_avm.vl["AVM_HU_PE_00"]["AVM_Ready"]
    self.AVM_ParkAssit_step = cp_avm.vl["AVM_HU_PE_00"]["AVM_ParkingAssist_Step"]
    self.AVM_FrontBtn = cp_avm.vl["AVM_HU_PE_00"]["AVM_FrontBtn_Type"]
    self.AVM_Option = cp_avm.vl["AVM_HU_PE_00"]["AVM_Option"]
    self.AVM_HU_FrontView = cp_avm.vl["AVM_HU_PE_00"]["AVM_HU_FrontViewPointOpt"]
    self.AVM_HU_RearView = cp_avm.vl["AVM_HU_PE_00"]["AVM_HU_RearView_Option"]
    self.AVM_HU_FrontView = cp_avm.vl["AVM_HU_PE_00"]["AVM_HU_FrontView_Option"]
    self.AVM_Version = cp_avm.vl["AVM_HU_PE_00"]["AVM_Version"]


    self.main_on = (cp_scc.vl["SCC11"]["MainMode_ACC"] != 0) if not self.no_radar else \
                                            cp.vl['EMS16']['CRUISE_LAMP_M']
    self.acc_active = (cp_scc.vl["SCC12"]['ACCMode'] != 0) if not self.no_radar else \
                                      (cp.vl["LVR12"]['CF_Lvr_CruiseSet'] != 0)
    self.pcm_acc_status = int(self.acc_active)

    self.v_wheel_fl = cp.vl["WHL_SPD11"]['WHL_SPD_FL'] * CV.KPH_TO_MS
    self.v_wheel_fr = cp.vl["WHL_SPD11"]['WHL_SPD_FR'] * CV.KPH_TO_MS
    self.v_wheel_rl = cp.vl["WHL_SPD11"]['WHL_SPD_RL'] * CV.KPH_TO_MS
    self.v_wheel_rr = cp.vl["WHL_SPD11"]['WHL_SPD_RR'] * CV.KPH_TO_MS
    self.v_ego_raw = (self.v_wheel_fl + self.v_wheel_fr + self.v_wheel_rl + self.v_wheel_rr) / 4.
    v_ego, self.a_ego = self.update_speed_kf(self.v_ego_raw)

    self.low_speed_lockout = self.v_ego_raw < 1.0



    self.clu_Vanz = cp.vl["CLU11"]["CF_Clu_Vanz"]
    self.clu_CruiseSwState = cp.vl["CLU11"]["CF_Clu_CruiseSwState"]
    self.clu_CruiseSwMain = cp.vl["CLU11"]["CF_Clu_CruiseSwMain"]
    self.clu_SldMainSW = cp.vl["CLU11"]["CF_Clu_SldMainSW"]
    self.v_ego = self.clu_Vanz * CV.KPH_TO_MS

    self.VSetDis = cp_scc.vl["SCC11"]['VSetDis']



    self.is_set_speed_in_mph = int(cp.vl["CLU11"]["CF_Clu_SPEED_UNIT"])
    speed_conv = CV.MPH_TO_MS if self.is_set_speed_in_mph else CV.KPH_TO_MS
    #self.cruise_set_speed = self.VSetDis * speed_conv if not self.no_radar else \
    #                                     (cp.vl["LVR12"]["CF_Lvr_CruiseSet"] * speed_conv)



    self.standstill = not self.v_ego_raw > 0.1


    self.steer_torque_driver = cp_mdps.vl["MDPS12"]['CR_Mdps_StrColTq']
    self.steer_torque_motor = cp_mdps.vl["MDPS12"]['CR_Mdps_OutTq']

    self.angle_steers = cp_sas.vl["SAS11"]['SAS_Angle']
    self.angle_steers_rate = cp_sas.vl["SAS11"]['SAS_Speed']
    self.yaw_rate = cp.vl["ESP12"]['YAW_RATE']
    self.left_blinker_on = cp.vl["CGW1"]['CF_Gway_TSigLHSw']
    self.left_blinker_flash = cp.vl["CGW1"]['CF_Gway_TurnSigLh']
    self.right_blinker_on = cp.vl["CGW1"]['CF_Gway_TSigRHSw']
    self.right_blinker_flash = cp.vl["CGW1"]['CF_Gway_TurnSigRh']
    self.steer_override = abs(self.steer_torque_driver) > STEER_THRESHOLD
    self.steer_state = cp_mdps.vl["MDPS12"]['CF_Mdps_ToiActive'] #0 NOT ACTIVE, 1 ACTIVE
    self.steer_error = cp_mdps.vl["MDPS12"]['CF_Mdps_ToiUnavail']
    self.brake_error = 0


    self.driverOverride = cp.vl["TCS13"]["DriverOverride"]     # 1 Acc,  2 bracking, 0 Normal

    self.sccInfoDisp = cp_scc.vl["SCC11"]['SCCInfoDisplay']
    self.stopped = self.sccInfoDisp == 4. if not self.no_radar else False
    self.lead_distance = cp_scc.vl["SCC11"]['ACC_ObjDist'] if not self.no_radar else 0
    self.lead_objspd = cp_scc.vl["SCC11"]['ACC_ObjRelSpd'] if not self.no_radar else 0
    self.lead_objspd = self.lead_objspd * CV.MS_TO_KPH

    self.user_brake = 0

    self.brake_pressed = cp.vl["TCS13"]['DriverBraking']
    self.brake_lights = bool(cp.vl["TCS13"]['BrakeLight'] or self.brake_pressed)
    if (self.driverOverride == 0 and cp.vl["TCS13"]['ACC_REQ'] == 1):
      self.pedal_gas = 0
    else:
      self.pedal_gas = cp.vl["EMS12"]['TPS']
    self.car_gas = cp.vl["EMS12"]['TPS']

    # gear  shifter
    gear = cp.vl["ELECT_GEAR"]["Elect_Gear_Shifter"]
    if gear in (5, 8): # 5: D, 8: sport mode
      self.gear_shifter = GearShifter.drive
    elif gear == 6:
      self.gear_shifter = GearShifter.neutral
    elif gear == 0:
      self.gear_shifter = GearShifter.park
    elif gear == 7:
      self.gear_shifter = GearShifter.reverse
    else:
      self.gear_shifter = GearShifter.drive # fixed by KYD to resolve "Gear not D" issue


    self.lkas_LdwsLHWarning = cp_cam.vl["LKAS11"]["CF_Lkas_LdwsLHWarning"]
    self.lkas_LdwsRHWarning = cp_cam.vl["LKAS11"]["CF_Lkas_LdwsRHWarning"]
    self.lkas_LdwsSysState = cp_cam.vl["LKAS11"]["CF_Lkas_LdwsSysState"]   # 16 over steer
    self.lkas_button_on = 7 > self.lkas_LdwsSysState != 0
    self.lkas_error = self.lkas_LdwsSysState == 7

    # Blind Spot Detection and Lane Change Assist signals
    self.lca_state = cp.vl["LCA11"]["CF_Lca_Stat"]
    self.lca_left = cp.vl["LCA11"]["CF_Lca_IndLeft"]
    self.lca_right = cp.vl["LCA11"]["CF_Lca_IndRight"]



    # save the entire LKAS11, CLU11, SCC12 and MDPS12
    self.lkas11 = cp_cam.vl["LKAS11"]
    self.clu11 = cp.vl["CLU11"]
    self.scc12 = cp_scc.vl["SCC12"]
    self.mdps12 = cp_mdps.vl["MDPS12"]
    self.cgw1 = cp.vl["CGW1"]
    self.avm = cp_avm.vl["AVM_HU_PE_00"]  
  
    # blinker process
    blinker_status = 0
    if self.left_blinker_flash and self.right_blinker_flash:
        blinker_status = 3
    elif self.left_blinker_flash:
        blinker_status = 2
    elif self.right_blinker_flash:
        blinker_status = 1


    if blinker_status:
        self.blinker_status = blinker_status
        self.blinker_timer = 200
    elif self.blinker_timer:
       self.blinker_timer -= 1
    else:
       self.blinker_status = 0





    self.cruise_set_speed_kph = self.update_cruiseSW()
    self.cruise_set_speed = self.cruise_set_speed_kph * speed_conv
    #str1 = 'C:{:.0f}  as={:.1f} set{:.1f}'.format( self.main_on,  self.pcm_acc_status,  self.cruise_set_speed )
    #str2 = 'sw={:.0f}/{:.0f}/{:.0f} gear={:.0f} scc={:.0f}'.format( self.clu_CruiseSwState, self.clu_CruiseSwMain, self.clu_SldMainSW, self.gear_shifter, self.sccInfoDisp )

    #trace1.printf( '{} {}'.format( str1, str2 ) )