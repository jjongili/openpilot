#!/usr/bin/env python3
from cereal import car
from selfdrive.config import Conversions as CV
from selfdrive.controls.lib.drive_helpers import EventTypes as ET, create_event
from selfdrive.controls.lib.vehicle_model import VehicleModel
from selfdrive.car.hyundai.carstate import CarState, get_can_parser, get_can2_parser, get_camera_parser, get_AVM_parser
from selfdrive.car.hyundai.values import Ecu, ECU_FINGERPRINT, CAR, FINGERPRINTS, LaneChangeParms
from selfdrive.car import STD_CARGO_KG, scale_rot_inertia, scale_tire_stiffness, is_ecu_disconnected, gen_empty_fingerprint
from selfdrive.car.interfaces import CarInterfaceBase
from common.params import Params

import common.log as trace1

GearShifter = car.CarState.GearShifter
ButtonType = car.CarState.ButtonEvent.Type

class CarInterface(CarInterfaceBase):
  def __init__(self, CP, CarController):
    self.CP = CP
    self.VM = VehicleModel(CP)
    self.frame = 0

    self.gas_pressed_prev = False
    self.brake_pressed_prev = False
    self.cruise_enabled_prev = False
    self.low_speed_alert = False

    self.blinker_status = 0
    self.blinker_timer = 0

    # *** init the major players ***
    self.CS = CarState(CP)
    self.cp = get_can_parser(CP)
    self.cp2 = get_can2_parser(CP)
    self.cp_cam = get_camera_parser(CP)
    self.cp_AVM = get_AVM_parser(CP)

    self.CC = None
    if CarController is not None:
      self.CC = CarController(self.cp.dbc_name, CP.carFingerprint)

    self.traceLKA = trace1.Loger("LKA")
    self.traceCLU = trace1.Loger("clu11")
    self.traceSCC = trace1.Loger("scc12")
    self.traceMDPS = trace1.Loger("mdps12")
    self.traceCGW = trace1.Loger("CGW1")

    self.params = Params()
    #self.lane_change_enabled = self.params.get('LaneChangeEnabled') == b'1'
    #self.speed_control_enabled = self.params.get('SpeedControlEnabled') == b'1'
    #self.car_avoid_enable = self.params.get('CarAvoidanceEnabled') == b'1'            

  @staticmethod
  def compute_gb(accel, speed):
    return float(accel) / 3.0

  @staticmethod
  def get_params(candidate, fingerprint=gen_empty_fingerprint(), has_relay=False, car_fw=[]):

    ret = car.CarParams.new_message()

    ret.carName = "hyundai"
    ret.carFingerprint = candidate
    ret.isPandaBlack = has_relay
    ret.safetyModel = car.CarParams.SafetyModel.hyundai
    ret.enableCruise = True  # stock acc
    ret.minSteerSpeed = 0    # 5 km/h     

    ret.steerActuatorDelay = 0.10  # Default delay   0.15
    ret.steerRateCost = 0.45
    ret.steerLimitTimer = 0.8
    tire_stiffness_factor = 0.75
    #ret.radarOffCan = False

    if candidate in [CAR.SANTAFE, CAR.SANTAFE_1]:
      ret.lateralTuning.pid.kf = 0.00005
      ret.mass = 3982. * CV.LB_TO_KG + STD_CARGO_KG
      ret.wheelbase = 2.766
      # Values from optimizer
      ret.steerRatio = 16.55  # 13.8 is spec end-to-end
      tire_stiffness_factor = 0.82
      ret.lateralTuning.pid.kiBP, ret.lateralTuning.pid.kpBP = [[9., 22.], [9., 22.]]   # 9m/s = 32.4km/h  ~  22m/s = 79.2 km/h
      ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.18,0.20], [0.02,0.05]]
    elif candidate == CAR.SORENTO:
      ret.lateralTuning.pid.kf = 0.00005
      ret.mass = 1950. + STD_CARGO_KG
      ret.wheelbase = 2.78
      ret.steerRatio = 14.4 * 1.15
      ret.lateralTuning.pid.kiBP, ret.lateralTuning.pid.kpBP = [[9., 22.], [9., 22.]]   # 9m/s = 32.4km/h  ~  22m/s = 79.2 km/h
      ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.18,0.20], [0.02,0.05]]
    elif candidate in [CAR.AVANTE, CAR.I30]:
      ret.lateralTuning.pid.kf = 0.00006
      ret.mass = 1275. + STD_CARGO_KG
      ret.wheelbase = 2.7
      ret.steerRatio = 13.73   #Spec
      tire_stiffness_factor = 0.385
      ret.lateralTuning.pid.kiBP, ret.lateralTuning.pid.kpBP = [[9., 22.], [9., 22.]]   # 9m/s = 32.4km/h  ~  22m/s = 79.2 km/h
      ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.18,0.20], [0.02,0.05]]
    elif candidate == CAR.GENESIS:
      ret.lateralTuning.pid.kf = 0.00005
      ret.mass = 2060. + STD_CARGO_KG
      ret.wheelbase = 3.01
      ret.steerRatio = 16.5
      ret.lateralTuning.pid.kiBP, ret.lateralTuning.pid.kpBP = [[9., 22.], [9., 22.]]   # 9m/s = 32.4km/h  ~  22m/s = 79.2 km/h
      ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.18,0.20], [0.02,0.05]]
    elif candidate in [CAR.GENESIS_G90, CAR.GENESIS_G80]:
      ret.mass = 2200
      ret.wheelbase = 3.15
      ret.steerRatio = 12.069
      ret.lateralTuning.pid.kiBP, ret.lateralTuning.pid.kpBP = [[9., 22.], [9., 22.]]   # 9m/s = 32.4km/h  ~  22m/s = 79.2 km/h
      ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.18,0.20], [0.02,0.05]]
    elif candidate in [CAR.K5, CAR.SONATA]:
      ret.lateralTuning.pid.kf = 0.00005
      ret.mass = 1470. + STD_CARGO_KG
      ret.wheelbase = 2.80
      ret.steerRatio = 12.75
      ret.steerRateCost = 0.4
      ret.lateralTuning.pid.kiBP, ret.lateralTuning.pid.kpBP = [[9., 22.], [9., 22.]]   # 9m/s = 32.4km/h  ~  22m/s = 79.2 km/h
      ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.18,0.20], [0.02,0.05]]
    elif candidate == CAR.SONATA_TURBO:
      ret.lateralTuning.pid.kf = 0.00005
      ret.mass = 1565. + STD_CARGO_KG
      ret.wheelbase = 2.80
      ret.steerRatio = 14.4 * 1.15   # 15% higher at the center seems reasonable
      ret.lateralTuning.pid.kiBP, ret.lateralTuning.pid.kpBP = [[9., 22.], [9., 22.]]   # 9m/s = 32.4km/h  ~  22m/s = 79.2 km/h
      ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.18,0.20], [0.02,0.05]]
    elif candidate in [CAR.K5_HYBRID, CAR.SONATA_HYBRID]:
      ret.lateralTuning.pid.kf = 0.00005
      ret.mass = 1595. + STD_CARGO_KG
      ret.wheelbase = 2.80
      ret.steerRatio = 12.75
      ret.steerRateCost = 0.4
      ret.lateralTuning.pid.kiBP, ret.lateralTuning.pid.kpBP = [[9., 22.], [9., 22.]]   # 9m/s = 32.4km/h  ~  22m/s = 79.2 km/h
      ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.18,0.20], [0.02,0.05]]
    elif candidate in [CAR.GRANDEUR, CAR.K7]:
      ret.lateralTuning.pid.kf = 0.00005
      ret.mass = 1570. + STD_CARGO_KG
      ret.wheelbase = 2.885
      ret.steerRatio = 12.5
      ret.steerRateCost = 0.4
      ret.lateralTuning.pid.kiBP, ret.lateralTuning.pid.kpBP = [[9., 22.], [9., 22.]]   # 9m/s = 32.4km/h  ~  22m/s = 79.2 km/h
      ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.18,0.20], [0.02,0.05]]
    elif candidate in [CAR.GRANDEUR_HYBRID, CAR.K7_HYBRID]:
      ret.lateralTuning.pid.kf = 0.00005
      ret.mass = 1675. + STD_CARGO_KG
      ret.wheelbase = 2.845
      ret.steerRatio = 12.0  #12.5
      ret.steerRateCost = 0.4 #0.4
      ret.lateralTuning.pid.kiBP, ret.lateralTuning.pid.kpBP = [[9., 22.], [9., 22.]]   # 32.4 KPH ~ 79.2 KPH
      ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.18,0.20], [0.02,0.05]]
    elif candidate == CAR.STINGER:
      ret.lateralTuning.pid.kf = 0.00005
      ret.mass = 1825. + STD_CARGO_KG
      ret.wheelbase = 2.78
      ret.steerRatio = 14.4 * 1.15   # 15% higher at the center seems reasonable
      ret.lateralTuning.pid.kiBP, ret.lateralTuning.pid.kpBP = [[9., 22.], [9., 22.]]   # 9m/s = 32.4km/h  ~  22m/s = 79.2 km/h
      ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.18,0.20], [0.02,0.05]]
    elif candidate == CAR.KONA:
      ret.lateralTuning.pid.kf = 0.00005
      ret.mass = 1330. + STD_CARGO_KG
      ret.wheelbase = 2.6
      ret.steerRatio = 13.5   #Spec
      ret.steerRateCost = 0.4
      tire_stiffness_factor = 0.385
      ret.lateralTuning.pid.kiBP, ret.lateralTuning.pid.kpBP = [[9., 22.], [9., 22.]]   # 9m/s = 32.4km/h  ~  22m/s = 79.2 km/h
      ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.18,0.20], [0.02,0.05]]
    elif candidate == CAR.KONA_EV:
      ret.lateralTuning.pid.kf = 0.00005
      ret.mass = 1330. + STD_CARGO_KG
      ret.wheelbase = 2.6
      ret.steerRatio = 13.5   #Spec
      ret.steerRateCost = 0.4
      tire_stiffness_factor = 0.385
      ret.lateralTuning.pid.kiBP, ret.lateralTuning.pid.kpBP = [[9., 22.], [9., 22.]]   # 9m/s = 32.4km/h  ~  22m/s = 79.2 km/h
      ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.18,0.20], [0.02,0.05]]
    elif candidate == CAR.NIRO:
      ret.lateralTuning.pid.kf = 0.00006
      ret.mass = 1425. + STD_CARGO_KG
      ret.wheelbase = 2.7
      ret.steerRatio = 13.73   #Spec
      tire_stiffness_factor = 0.385
      ret.lateralTuning.pid.kiBP, ret.lateralTuning.pid.kpBP = [[9., 22.], [9., 22.]]   # 9m/s = 32.4km/h  ~  22m/s = 79.2 km/h
      ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.18,0.20], [0.02,0.05]]
    elif candidate == CAR.NIRO_EV:
      ret.lateralTuning.pid.kf = 0.00006
      ret.mass = 1425. + STD_CARGO_KG
      ret.wheelbase = 2.7
      ret.steerRatio = 13.73   #Spec
      tire_stiffness_factor = 0.385
      ret.lateralTuning.pid.kiBP, ret.lateralTuning.pid.kpBP = [[9., 22.], [9., 22.]]   # 9m/s = 32.4km/h  ~  22m/s = 79.2 km/h
      ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.18,0.20], [0.02,0.05]]
    elif candidate == CAR.IONIQ:
      ret.lateralTuning.pid.kf = 0.00006
      ret.mass = 1275. + STD_CARGO_KG
      ret.wheelbase = 2.7
      ret.steerRatio = 13.73   #Spec
      tire_stiffness_factor = 0.385
      ret.lateralTuning.pid.kiBP, ret.lateralTuning.pid.kpBP = [[9., 22.], [9., 22.]]   # 9m/s = 32.4km/h  ~  22m/s = 79.2 km/h
      ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.18,0.20], [0.02,0.05]]
    elif candidate == CAR.IONIQ_EV:
      ret.lateralTuning.pid.kf = 0.00005
      ret.mass = 1490. + STD_CARGO_KG   #weight per hyundai site https://www.hyundaiusa.com/ioniq-electric/specifications.aspx
      ret.wheelbase = 2.7
      ret.steerRatio = 13.25   #Spec
      ret.steerRateCost = 0.4
      ret.lateralTuning.pid.kiBP, ret.lateralTuning.pid.kpBP = [[9., 22.], [9., 22.]]   # 9m/s = 32.4km/h  ~  22m/s = 79.2 km/h
      ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.18,0.20], [0.02,0.05]]
    elif candidate == CAR.K3:
      ret.lateralTuning.pid.kf = 0.00005
      ret.mass = 3558. * CV.LB_TO_KG
      ret.wheelbase = 2.80
      ret.steerRatio = 13.75
      tire_stiffness_factor = 0.5
      ret.lateralTuning.pid.kiBP, ret.lateralTuning.pid.kpBP = [[9., 22.], [9., 22.]]   # 9m/s = 32.4km/h  ~  22m/s = 79.2 km/h
      ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.18,0.20], [0.02,0.05]]
    elif candidate == CAR.NEXO:
      ret.lateralTuning.pid.kf = 0.00005
      ret.mass = 1885. + STD_CARGO_KG
      ret.wheelbase = 2.79
      ret.steerRatio = 12.5
      ret.lateralTuning.pid.kiBP, ret.lateralTuning.pid.kpBP = [[9., 22.], [9., 22.]]   # 9m/s = 32.4km/h  ~  22m/s = 79.2 km/h
      ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.18,0.20], [0.02,0.05]]

    ret.minEnableSpeed = -1.   # enable is done by stock ACC, so ignore this

    ret.longitudinalTuning.kpBP = [0., 5., 35.]
    ret.longitudinalTuning.kpV = [1.2, 0.8, 0.5]
    ret.longitudinalTuning.kiBP = [0., 35.]
    ret.longitudinalTuning.kiV = [0.18, 0.12]
    ret.longitudinalTuning.deadzoneBP = [0.]
    ret.longitudinalTuning.deadzoneV = [0.]

    ret.centerToFront = ret.wheelbase * 0.4

    # TODO: get actual value, for now starting with reasonable value for
    # civic and scaling by mass and wheelbase
    ret.rotationalInertia = scale_rot_inertia(ret.mass, ret.wheelbase)

    # TODO: start from empirically derived lateral slip stiffness for the civic and scale by
    # mass and CG position, so all cars will have approximately similar dyn behaviors
    ret.tireStiffnessFront, ret.tireStiffnessRear = scale_tire_stiffness(ret.mass, ret.wheelbase, ret.centerToFront,
                                                                         tire_stiffness_factor=tire_stiffness_factor)


    # no rear steering, at least on the listed cars above
    ret.steerRatioRear = 0.
    ret.steerControlType = car.CarParams.SteerControlType.torque

    # steer, gas, brake limitations VS speed
    ret.steerMaxBP = [0.]
    ret.steerMaxV = [1.0]
    ret.gasMaxBP = [0.]
    ret.gasMaxV = [0.5]
    ret.brakeMaxBP = [0., 20.]
    ret.brakeMaxV = [1., 0.8]

    ret.enableCamera = is_ecu_disconnected(fingerprint[0], FINGERPRINTS, ECU_FINGERPRINT, candidate, Ecu.fwdCamera) or has_relay
    ret.openpilotLongitudinalControl = False

    ret.stoppingControl = True
    ret.startAccel = 0.0

    # ignore CAN2 address if L-CAN on the same BUS
    ret.mdpsBus = 1 if 593 in fingerprint[1] and 1296 not in fingerprint[1] else 0
    ret.sasBus = 1 if 688 in fingerprint[1] and 1296 not in fingerprint[1] else 0
    ret.sccBus = 0 if 1056 in fingerprint[0] else 1 if 1056 in fingerprint[1] and 1296 not in fingerprint[1] \
                                                                     else 2 if 1056 in fingerprint[2] else -1
    ret.autoLcaEnabled = 0

    return ret

  # returns a car.CarState
  def update(self, c, can_strings):     # c => CC
    # ******************* do can recv *******************
    self.cp.update_strings(can_strings)
    self.cp2.update_strings(can_strings)
    self.cp_cam.update_strings(can_strings)
    self.cp_AVM.update_strings(can_strings)

    self.CS.update(self.cp, self.cp2, self.cp_cam, self.cp_AVM)
    # create message
    ret = car.CarState.new_message()

    ret.canValid = self.cp.can_valid and self.cp_cam.can_valid   #and self.cp_AVM.can_valid

    # speeds
    ret.vEgo = self.CS.v_ego
    ret.vEgoRaw = self.CS.v_ego_raw
    ret.aEgo = self.CS.a_ego
    ret.yawRate = self.CS.yaw_rate
    ret.standstill = self.CS.standstill
    ret.wheelSpeeds.fl = self.CS.v_wheel_fl
    ret.wheelSpeeds.fr = self.CS.v_wheel_fr
    ret.wheelSpeeds.rl = self.CS.v_wheel_rl
    ret.wheelSpeeds.rr = self.CS.v_wheel_rr

    # gear shifter
    ret.gearShifter = self.CS.gear_shifter

    # gas pedal
    ret.gas = self.CS.car_gas
    ret.gasPressed = self.CS.pedal_gas > 1e-3   # tolerance to avoid false press reading

    # brake pedal
    ret.brake = self.CS.user_brake
    ret.brakePressed = self.CS.brake_pressed != 0
    ret.brakeLights = self.CS.brake_lights

    # steering wheel
    ret.steeringAngle = self.CS.angle_steers
    ret.steeringRate = self.CS.angle_steers_rate  # it's unsigned

    ret.steeringTorque = self.CS.steer_torque_driver
    ret.steeringPressed = self.CS.steer_override
    ret.steeringRateLimited = self.CC.steer_rate_limited if self.CC is not None else False
    # cruise state
    # most HKG cars has no long control, it is safer and easier to engage by main on
    ret.cruiseState.enabled = (self.CS.pcm_acc_status != 0) if self.CC.longcontrol else bool(self.CS.main_on)
    if self.CS.pcm_acc_status != 0:
      ret.cruiseState.speed = self.CS.cruise_set_speed
    else:
      ret.cruiseState.speed = 0
    ret.cruiseState.available = bool(self.CS.main_on)
    ret.cruiseState.standstill = False

    #ret.cruise_set_mode = self.CS.cruise_set_mode
    
    # Some HKG cars only have blinker flash signal
    #if self.CP.carFingerprint not in [CAR.IONIQ, CAR.KONA]:
      #self.CS.left_blinker_on = self.CS.left_blinker_flash or self.CS.prev_left_blinker_on and self.CC.turning_signal_timer
      #self.CS.right_blinker_on = self.CS.right_blinker_flash or self.CS.prev_right_blinker_on and self.CC.turning_signal_timer

    blinker_status = self.CS.blinker_status
    if self.CS.left_blinker_flash or self.CS.right_blinker_flash:
      self.blinker_timer = 50
    elif self.blinker_timer: 
      self.blinker_timer -= 1
    else:
      blinker_status = 0


    if blinker_status == 3:
      ret.leftBlinker = bool(self.blinker_timer)
      ret.rightBlinker = bool(self.blinker_timer)
    elif blinker_status == 1:
      ret.leftBlinker = False
      ret.rightBlinker = bool(self.blinker_timer)
    elif blinker_status == 2:
      ret.leftBlinker = bool(self.blinker_timer)
      ret.rightBlinker = False
    else:
      ret.leftBlinker = False
      ret.rightBlinker = False
    
    #ret.leftBlinker = bool(self.CS.left_blinker_flash)
    #ret.rightBlinker = bool(self.CS.right_blinker_flash)

    ret.lcaLeft = self.CS.lca_left != 0
    ret.lcaRight = self.CS.lca_right != 0

    # TODO: button presses
    buttonEvents = []

    if self.CS.left_blinker_on != self.CS.prev_left_blinker_on:
      be = car.CarState.ButtonEvent.new_message()
      be.type = ButtonType.leftBlinker
      be.pressed = self.CS.left_blinker_on != 0
      buttonEvents.append(be)

    if self.CS.right_blinker_on != self.CS.prev_right_blinker_on:
      be = car.CarState.ButtonEvent.new_message()
      be.type = ButtonType.rightBlinker
      be.pressed = self.CS.right_blinker_on != 0
      buttonEvents.append(be)
      
    ret.buttonEvents = buttonEvents

    ret.doorOpen = not self.CS.door_all_closed
    ret.seatbeltUnlatched = not self.CS.seatbelt

    # low speed steer alert hysteresis logic (only for cars with steer cut off above 10 m/s)

    # turning indicator alert hysteresis logic
    self.turning_indicator_alert = self.CC.turning_indicator
  
    # LKAS button alert logic
    #self.lkas_button_alert = not self.CC.lkas_button
    self.low_speed_alert = self.CC.low_speed_car
    self.steer_angle_over_alert = self.CC.streer_angle_over


    events = []
    if self.CS.esp_disabled:
      events.append(create_event('espDisabled', [ET.NO_ENTRY, ET.SOFT_DISABLE])) 
    elif ret.doorOpen:
      events.append(create_event('doorOpen', [ET.NO_ENTRY, ET.SOFT_DISABLE]))
    elif ret.seatbeltUnlatched:
      events.append(create_event('seatbeltNotLatched', [ET.NO_ENTRY, ET.SOFT_DISABLE]))
    elif not self.CS.main_on:
      events.append(create_event('wrongCarMode', [ET.NO_ENTRY, ET.USER_DISABLE])) 
    elif ret.gearShifter == GearShifter.reverse:
      events.append(create_event('reverseGear', [ET.NO_ENTRY, ET.USER_DISABLE]))      
    elif not ret.gearShifter == GearShifter.drive:
      events.append(create_event('wrongGear', [ET.NO_ENTRY, ET.USER_DISABLE]))
    elif self.steer_angle_over_alert or self.CS.steer_error:
      events.append(create_event('steerTempUnavailable', [ET.NO_ENTRY, ET.WARNING]))
    #elif self.lkas_button_alert:
    #  events.append(create_event('lkasButtonOff', [ET.WARNING]))
    elif self.CS.lkas_LdwsLHWarning or self.CS.lkas_LdwsRHWarning:
      events.append(create_event('ldwPermanent', [ET.WARNING]))
    
    if ret.cruiseState.enabled != self.cruise_enabled_prev:
        if ret.cruiseState.enabled:
            events.append(create_event('pcmEnable', [ET.ENABLE]))
        else:
            events.append(create_event('pcmDisable', [ET.USER_DISABLE]))
        self.cruise_enabled_prev = ret.cruiseState.enabled
    elif  ret.cruiseState.enabled:
        if self.turning_indicator_alert:
          events.append(create_event('turningIndicatorOn', [ET.WARNING]))
        elif self.CC.steer_torque_over:
          events.append(create_event('steerTorqueOver', [ET.WARNING]))          
        elif self.CS.stopped:
          if ret.cruiseState.standstill:
            events.append(create_event('resumeRequired', [ET.WARNING]))
          else:
            events.append(create_event('preStoped', [ET.WARNING]))
        #elif self.low_speed_alert and not self.CS.mdps_bus:
        #  events.append(create_event('belowSteerSpeed', [ET.WARNING]))

    # disable on pedals rising edge or when brake is pressed and speed isn't zero
    if self.CC.longcontrol:
      if ((ret.gasPressed and not self.gas_pressed_prev) or (ret.brakePressed and (not self.brake_pressed_prev or ret.vEgoRaw > 0.1))):
          events.append(create_event('pedalPressed', [ET.NO_ENTRY, ET.USER_DISABLE]))
      if ret.gasPressed:
        events.append(create_event('pedalPressed', [ET.PRE_ENABLE]))


    #TODO Varible for min Speed for LCA
    if ret.rightBlinker and ret.lcaRight and self.CS.v_ego > LaneChangeParms.LANE_CHANGE_SPEED_MIN: 
      events.append(create_event('rightLCAbsm', [ET.WARNING]))
    if ret.leftBlinker and ret.lcaLeft and self.CS.v_ego > LaneChangeParms.LANE_CHANGE_SPEED_MIN: 
      events.append(create_event('leftLCAbsm', [ET.WARNING]))

    ret.events = events

    self.gas_pressed_prev = ret.gasPressed
    self.brake_pressed_prev = ret.brakePressed
    #self.log_update( can_strings )
    return ret.as_reader()

  def apply(self, c, sm, LaC ):
    can_sends = self.CC.update(c.enabled, self.CS, self.frame, c.actuators,
                               c.cruiseControl.cancel, c.hudControl.visualAlert, 
                               c.hudControl.leftLaneVisible, c.hudControl.rightLaneVisible, sm, LaC )
    self.frame += 1
    return can_sends

  def log_update(self, can_string):
      v_ego = self.CS.v_ego * CV.MS_TO_KPH
      log_v_ego = ' v_ego={:5.0f} km/h '.format( v_ego ) 
      if v_ego > 10:
          log_data =  log_v_ego + str(self.CS.lkas11)
          self.traceLKA.add( log_data )

          log_data = log_v_ego + str(self.CS.clu11)      
          self.traceCLU.add( log_data )

          log_data = log_v_ego + str(self.CS.scc12)      
          self.traceSCC.add( log_data )

          log_data = log_v_ego + str(self.CS.mdps12)      
          self.traceMDPS.add( log_data )

      log_data = log_v_ego + str(self.CS.cgw1)      
      self.traceCGW.add( log_data )

      