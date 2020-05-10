#!/usr/bin/env python3
import gc

from cereal import car
from common.params import Params
from common.realtime import set_realtime_priority
from selfdrive.swaglog import cloudlog
from selfdrive.controls.lib.planner import Planner
from selfdrive.controls.lib.vehicle_model import VehicleModel
from selfdrive.controls.lib.pathplanner import PathPlanner
import cereal.messaging as messaging


def plannerd_thread(sm=None, pm=None):
  gc.disable()

  # start the loop
  set_realtime_priority(2)

  cloudlog.info("plannerd is waiting for CarParams")
  CP = car.CarParams.from_bytes(Params().get("CarParams", block=True))
  cloudlog.info("plannerd got CarParams: %s", CP.carName)

  PL = Planner(CP)
  PP = PathPlanner(CP)

  VM = VehicleModel(CP)

  if sm is None:
    sm = messaging.SubMaster(['carState', 'controlsState', 'radarState', 'model', 'liveParameters'])

  if pm is None:
    pm = messaging.PubMaster(['plan', 'liveLongitudinalMpc', 'pathPlan', 'liveMpc'])

  sm['liveParameters'].valid = True
  sm['liveParameters'].sensorValid = True
  sm['liveParameters'].steerRatio = CP.steerRatio
  sm['liveParameters'].stiffnessFactor = 1.0

  while True:
    sm.update()

    nlist = 0
    for s in sm.updated:
      nlist += 1
      str_log = 'plannerd{}  s={}  alive={} valid={}  updated={}'.format( nlist,  s, sm.alive[s],  sm.valid[s], sm.updated[s]  )
      print(  str_log ) 

    if sm.updated['model']:
      PP.update(sm, pm, CP, VM)
    else:
      print( 'ERR => sm.updated_model' )




    if sm.updated['radarState']:
      PL.update(sm, pm, CP, VM, PP)
    else:
      print( 'ERR => sm.updated_radarState' )


def main(sm=None, pm=None):
  plannerd_thread(sm, pm)


if __name__ == "__main__":
  main()
