from cereal import car
from selfdrive.car import dbc_dict
Ecu = car.CarParams.Ecu

class CAR:
  NIRO = "KIA NIRO Hybrid"
  NIRO_EV = "KIA NIRO ELECTRIC"


class Buttons:
  NONE = 0
  RES_ACCEL = 1
  SET_DECEL = 2
  CANCEL = 4

FINGERPRINTS = {
  CAR.NIRO: [{
    68: 8, 127: 8, 304: 8, 320: 8, 339: 8, 352: 8, 356: 4, 544: 8, 576: 8, 832: 8, 881: 8, 882: 8, 902: 8, 903: 8, 916: 8, 1040: 8, 1056: 8, 1057: 8, 1078: 4, 1136: 6, 1173: 8, 1225: 8, 1265: 4, 1280: 1, 1287: 4, 1290: 8, 1291: 8, 1292: 8, 1294: 8, 1322: 8, 1342: 6, 1345: 8, 1348: 8, 1355: 8, 1363: 8, 1369: 8, 1407: 8, 1419: 8, 1427: 6, 1429: 8, 1430: 8, 1448: 8, 1456: 4, 1470: 8, 1476: 8, 1535: 8
  }],
  CAR.NIRO_EV: [{
  }],
}

ECU_FINGERPRINT = {
  Ecu.fwdCamera: [832, 1156, 1191, 1342]
}

CHECKSUM = {
  "crc8": [CAR.NIRO],
  "6B": [],

FEATURES = {
  "use_cluster_gears": [],  # Use Cluster for Gear Selection, rather than Transmission
  "use_tcu_gears": [],  # Use TCU Message for Gear Selection
  "use_elect_gears": [],
}

DBC = {
  CAR.NIRO: dbc_dict('hyundai_kia_generic', None),
  CAR.NIRO_EV: dbc_dict('hyundai_kia_generic', None),
}

STEER_THRESHOLD = 150
