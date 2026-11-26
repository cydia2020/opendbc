import copy
import math

from opendbc.can import CANDefine, CANParser
from opendbc.car import Bus, DT_CTRL, create_button_events, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.common.filter_simple import FirstOrderFilter
from opendbc.car.interfaces import CarStateBase
from opendbc.car.toyota.values import ToyotaFlags, CAR, DBC, STEER_THRESHOLD, EPS_SCALE

ButtonType = structs.CarState.ButtonEvent.Type
SteerControlType = structs.CarParams.SteerControlType

# These steering fault definitions seem to be common across LKA (torque) and LTA (angle):
# - high steer rate fault: goes to 21 or 25 for 1 frame, then 9 for 2 seconds
# - lka/lta msg drop out: goes to 9 then 11 for a combined total of 2 seconds, then 3.
#     if using the other control command, goes directly to 3 after 1.5 seconds
# - initializing: LTA can report 0 as long as STEER_TORQUE_SENSOR->STEER_ANGLE_INITIALIZING is 1,
#     and is a catch-all for LKA
TEMP_STEER_FAULTS = (0, 9, 11, 21, 25)
# - lka/lta msg drop out: 3 (recoverable)
# - prolonged high driver torque: 17 (permanent)
PERM_STEER_FAULTS = (3, 17)


def _fresh_signal(cp, message, signal, max_age_s):
  # Use the parser's CAN/replay clock and only read registered signals.
  timestamp = cp.ts_nanos.get(message, {}).get(signal, 0)
  age = cp._last_update_nanos - timestamp
  if timestamp <= 0 or not 0 <= age <= max_age_s * 1e9:
    return None
  value = cp.vl.get(message, {}).get(signal)
  return value if value is not None and math.isfinite(value) else None


class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    can_define = CANDefine(DBC[CP.carFingerprint][Bus.pt])
    self.eps_torque_scale = EPS_SCALE[CP.carFingerprint] / 100.
    self.cluster_speed_hyst_gap = CV.KPH_TO_MS / 2.
    self.cluster_min_speed = CV.KPH_TO_MS / 2.

    if CP.flags & ToyotaFlags.SECOC.value:
      self.shifter_values = can_define.dv["GEAR_PACKET_HYBRID"]["GEAR"]
    else:
      self.shifter_values = can_define.dv["GEAR_PACKET"]["GEAR"]

    # On cars with cp.vl["STEER_TORQUE_SENSOR"]["STEER_ANGLE"]
    # the signal is zeroed to where the steering angle is at start.
    # Need to apply an offset as soon as the steering angle measurements are both received
    self.accurate_steer_angle_seen = False
    self.angle_offset = FirstOrderFilter(None, 60.0, DT_CTRL, initialized=False)
    self.secondary_steer_lost = False

    self.lkas_button = 0
    self.distance_button = 0

    self.pcm_follow_distance = 0

    self.acc_type = 1
    self.lkas_hud = {}
    self.gvc = 0.0
    self.secoc_synchronization = None

  def update(self, can_parsers) -> structs.CarState:
    cp = can_parsers[Bus.pt]
    cp_cam = can_parsers[Bus.cam]

    ret = structs.CarState()
    dsu_rerouted = bool(self.CP.flags & ToyotaFlags.DSU_REROUTED)
    camera_acc = dsu_rerouted or bool(self.CP.flags & ToyotaFlags.TSS2 and not self.CP.flags & ToyotaFlags.RADAR_ACC)
    cp_acc = cp_cam if camera_acc else cp

    if not self.CP.flags & ToyotaFlags.SECOC.value:
      self.gvc = cp.vl["VSC1S07"]["GVC"]

    ret.doorOpen = any([cp.vl["BODY_CONTROL_STATE"]["DOOR_OPEN_FL"], cp.vl["BODY_CONTROL_STATE"]["DOOR_OPEN_FR"],
                        cp.vl["BODY_CONTROL_STATE"]["DOOR_OPEN_RL"], cp.vl["BODY_CONTROL_STATE"]["DOOR_OPEN_RR"]])
    ret.seatbeltUnlatched = cp.vl["BODY_CONTROL_STATE"]["SEATBELT_DRIVER_UNLATCHED"] != 0
    ret.parkingBrake = cp.vl["BODY_CONTROL_STATE"]["PARKING_BRAKE"] == 1

    ret.brakePressed = cp.vl["BRAKE_MODULE"]["BRAKE_PRESSED"] != 0
    ret.brakeHoldActive = cp.vl["ESP_CONTROL"]["BRAKE_HOLD_ACTIVE"] == 1

    if self.CP.flags & ToyotaFlags.SECOC.value:
      self.secoc_synchronization = copy.copy(cp.vl["SECOC_SYNCHRONIZATION"])
      ret.gasPressed = cp.vl["GAS_PEDAL"]["GAS_PEDAL_USER"] > 0
      can_gear = int(cp.vl["GEAR_PACKET_HYBRID"]["GEAR"])
    else:
      ret.gasPressed = cp.vl["PCM_CRUISE"]["GAS_RELEASED"] == 0
      can_gear = int(cp.vl["GEAR_PACKET"]["GEAR"])
      if not self.CP.flags & ToyotaFlags.DISABLE_RADAR.value:
        ret.stockAeb = bool(cp_acc.vl["PRE_COLLISION"]["PRECOLLISION_ACTIVE"] and cp_acc.vl["PRE_COLLISION"]["FORCE"] < -1e-5)

    wheel_speeds = [cp.vl["WHEEL_SPEEDS"][f"WHEEL_SPEED_{whl}"] for whl in ("FL", "FR", "RL", "RR")]
    self.parse_wheel_speeds(ret, *wheel_speeds)
    ret.vEgoCluster = ret.vEgo * 1.015  # minimum of all the cars

    # moving if any wheel turns, either way, like the safety
    ret.standstill = all(abs(speed) < 1e-3 for speed in wheel_speeds)

    ret.vehicleSensorsInvalid = any(cp.vl["WHEEL_SPEEDS"][f"WHEEL_SPEED_{whl}_FAULT"]
                                    for whl in ("FL", "FR", "RL", "RR"))

    ret.steeringAngleDeg = cp.vl["STEER_ANGLE_SENSOR"]["STEER_ANGLE"] + cp.vl["STEER_ANGLE_SENSOR"]["STEER_FRACTION"]
    ret.steeringRateDeg = cp.vl["STEER_ANGLE_SENSOR"]["STEER_RATE"]
    secondary_steer_fault = False
    if self.CP.flags & ToyotaFlags.SECONDARY_STEER_ANGLE:
      secondary_steer_fault = self._update_secondary_steering(cp, ret)
    else:
      torque_sensor_angle_deg = cp.vl["STEER_TORQUE_SENSOR"]["STEER_ANGLE"]

      # On some cars, the angle measurement is non-zero while initializing
      if abs(torque_sensor_angle_deg) > 1e-3 and not bool(cp.vl["STEER_TORQUE_SENSOR"]["STEER_ANGLE_INITIALIZING"]):
        self.accurate_steer_angle_seen = True

      if self.accurate_steer_angle_seen:
        # Offset seems to be invalid for large steering angles and high angle rates
        if abs(ret.steeringAngleDeg) < 90 and abs(ret.steeringRateDeg) < 100 and cp.can_valid:
          self.angle_offset.update(torque_sensor_angle_deg - ret.steeringAngleDeg)

        if self.angle_offset.initialized:
          ret.steeringAngleOffsetDeg = self.angle_offset.x
          ret.steeringAngleDeg = torque_sensor_angle_deg - self.angle_offset.x

    ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(can_gear, None))
    ret.leftBlinker = cp.vl["BLINKERS_STATE"]["TURN_SIGNALS"] == 1
    ret.rightBlinker = cp.vl["BLINKERS_STATE"]["TURN_SIGNALS"] == 2

    ret.steeringTorque = cp.vl["STEER_TORQUE_SENSOR"]["STEER_TORQUE_DRIVER"]
    ret.steeringTorqueEps = cp.vl["STEER_TORQUE_SENSOR"]["STEER_TORQUE_EPS"] * self.eps_torque_scale
    # we could use the override bit from dbc, but it's triggered at too high torque values
    ret.steeringPressed = abs(ret.steeringTorque) > STEER_THRESHOLD

    # Check EPS LKA/LTA fault status
    ret.steerFaultTemporary = secondary_steer_fault or cp.vl["EPS_STATUS"]["LKA_STATE"] in TEMP_STEER_FAULTS
    ret.steerFaultPermanent = cp.vl["EPS_STATUS"]["LKA_STATE"] in PERM_STEER_FAULTS

    if self.CP.steerControlType == SteerControlType.angle:
      ret.steerFaultTemporary = ret.steerFaultTemporary or cp.vl["EPS_STATUS"]["LTA_STATE"] in TEMP_STEER_FAULTS
      ret.steerFaultPermanent = ret.steerFaultPermanent or cp.vl["EPS_STATUS"]["LTA_STATE"] in PERM_STEER_FAULTS

      # Lane Tracing Assist control is unavailable (EPS_STATUS->LTA_STATE=0) until
      # the more accurate angle sensor signal is initialized
      if not self.accurate_steer_angle_seen:
        ret.vehicleSensorsInvalid = True

    if self.CP.flags & ToyotaFlags.UNSUPPORTED_DSU:
      # TODO: find the bit likely in DSU_CRUISE that describes an ACC fault. one may also exist in CLUTCH
      ret.cruiseState.available = cp.vl["DSU_CRUISE"]["MAIN_ON"] != 0
      ret.cruiseState.speed = cp.vl["DSU_CRUISE"]["SET_SPEED"] * CV.KPH_TO_MS
      cluster_set_speed = cp.vl["PCM_CRUISE_ALT"]["UI_SET_SPEED"]
    else:
      ret.accFaulted = cp.vl["PCM_CRUISE_2"]["ACC_FAULTED"] != 0
      ret.carFaultedNonCritical = cp.vl["PCM_CRUISE_SM"]["TEMP_ACC_FAULTED"] != 0
      ret.cruiseState.available = cp.vl["PCM_CRUISE_2"]["MAIN_ON"] != 0
      ret.cruiseState.speed = cp.vl["PCM_CRUISE_2"]["SET_SPEED"] * CV.KPH_TO_MS
      cluster_set_speed = cp.vl["PCM_CRUISE_SM"]["UI_SET_SPEED"]

    # UI_SET_SPEED is always non-zero when main is on, hide until first enable
    is_metric = cp.vl["BODY_CONTROL_STATE_2"]["UNITS"] in (1, 2)
    if ret.cruiseState.speed != 0:
      conversion_factor = CV.KPH_TO_MS if is_metric else CV.MPH_TO_MS
      ret.cruiseState.speedCluster = cluster_set_speed * conversion_factor

    if (dsu_rerouted or self.CP.flags & ToyotaFlags.TSS2) and not self.CP.flags & ToyotaFlags.DISABLE_RADAR.value:
      # Preserve the original private fork's TSS-P ACC type.
      if not dsu_rerouted:
        self.acc_type = cp_acc.vl["ACC_CONTROL"]["ACC_TYPE"]
      ret.stockFcw = bool(cp_acc.vl["PCS_HUD"]["FCW"])

    # some TSS2 cars have low speed lockout permanently set, so ignore on those cars
    # these cars are identified by an ACC_TYPE value of 2.
    # TODO: it is possible to avoid the lockout and gain stop and go if you
    # send your own ACC_CONTROL msg on startup with ACC_TYPE set to 1
    if (not (self.CP.flags & ToyotaFlags.TSS2) and not (self.CP.flags & ToyotaFlags.UNSUPPORTED_DSU)) or \
       (self.CP.flags & ToyotaFlags.TSS2 and self.acc_type == 1):
      if self.CP.openpilotLongitudinalControl:
        ret.accFaulted = ret.accFaulted or cp.vl["PCM_CRUISE_2"]["LOW_SPEED_LOCKOUT"] == 2

    pcm_acc_status = cp.vl["PCM_CRUISE"]["CRUISE_STATE"]
    ret.cruiseState.standstill = pcm_acc_status == 7
    ret.cruiseState.enabled = bool(cp.vl["PCM_CRUISE"]["CRUISE_ACTIVE"])
    ret.cruiseState.nonAdaptive = pcm_acc_status in (1, 2, 3, 4, 5, 6)

    ret.espDisabled = cp.vl["ESP_CONTROL"]["TC_DISABLED"] != 0

    if self.CP.flags & ToyotaFlags.HAS_BSM:
      ret.leftBlindspot = (cp.vl["BSM"]["L_ADJACENT"] == 1) or (cp.vl["BSM"]["L_APPROACHING"] == 1)
      ret.rightBlindspot = (cp.vl["BSM"]["R_ADJACENT"] == 1) or (cp.vl["BSM"]["R_APPROACHING"] == 1)

    if self.CP.carFingerprint != CAR.TOYOTA_PRIUS_V:
      self.lkas_hud = copy.copy(cp_cam.vl["LKAS_HUD"])

    if not (self.CP.flags & ToyotaFlags.UNSUPPORTED_DSU):
      self.pcm_follow_distance = cp.vl["PCM_CRUISE_2"]["PCM_FOLLOW_DISTANCE"]

    buttonEvents = []
    if dsu_rerouted or self.CP.flags & ToyotaFlags.TSS2:
      # lkas button is wired to the camera
      prev_lkas_button = self.lkas_button
      self.lkas_button = cp_cam.vl["LKAS_HUD"]["LDA_ON_MESSAGE"]

      # Cycles between 1 and 2 when pressing the button, then rests back at 0 after ~3s
      if self.lkas_button != 0 and self.lkas_button != prev_lkas_button:
        buttonEvents.extend(create_button_events(1, 0, {1: ButtonType.lkas}) +
                            create_button_events(0, 1, {1: ButtonType.lkas}))

      if not (self.CP.flags & (ToyotaFlags.RADAR_ACC | ToyotaFlags.SECOC)):
        # distance button is wired to the ACC module (camera or radar)
        prev_distance_button = self.distance_button
        self.distance_button = cp_acc.vl["ACC_CONTROL"]["DISTANCE"]

        buttonEvents += create_button_events(self.distance_button, prev_distance_button, {1: ButtonType.gapAdjustCruise})

    ret.buttonEvents = buttonEvents
    self._update_private_telemetry(cp, ret)
    return ret

  def _update_secondary_steering(self, cp, ret):
    # Keep the factory steering rate, torque and EPS faults. The high-resolution
    # sensor replaces only the angle after the factory-referenced offset exists.
    msg = cp.dbc.name_to_msg["SECONDARY_STEER_ANGLE"]
    checksum = msg.sigs.get("CHECKSUM")
    angle = _fresh_signal(cp, msg.name, "STEER_ANGLE", 0.1)
    valid = checksum is not None and checksum.calc_checksum is not None and angle is not None
    if not valid:
      # Once used, do not automatically switch back to this sensor after loss:
      # a restarted sensor may have a different zero. Require card restart.
      self.secondary_steer_lost |= self.angle_offset.initialized
      return True
    if self.secondary_steer_lost:
      return True

    self.accurate_steer_angle_seen = True  # a valid zero is a real measurement
    factory_fresh = all(_fresh_signal(cp, "STEER_ANGLE_SENSOR", sig, 0.1) is not None
                        for sig in ("STEER_ANGLE", "STEER_FRACTION", "STEER_RATE"))
    if not factory_fresh:
      self.secondary_steer_lost |= self.angle_offset.initialized
      return True
    if abs(ret.steeringAngleDeg) < 90 and abs(ret.steeringRateDeg) < 100 and cp.can_valid:
      self.angle_offset.update(angle - ret.steeringAngleDeg)
    if not self.angle_offset.initialized:
      return True

    ret.steeringAngleOffsetDeg = self.angle_offset.x
    ret.steeringAngleDeg = angle - self.angle_offset.x
    return False

  @staticmethod
  def _update_private_telemetry(cp, ret):
    def display_sample(message, signal, min_age=0.25, max_age=1.0):
      # Match the parser's measured message frequency, allowing for jitter.
      # This tolerance is only for display telemetry, not driving inputs.
      msg = cp.dbc.name_to_msg.get(message)
      state = cp.message_states.get(msg.address) if msg is not None else None
      frequency = state.frequency if state is not None else 0.0
      age_limit = max(min_age, min(max_age, 2.5 / frequency)) if frequency > 0 else min_age
      return _fresh_signal(cp, message, signal, age_limit)

    acc_brake = display_sample("ESP_CONTROL", "BRAKE_LIGHTS_ACC")
    pedal_brake = display_sample("BRAKE_MODULE", "BRAKE_PRESSED")
    # A fresh positive on either input is sufficient. A confirmed release
    # requires both inputs to be fresh and zero.
    if acc_brake or pedal_brake:
      ret.brakeLights = True
      ret.brakeLightsValid = True
    elif acc_brake is not None and pedal_brake is not None:
      ret.brakeLights = False
      ret.brakeLightsValid = True

    dimmed = display_sample("BODY_CONTROL_STATE", "METER_DIMMED", 2.0, 10.0)
    slider = display_sample("BODY_CONTROL_STATE_2", "METER_SLIDER_BRIGHTNESS_PCT", 2.0, 10.0)
    low = display_sample("BODY_CONTROL_STATE_2", "METER_SLIDER_LOW_BRIGHTNESS", 2.0, 10.0)
    if dimmed == 1 and slider is not None and 0 <= slider <= 100:
      ret.meterBrightness = slider * 0.5
      ret.meterBrightnessValid = True
    elif dimmed == 0 and low in (0, 1):
      ret.meterBrightness = 1.0 if low else 100.0
      ret.meterBrightnessValid = True

    force = _fresh_signal(cp, "PCM_CRUISE", "NEUTRAL_FORCE", 0.25)
    if force is not None:
      ret.pcmNeutralForce = force
      ret.pcmNeutralForceValid = True

  @staticmethod
  def get_can_parsers(CP):
    pt_messages = [
      ("BLINKERS_STATE", float('nan')),
    ]
    if CP.flags & ToyotaFlags.SECONDARY_STEER_ANGLE:
      # Required input on this private platform. Learn the actual frequency;
      # the angle selection additionally checks the accepted sample's age.
      pt_messages.append(("SECONDARY_STEER_ANGLE", 0))

    return {
      Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], pt_messages, 0),
      Bus.cam: CANParser(DBC[CP.carFingerprint][Bus.pt], [], 2),
    }
