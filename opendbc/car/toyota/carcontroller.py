import copy
import math
from opendbc.car import apply_meas_steer_torque_limits, apply_std_steer_angle_limits, common_fault_avoidance, \
                                        make_tester_present_msg, rate_limit, structs, DT_CTRL
from opendbc.car.can_definitions import CanData
from opendbc.car.common.numpy_fast import clip, interp
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.toyota import toyotacan
from opendbc.car.toyota.values import CAR, STATIC_DSU_MSGS, NO_STOP_TIMER_CAR, TSS2_CAR, \
                                        CarControllerParams, ToyotaFlags, \
                                        UNSUPPORTED_DSU_CAR
from opendbc.can.packer import CANPacker

LongCtrlState = structs.CarControl.Actuators.LongControlState
SteerControlType = structs.CarParams.SteerControlType
VisualAlert = structs.CarControl.HUDControl.VisualAlert
AudibleAlert = structs.CarControl.HUDControl.AudibleAlert

ACCELERATION_DUE_TO_GRAVITY = 9.81  # m/s^2

ACCEL_WINDUP_LIMIT = 0.5  # m/s^2 / frame

# LKA limits
# EPS faults if you apply torque while the steering rate is above 100 deg/s for too long
MAX_STEER_RATE = 100  # deg/s
MAX_STEER_RATE_FRAMES = 18  # tx control frames needed before torque can be cut

# EPS allows user torque above threshold for 50 frames before permanently faulting
MAX_USER_TORQUE = 500

# LTA limits
# EPS ignores commands above this angle and causes PCS to fault
MAX_LTA_ANGLE = 94.9461  # deg
MAX_LTA_DRIVER_TORQUE_ALLOWANCE = 150  # slightly above steering pressed allows some resistance when changing lanes

# PCM compensatory force calculation threshold interpolation values
COMPENSATORY_CALCULATION_THRESHOLD_V = [-0.2, -0.2, -0.05]  # m/s^2
COMPENSATORY_CALCULATION_THRESHOLD_BP = [0., 20., 32.]  # m/s

# resume, lead, and lane lines hysteresis
UI_HYSTERESIS_TIME = 1.  # seconds

class CarController(CarControllerBase):
  def __init__(self, dbc_name, CP):
    super().__init__(dbc_name, CP)
    self.params = CarControllerParams(self.CP)
    self.last_steer = 0
    self.last_angle = 0
    self.alert_active = False
    self.resume_off_frames = 0.
    self.standstill_req = False
    self._standstill_req = False
    self.lead = False
    self.left_lane = False
    self.right_lane = False
    self.steer_rate_counter = 0
    self.prohibit_neg_calculation = True
    self.distance_button = 0

    self.pcm_accel_compensation = 0.0
    self.permit_braking = 0.0

    self.packer = CANPacker(dbc_name)
    self.accel = 0

  def update(self, CC, CS, now_nanos):
    actuators = CC.actuators
    hud_control = CC.hudControl
    pcm_cancel_cmd = CC.cruiseControl.cancel
    lat_active = CC.latActive and abs(CS.out.steeringTorque) < MAX_USER_TORQUE

    # *** control msgs ***
    can_sends = []

    # *** steer torque ***
    new_steer = int(round(actuators.steer * self.params.STEER_MAX))
    apply_steer = apply_meas_steer_torque_limits(new_steer, self.last_steer, CS.out.steeringTorqueEps, self.params)

    # >100 degree/sec steering fault prevention
    self.steer_rate_counter, apply_steer_req = common_fault_avoidance(abs(CS.out.steeringRateDeg) >= MAX_STEER_RATE, lat_active,
                                                                      self.steer_rate_counter, MAX_STEER_RATE_FRAMES)

    if not lat_active:
      apply_steer = 0

    # *** steer angle ***
    if self.CP.steerControlType == SteerControlType.angle:
      # If using LTA control, disable LKA and set steering angle command
      apply_steer = 0
      apply_steer_req = False
      if self.frame % 2 == 0:
        # EPS uses the torque sensor angle to control with, offset to compensate
        apply_angle = actuators.steeringAngleDeg + CS.out.steeringAngleOffsetDeg

        # Angular rate limit based on speed
        apply_angle = apply_std_steer_angle_limits(apply_angle, self.last_angle, CS.out.vEgoRaw, self.params)

        if not lat_active:
          apply_angle = CS.out.steeringAngleDeg + CS.out.steeringAngleOffsetDeg

        self.last_angle = clip(apply_angle, -MAX_LTA_ANGLE, MAX_LTA_ANGLE)

    self.last_steer = apply_steer

    # toyota can trace shows STEERING_LKA at 42Hz, with counter adding alternatively 1 and 2;
    # sending it at 100Hz seem to allow a higher rate limit, as the rate limit seems imposed
    # on consecutive messages
    can_sends.append(toyotacan.create_steer_command(self.packer, apply_steer, apply_steer_req))

    # STEERING_LTA does not seem to allow more rate by sending faster, and may wind up easier
    if self.frame % 2 == 0 and self.CP.carFingerprint in TSS2_CAR:
      lta_active = lat_active and self.CP.steerControlType == SteerControlType.angle
      # cut steering torque with TORQUE_WIND_DOWN when either EPS torque or driver torque is above
      # the threshold, to limit max lateral acceleration and for driver torque blending respectively.
      full_torque_condition = (abs(CS.out.steeringTorqueEps) < self.params.STEER_MAX and
                               abs(CS.out.steeringTorque) < MAX_LTA_DRIVER_TORQUE_ALLOWANCE)

      # TORQUE_WIND_DOWN at 0 ramps down torque at roughly the max down rate of 1500 units/sec
      torque_wind_down = 100 if lta_active and full_torque_condition else 0
      can_sends.append(toyotacan.create_lta_steer_command(self.packer, self.CP.steerControlType, self.last_angle,
                                                          lta_active, self.frame // 2, torque_wind_down))
    # *** gas and brake ***
    # For cars where we allow a higher max acceleration of 2.0 m/s^2, compensate for PCM request overshoot and imprecise braking
    # TODO: sometimes when switching from brake to gas quickly, CLUTCH->ACCEL_NET shows a slow unwind. make it go to 0 immediately
    if self.CP.flags & ToyotaFlags.RAISED_ACCEL_LIMIT:
      if CC.longActive and not CS.out.cruiseState.standstill:
        # calculate amount of acceleration PCM should apply to reach target, given pitch
        accel_due_to_pitch = math.sin(CS.slope_angle) * ACCELERATION_DUE_TO_GRAVITY
        net_acceleration_request = actuators.accel + accel_due_to_pitch

        # let PCM handle stopping for now
        pcm_accel_compensation = 0.0
        if actuators.longControlState != LongCtrlState.stopping:
          pcm_accel_compensation = 2.0 * (CS.pcm_accel_net - net_acceleration_request)

        # prevent compensation windup
        pcm_accel_compensation = clip(pcm_accel_compensation, actuators.accel - self.params.ACCEL_MAX,
                                      actuators.accel - self.params.ACCEL_MIN)

        self.pcm_accel_compensation = rate_limit(pcm_accel_compensation, self.pcm_accel_compensation, -0.01, 0.01)
        pcm_accel_cmd = actuators.accel - self.pcm_accel_compensation

        # Along with rate limiting positive jerk below, this greatly improves gas response time
        # Consider the net acceleration request that the PCM should be applying (pitch included)
        if net_acceleration_request < 0.1:
          self.permit_braking = True
        elif net_acceleration_request > 0.2:
          self.permit_braking = False
      else:
        self.pcm_accel_compensation = 0.0
        pcm_accel_cmd = actuators.accel
        self.permit_braking = True
    else:
      # Set thresholds for compensatory force calculations
      comp_thresh = interp(CS.out.vEgo, COMPENSATORY_CALCULATION_THRESHOLD_BP, COMPENSATORY_CALCULATION_THRESHOLD_V)
      if not CC.longActive:
        self.prohibit_neg_calculation = True
      if CS.pcm_accel_net > comp_thresh:
        self.prohibit_neg_calculation = False
      # Calculate acceleration offset only when allowed
      self.pcm_accel_compensation = CS.pcm_accel_net if CC.longActive and not self.prohibit_neg_calculation else 0.0
      # Compute PCM acceleration command only if long control is active
      pcm_accel_cmd = clip(actuators.accel + self.pcm_accel_compensation, self.params.ACCEL_MIN, self.params.ACCEL_MAX) if CC.longActive and not \
         CS.out.cruiseState.standstill else 0.0
      if pcm_accel_cmd < 0.1:
        self.permit_braking = True
      elif pcm_accel_cmd > 0.2:
        self.permit_braking = False

    # *** standstill logic ***
    # mimic stock behaviour, set standstill_req to False only when openpilot wants to resume
    if not CC.cruiseControl.resume:
        self.resume_off_frames += 1  # frame counter for hysteresis
        # add a 1.5 second hysteresis to when CC.cruiseControl.resume turns off in order to prevent
        # vehicle's dash from blinking
        if self.resume_off_frames >= UI_HYSTERESIS_TIME / DT_CTRL:
            self._standstill_req = True
    else:
        self.resume_off_frames = 0
        self._standstill_req = False
    # ignore standstill on NO_STOP_TIMER_CAR
    self.standstill_req = self._standstill_req and self.CP.carFingerprint not in NO_STOP_TIMER_CAR

    # handle UI messages
    fcw_alert = hud_control.visualAlert == VisualAlert.fcw
    steer_alert = hud_control.visualAlert in VisualAlert.steerRequired and not hud_control.enableVehicleBuzzer
    alert_prompt = hud_control.audibleAlert in (AudibleAlert.promptDistracted, AudibleAlert.prompt) and hud_control.enableVehicleBuzzer
    alert_prompt_repeat = hud_control.audibleAlert in (AudibleAlert.promptRepeat, AudibleAlert.warningSoft) and hud_control.enableVehicleBuzzer
    alert_immediate = hud_control.audibleAlert == AudibleAlert.warningImmediate and hud_control.enableVehicleBuzzer
    cancel_chime = pcm_cancel_cmd and not hud_control.enableVehicleBuzzer

    # *** ui hysteresis ***
    if self.frame % (UI_HYSTERESIS_TIME / DT_CTRL) == 0:
      self.lead = hud_control.leadVisible
      self.left_lane = hud_control.leftLaneVisible
      self.right_lane = hud_control.rightLaneVisible

    # we can spam can to cancel the system even if we are using lat only control
    if (self.frame % 3 == 0 and self.CP.openpilotLongitudinalControl) or pcm_cancel_cmd:
      lead = hud_control.leadVisible or CS.out.vEgo < 12.  # at low speed we always assume the lead is present so ACC can be engaged

      # Press distance button until we are at the correct bar length. Only change while enabled to avoid skipping startup popup
      if self.frame % 6 == 0 and self.CP.openpilotLongitudinalControl:
        desired_distance = 4 - hud_control.leadDistanceBars
        if CS.pcm_follow_distance != desired_distance:
          self.distance_button = not self.distance_button
        else:
          self.distance_button = 0

      # Lexus IS uses a different cancellation message
      if pcm_cancel_cmd and self.CP.carFingerprint in UNSUPPORTED_DSU_CAR:
        can_sends.append(toyotacan.create_acc_cancel_command(self.packer))
      elif self.CP.openpilotLongitudinalControl:
        can_sends.append(toyotacan.create_accel_command(self.packer, pcm_accel_cmd, actuators.accel, self.permit_braking, CS.out.aEgo, CC.longActive,\
                                                        pcm_cancel_cmd, self.standstill_req, lead, CS.acc_type, fcw_alert, self.distance_button))
        self.accel = pcm_accel_cmd
      else:
        can_sends.append(toyotacan.create_accel_command(self.packer, 0, 0, 0, 0, 0, pcm_cancel_cmd, 0, lead, CS.acc_type, 0, self.distance_button))

    # *** hud ui ***
    # usually this is sent at a much lower rate, but no adverse effects has been observed when sent at a much higher rate
    # doing so simplifies carcontroller logic and allows faster response from the vehicle's combination meter
    if self.frame % 3 == 0 and self.CP.carFingerprint != CAR.TOYOTA_PRIUS_V:
      can_sends.append(toyotacan.create_ui_command(self.packer, steer_alert, cancel_chime, self.left_lane,
                                                   self.right_lane, CC.enabled, CS.lkas_hud, CS.lda_left_lane,
                                                   CS.lda_right_lane, CS.sws_beeps, CS.out.lkasEnabled, alert_prompt,
                                                   alert_prompt_repeat, alert_immediate))

    if self.CP.enableDsu or self.CP.flags & ToyotaFlags.DISABLE_RADAR.value:
      can_sends.append(toyotacan.create_fcw_command(self.packer, fcw_alert))

    # *** static msgs ***
    for addr, cars, bus, fr_step, vl in STATIC_DSU_MSGS:
      if self.frame % fr_step == 0 and self.CP.enableDsu and self.CP.carFingerprint in cars:
        can_sends.append(CanData(addr, vl, bus))

    # keep radar disabled
    if self.frame % 20 == 0 and self.CP.flags & ToyotaFlags.DISABLE_RADAR.value:
      can_sends.append(make_tester_present_msg(0x750, 0, 0xF))

    new_actuators = copy.copy(actuators)
    new_actuators.steer = apply_steer / self.params.STEER_MAX
    new_actuators.steerOutputCan = apply_steer
    new_actuators.steeringAngleDeg = self.last_angle
    new_actuators.accel = self.accel

    self.frame += 1
    return new_actuators, can_sends
