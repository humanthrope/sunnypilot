"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import time

import numpy as np

from openpilot.cereal import custom
from opendbc.car.structs import car
from opendbc.car import structs
from openpilot.common.constants import CV
from openpilot.common.params import Params
from openpilot.sunnypilot import PARAMS_UPDATE_PERIOD, get_sanitize_int_param
from openpilot.sunnypilot.selfdrive.car.intelligent_cruise_button_management.helpers import get_minimum_set_speed
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.speed_limit_assist import ACTIVE_STATES as SLA_ACTIVE_STATES
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.helpers import compare_cluster_target, get_speed_limit_offset, \
  speed_limit_ahead_shown
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.common import Mode as SpeedLimitMode, OffsetType

ButtonType = car.CarState.ButtonEvent.Type
SpeedLimitAssistState = custom.LongitudinalPlanSP.SpeedLimit.AssistState

CRUISE_BUTTON_TIMER = {ButtonType.decelCruise: 0, ButtonType.accelCruise: 0,
                       ButtonType.setCruise: 0, ButtonType.resumeCruise: 0,
                       ButtonType.cancel: 0, ButtonType.mainCruise: 0}

V_CRUISE_MIN = 8
V_CRUISE_MAX = 145
V_CRUISE_UNSET = 255


def update_manual_button_timers(CS: car.CarState, button_timers: dict[car.CarState.ButtonEvent.Type, int]) -> None:
  # increment timer for buttons still pressed
  for k in button_timers:
    if button_timers[k] > 0:
      button_timers[k] += 1

  for b in CS.buttonEvents:
    if b.type.raw in button_timers:
      # Start/end timer and store current state on change of button pressed
      button_timers[b.type.raw] = 1 if b.pressed else 0


class VCruiseHelperSP:
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP) -> None:
    self.CP = CP
    self.CP_SP = CP_SP
    self.v_cruise_kph = V_CRUISE_UNSET
    self.v_cruise_cluster_kph = V_CRUISE_UNSET
    self.params = Params()
    self.v_cruise_min = 0
    self.enabled_prev = False

    self.custom_acc_enabled = self.params.get_bool("CustomAccIncrementsEnabled")
    self.short_increment = self.params.get("CustomAccShortPressIncrement", return_default=True)
    self.long_increment = self.params.get("CustomAccLongPressIncrement", return_default=True)

    self.enable_button_timers = CRUISE_BUTTON_TIMER

    # Speed Limit Assist
    self.sla_state = SpeedLimitAssistState.disabled
    self.prev_sla_state = SpeedLimitAssistState.disabled
    self.has_speed_limit = False
    self.speed_limit_final_last = 0.
    self.speed_limit_final_last_kph = 0.
    self.prev_speed_limit_final_last_kph = 0.
    self.req_plus = False
    self.req_minus = False

    # ICBM long press snap to speed limit
    self.speed_limit_snapped = False
    self.speed_limit_last = 0.
    self.speed_limit_ahead_shown = False
    self.speed_limit_ahead = 0.
    self.speed_limit_mode = SpeedLimitMode.off
    self.speed_limit_offset_type = OffsetType.off
    self.speed_limit_offset_value = 0
    self._speed_limit_params_update_time = 0.
    self.read_speed_limit_params()

  def read_custom_set_speed_params(self) -> None:
    self.custom_acc_enabled = self.params.get_bool("CustomAccIncrementsEnabled")
    self.short_increment = self.params.get("CustomAccShortPressIncrement", return_default=True)
    self.long_increment = self.params.get("CustomAccLongPressIncrement", return_default=True)

  def update_speed_limit_params(self) -> None:
    # same update period as SpeedLimitResolver, which applies these to the current speed limit
    if time.monotonic() - self._speed_limit_params_update_time > PARAMS_UPDATE_PERIOD:
      self.read_speed_limit_params()
      self._speed_limit_params_update_time = time.monotonic()

  def read_speed_limit_params(self) -> None:
    self.speed_limit_mode = self.params.get("SpeedLimitMode", return_default=True)
    self.speed_limit_offset_type = get_sanitize_int_param("SpeedLimitOffsetType", OffsetType.min().value, OffsetType.max().value,
                                                          self.params)
    self.speed_limit_offset_value = self.params.get("SpeedLimitValueOffset", return_default=True)

  def update_v_cruise_delta(self, long_press: bool, v_cruise_delta: float) -> tuple[bool, float]:
    if not self.custom_acc_enabled:
      v_cruise_delta = v_cruise_delta * (5 if long_press else 1)
      return long_press, v_cruise_delta

    # Apply user-specified multipliers to the base increment
    short_increment = np.clip(self.short_increment, 1, 10)
    long_increment = np.clip(self.long_increment, 1, 10)

    actual_increment = long_increment if long_press else short_increment
    round_to_nearest = actual_increment in (5, 10)
    v_cruise_delta = v_cruise_delta * actual_increment

    return round_to_nearest, v_cruise_delta

  def get_minimum_set_speed(self, is_metric: bool) -> None:
    if self.CP_SP.pcmCruiseSpeed:
      self.v_cruise_min = V_CRUISE_MIN
      return

    self.v_cruise_min = get_minimum_set_speed(is_metric)

  def update_enabled_state(self, CS: car.CarState, enabled: bool) -> bool:
    # special enabled state for non pcmCruiseSpeed, unchanged for non pcmCruise
    if not self.CP_SP.pcmCruiseSpeed:
      update_manual_button_timers(CS, self.enable_button_timers)
      button_pressed = any(self.enable_button_timers[k] > 0 for k in self.enable_button_timers)

      if enabled and not self.enabled_prev:
        self.enabled_prev = not button_pressed
        enabled = False
      elif not enabled:
        self.enabled_prev = enabled

      return enabled and self.enabled_prev

    return enabled

  def update_speed_limit_assist(self, is_metric, LP_SP: custom.LongitudinalPlanSP,
                                LMD_SP: "custom.LiveMapDataSP | None" = None) -> None:
    resolver = LP_SP.speedLimit.resolver
    self.has_speed_limit = resolver.speedLimitValid or resolver.speedLimitLastValid
    self.speed_limit_final_last = LP_SP.speedLimit.resolver.speedLimitFinalLast
    self.speed_limit_final_last_kph = self.speed_limit_final_last * CV.MS_TO_KPH
    self.sla_state = LP_SP.speedLimit.assist.state
    self.req_plus, self.req_minus = compare_cluster_target(self.v_cruise_cluster_kph * CV.KPH_TO_MS,
                                                           self.speed_limit_final_last, is_metric)

    self.speed_limit_last = resolver.speedLimitLast
    self.speed_limit_ahead = LMD_SP.speedLimitAhead if LMD_SP is not None else 0.
    self.speed_limit_ahead_shown = LMD_SP is not None and speed_limit_ahead_shown(self.speed_limit_mode, self.sla_state,
                                                                                  resolver.source, resolver.speedLimit,
                                                                                  LMD_SP.speedLimitAheadValid, self.speed_limit_ahead)

  @property
  def update_speed_limit_final_last_changed(self) -> bool:
    return self.has_speed_limit and bool(self.speed_limit_final_last_kph != self.prev_speed_limit_final_last_kph)

  def update_speed_limit_assist_pre_active_confirmed(self, button_type: car.CarState.ButtonEvent.Type) -> bool:
    if self.sla_state == SpeedLimitAssistState.preActive or self.prev_sla_state == SpeedLimitAssistState.preActive:
      if button_type == ButtonType.decelCruise and self.req_minus:
        return True
      if button_type == ButtonType.accelCruise and self.req_plus:
        return True

    return False

  def update_v_cruise_speed_limit_snap(self, CS: car.CarState, button_type: car.CarState.ButtonEvent.Type,
                                       long_press: bool, is_metric: bool, v_cruise_delta: float) -> bool:
    """
    ICBM: on a long press, snap the set speed to the speed limit (with offset) if the limit
    is in the direction of the pressed button. If the UI is showing an upcoming speed limit,
    that one is used instead. Further long press repeats in the same hold
    keep the set speed at the speed limit until the button is released.
    Off when the speed limit isn't displayed (SpeedLimitMode off): long presses keep their regular increment.
    v_cruise_delta is the base set speed increment in kph (1 kph or 1 mph), the snap target is the nearest multiple of it.
    Returns True if the set speed change was handled here.
    """
    if not (self.CP.pcmCruise and not self.CP_SP.pcmCruiseSpeed) or not long_press:
      return False

    if self.speed_limit_mode == SpeedLimitMode.off:
      return False

    if self.speed_limit_snapped:
      return True

    if not self.has_speed_limit:
      return False

    # offset applied like SpeedLimitResolver, so the target rounds like SLA's. speedLimitFinalLast is published
    # as Float32, which can round the other way on .5 (65 mph + 10%: SLA 72, Float32 71)
    speed_limit = self.speed_limit_ahead if self.speed_limit_ahead_shown else self.speed_limit_last
    target = speed_limit + get_speed_limit_offset(self.speed_limit_offset_type, self.speed_limit_offset_value, speed_limit, is_metric)

    # back to kph from the displayed value: IMPERIAL_INCREMENT is rounded, n * IMPERIAL_INCREMENT shows as n - 1 from 87 mph
    speed_conv = CV.MS_TO_KPH if is_metric else CV.MS_TO_MPH
    target_kph = round(target * speed_conv) * (1. if is_metric else CV.MPH_TO_KPH)
    target_kph = round(target_kph / v_cruise_delta) * v_cruise_delta
    target_kph = float(np.clip(round(target_kph, 1), self.v_cruise_min, V_CRUISE_MAX))

    # compare as displayed: v_cruise_kph isn't necessarily a multiple of the unit (PCM initial set speed, SLA, gas override)
    req_plus, req_minus = compare_cluster_target(self.v_cruise_kph * CV.KPH_TO_MS, target_kph * CV.KPH_TO_MS, is_metric)
    if not ((button_type == ButtonType.accelCruise and req_plus) or
            (button_type == ButtonType.decelCruise and req_minus)):
      return False

    # same as a regular press: if decel is pressed while overriding, clip cruise speed to minimum of vEgo
    if CS.gasPressed and button_type == ButtonType.decelCruise:
      target_kph = float(np.clip(round(max(target_kph, CS.vEgo * CV.MS_TO_KPH), 1), self.v_cruise_min, V_CRUISE_MAX))

    self.v_cruise_kph = target_kph
    self.speed_limit_snapped = True
    return True

  def update_speed_limit_assist_v_cruise_non_pcm(self) -> None:
    if self.sla_state in SLA_ACTIVE_STATES and (self.prev_sla_state not in SLA_ACTIVE_STATES or
                                                self.update_speed_limit_final_last_changed):
      self.v_cruise_kph = np.clip(round(self.speed_limit_final_last_kph, 1), self.v_cruise_min, V_CRUISE_MAX)

    self.prev_sla_state = self.sla_state
    self.prev_speed_limit_final_last_kph = self.speed_limit_final_last_kph
