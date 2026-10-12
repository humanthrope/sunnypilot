"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from openpilot.cereal import custom
from opendbc.car.structs import car
from openpilot.common.constants import CV
from openpilot.common.params import Params
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.common import Mode as SpeedLimitMode

AssistState = custom.LongitudinalPlanSP.SpeedLimit.AssistState
SpeedLimitSource = custom.LongitudinalPlanSP.SpeedLimit.Source


def speed_limit_ahead_shown(mode: SpeedLimitMode, assist_state: custom.LongitudinalPlanSP.SpeedLimit.AssistState,
                            source: custom.LongitudinalPlanSP.SpeedLimit.Source, speed_limit: float,
                            speed_limit_ahead_valid: bool, speed_limit_ahead: float) -> bool:
  """
  Whether the onroad UI shows the upcoming speed limit ("AHEAD"). ICBM snaps the set speed to it when shown.
  speed_limit and speed_limit_ahead can be in any unit, as long as it's the same for both.
  """
  return (mode != SpeedLimitMode.off and
          assist_state != AssistState.preActive and
          source == SpeedLimitSource.map and
          speed_limit_ahead_valid and
          speed_limit_ahead > 0 and
          speed_limit_ahead != speed_limit)


def compare_cluster_target(v_cruise_cluster: float, target_set_speed: float, is_metric: bool) -> tuple[bool, bool]:
  speed_conv = CV.MS_TO_KPH if is_metric else CV.MS_TO_MPH
  v_cruise_cluster_conv = round(v_cruise_cluster * speed_conv)
  target_set_speed_conv = round(target_set_speed * speed_conv)

  req_plus = v_cruise_cluster_conv < target_set_speed_conv
  req_minus = v_cruise_cluster_conv > target_set_speed_conv

  return req_plus, req_minus


def set_speed_limit_assist_availability(CP: car.CarParams, CP_SP: custom.CarParamsSP, params: Params | None = None) -> bool:
  if params is None:
    params = Params()

  is_release = params.get_bool("IsReleaseSpBranch")
  disallow_in_release = CP.brand == "tesla" and is_release
  always_disallow = CP.brand == "rivian"
  allowed = True

  if disallow_in_release or always_disallow:
    allowed = False

  if not CP.openpilotLongitudinalControl and CP_SP.pcmCruiseSpeed:
    allowed = False

  if not allowed:
    if params.get("SpeedLimitMode", return_default=True) == SpeedLimitMode.assist:
      params.put("SpeedLimitMode", int(SpeedLimitMode.warning), block=True)

  return allowed
