"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from openpilot.common.parameterized import parameterized

from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.common import Mode as SpeedLimitMode
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.helpers import AssistState, SpeedLimitSource, speed_limit_ahead_shown

SHOWN = {
  "mode": SpeedLimitMode.information,
  "assist_state": AssistState.active,
  "source": SpeedLimitSource.map,
  "speed_limit": 25.,
  "speed_limit_ahead_valid": True,
  "speed_limit_ahead": 30.,
}


class TestSpeedLimitAheadShown:
  def test_shown(self):
    assert speed_limit_ahead_shown(**SHOWN)

  @parameterized.expand([
    ("mode_off", {"mode": SpeedLimitMode.off}),
    ("pre_active", {"assist_state": AssistState.preActive}),
    ("source_car", {"source": SpeedLimitSource.car}),
    ("source_none", {"source": SpeedLimitSource.none}),
    ("ahead_invalid", {"speed_limit_ahead_valid": False}),
    ("ahead_zero", {"speed_limit_ahead": 0.}),
    ("ahead_same_as_current", {"speed_limit_ahead": 25.}),
  ])
  def test_not_shown(self, _, overrides):
    assert not speed_limit_ahead_shown(**(SHOWN | overrides))
