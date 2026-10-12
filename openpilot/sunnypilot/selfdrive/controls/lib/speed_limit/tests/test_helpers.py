"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from openpilot.common.constants import CV
from openpilot.common.parameterized import parameterized
from openpilot.common.test import OpenpilotTestCase

from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.common import Mode as SpeedLimitMode, OffsetType
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.helpers import AssistState, SpeedLimitSource, get_speed_limit_offset, \
  speed_limit_ahead_shown

SHOWN = {
  "mode": SpeedLimitMode.information,
  "assist_state": AssistState.active,
  "source": SpeedLimitSource.map,
  "speed_limit": 25.,
  "speed_limit_ahead_valid": True,
  "speed_limit_ahead": 30.,
}


class TestSpeedLimitAheadShown(OpenpilotTestCase):
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


class TestGetSpeedLimitOffset(OpenpilotTestCase):
  @parameterized.expand([
    (OffsetType.off, 5, True, 0.),
    (OffsetType.fixed, 5, True, 5 * CV.KPH_TO_MS),
    (OffsetType.fixed, 5, False, 5 * CV.MPH_TO_MS),
    (OffsetType.percentage, 10, False, 2.5),  # 10% of 25 m/s, independent of units
    (OffsetType.percentage, -10, True, -2.5),
  ])
  def test_offset(self, offset_type, offset_value, is_metric, expected):
    self.assertAlmostEqual(get_speed_limit_offset(offset_type, offset_value, 25., is_metric), expected)

  def test_unsupported_offset_type(self):
    with self.assertRaises(NotImplementedError):
      get_speed_limit_offset(3, 5, 25., True)
