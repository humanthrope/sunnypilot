from opendbc.car.structs import car
from openpilot.cereal import custom
from openpilot.common.constants import CV
from openpilot.common.parameterized import parameterized
from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.car.cruise import VCruiseHelper, CRUISE_LONG_PRESS, IMPERIAL_INCREMENT
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.common import Mode as SpeedLimitMode, OffsetType

ButtonEvent = car.CarState.ButtonEvent
ButtonType = car.CarState.ButtonEvent.Type
SpeedLimitSource = custom.LongitudinalPlanSP.SpeedLimit.Source


class TestIcbmSpeedLimitSnap(OpenpilotTestCase):
  def setup_method(self):
    # ICBM: pcmCruise with openpilot-controlled set speed
    self.CP = car.CarParams(pcmCruise=True)
    self.CP_SP = custom.CarParamsSP(pcmCruiseSpeed=False)
    self.v_cruise_helper = VCruiseHelper(self.CP, self.CP_SP)
    self.v_cruise_helper.custom_acc_enabled = False
    self.v_cruise_helper.speed_limit_mode = SpeedLimitMode.information
    self.v_cruise_helper.speed_limit_offset_type = OffsetType.off

  def set_speed_limit(self, speed_limit_mph: float | None, ahead_mph: float | None = None,
                      source=SpeedLimitSource.map) -> None:
    lp_sp = custom.LongitudinalPlanSP.new_message()
    lmd_sp = custom.LiveMapDataSP.new_message()
    if speed_limit_mph is not None:
      resolver = lp_sp.speedLimit.resolver
      resolver.speedLimitValid = True
      resolver.source = source
      resolver.speedLimit = speed_limit_mph * CV.MPH_TO_MS
      offset = self.v_cruise_helper.get_speed_limit_offset(resolver.speedLimit, False)
      resolver.speedLimitFinalLast = resolver.speedLimit + offset
    if ahead_mph is not None:
      lmd_sp.speedLimitAheadValid = True
      lmd_sp.speedLimitAhead = ahead_mph * CV.MPH_TO_MS
      lmd_sp.speedLimitAheadDistance = 300.
    self.v_cruise_helper.update_speed_limit_assist(False, lp_sp, lmd_sp)

  def engage(self, set_speed_mph: float) -> None:
    # PCM provides the initial set speed, then ICBM takes over on the next frame without buttons
    CS = car.CarState(cruiseState={"available": True, "speed": set_speed_mph * CV.MPH_TO_MS,
                                   "speedCluster": set_speed_mph * CV.MPH_TO_MS})
    for _ in range(3):
      self.v_cruise_helper.update_v_cruise(CS, enabled=True, is_metric=False)
    # align to the imperial increment so long press rounding is deterministic
    self.v_cruise_helper.v_cruise_kph = round(set_speed_mph * IMPERIAL_INCREMENT, 1)

  def hold(self, button_type: car.CarState.ButtonEvent.Type, frames: int) -> None:
    CS = car.CarState(cruiseState={"available": True})
    CS.buttonEvents = [ButtonEvent(type=button_type, pressed=True)]
    self.v_cruise_helper.update_v_cruise(CS, enabled=True, is_metric=False)
    CS.buttonEvents = []
    for _ in range(frames - 1):
      self.v_cruise_helper.update_v_cruise(CS, enabled=True, is_metric=False)
    CS.buttonEvents = [ButtonEvent(type=button_type, pressed=False)]
    self.v_cruise_helper.update_v_cruise(CS, enabled=True, is_metric=False)

  @property
  def set_speed_mph(self) -> int:
    return round(self.v_cruise_helper.v_cruise_kph * CV.KPH_TO_MPH)

  @parameterized.expand([
    (ButtonType.accelCruise, 45, 65, 65),  # limit above: + snaps up
    (ButtonType.decelCruise, 65, 45, 45),  # limit below: - snaps down
    (ButtonType.accelCruise, 65, 45, 70),  # limit below: + does a normal long press
    (ButtonType.decelCruise, 45, 65, 40),  # limit above: - does a normal long press
  ])
  def test_long_press_snap(self, button_type, start_mph, limit_mph, expected_mph):
    self.engage(start_mph)
    self.set_speed_limit(limit_mph)
    self.hold(button_type, CRUISE_LONG_PRESS + 1)
    assert self.set_speed_mph == expected_mph

  def test_short_press_does_not_snap(self):
    self.engage(45)
    self.set_speed_limit(65)
    self.hold(ButtonType.accelCruise, 2)
    assert self.set_speed_mph == 46

  def test_no_speed_limit(self):
    self.engage(45)
    self.set_speed_limit(None)
    self.hold(ButtonType.accelCruise, CRUISE_LONG_PRESS + 1)
    assert self.set_speed_mph == 50

  def test_continued_hold_stays_at_limit(self):
    self.engage(45)
    self.set_speed_limit(65)
    self.hold(ButtonType.accelCruise, CRUISE_LONG_PRESS * 3 + 1)
    assert self.set_speed_mph == 65

  def test_new_hold_after_snap_continues(self):
    self.engage(45)
    self.set_speed_limit(65)
    self.hold(ButtonType.accelCruise, CRUISE_LONG_PRESS + 1)
    self.hold(ButtonType.accelCruise, CRUISE_LONG_PRESS + 1)
    assert self.set_speed_mph == 70

  @parameterized.expand([
    (ButtonType.accelCruise, 45, 55, 65, 65),  # upcoming higher limit is used
    (ButtonType.decelCruise, 65, 55, 45, 45),  # upcoming lower limit is used
    (ButtonType.accelCruise, 45, 35, 55, 55),  # current limit below, upcoming above
    (ButtonType.decelCruise, 45, 35, 55, 40),  # upcoming limit above: normal long press
  ])
  def test_long_press_snap_to_ahead(self, button_type, start_mph, limit_mph, ahead_mph, expected_mph):
    self.engage(start_mph)
    self.set_speed_limit(limit_mph, ahead_mph)
    self.hold(button_type, CRUISE_LONG_PRESS + 1)
    assert self.set_speed_mph == expected_mph

  @parameterized.expand([
    (SpeedLimitMode.off, SpeedLimitSource.map),
    (SpeedLimitMode.information, SpeedLimitSource.car),
  ])
  def test_ahead_not_shown_uses_current_limit(self, mode, source):
    # the UI hides the upcoming limit when speed limit mode is off or the source isn't map data
    self.v_cruise_helper.speed_limit_mode = mode
    self.engage(45)
    self.set_speed_limit(55, 65, source=source)
    self.hold(ButtonType.accelCruise, CRUISE_LONG_PRESS + 1)
    assert self.set_speed_mph == 55

  @parameterized.expand([
    (OffsetType.fixed, 5, 70),        # 65 + 5
    (OffsetType.percentage, 10, 72),  # 65 * 1.1 = 71.5
  ])
  def test_ahead_snap_includes_offset(self, offset_type, offset_value, expected_mph):
    self.v_cruise_helper.speed_limit_offset_type = offset_type
    self.v_cruise_helper.speed_limit_offset_value = offset_value
    self.engage(45)
    self.set_speed_limit(55, 65)
    self.hold(ButtonType.accelCruise, CRUISE_LONG_PRESS + 1)
    assert self.set_speed_mph == expected_mph
