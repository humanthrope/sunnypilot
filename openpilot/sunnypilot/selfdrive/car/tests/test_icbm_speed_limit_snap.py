from opendbc.car.structs import car
from openpilot.cereal import custom
from openpilot.common.constants import CV
from openpilot.common.parameterized import parameterized
from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.car.cruise import VCruiseHelper, CRUISE_LONG_PRESS, IMPERIAL_INCREMENT

ButtonEvent = car.CarState.ButtonEvent
ButtonType = car.CarState.ButtonEvent.Type


class TestIcbmSpeedLimitSnap(OpenpilotTestCase):
  def setup_method(self):
    # ICBM: pcmCruise with openpilot-controlled set speed
    self.CP = car.CarParams(pcmCruise=True)
    self.CP_SP = custom.CarParamsSP(pcmCruiseSpeed=False)
    self.v_cruise_helper = VCruiseHelper(self.CP, self.CP_SP)
    self.v_cruise_helper.custom_acc_enabled = False

  def set_speed_limit(self, speed_limit_mph: float | None) -> None:
    lp_sp = custom.LongitudinalPlanSP.new_message()
    if speed_limit_mph is not None:
      lp_sp.speedLimit.resolver.speedLimitValid = True
      lp_sp.speedLimit.resolver.speedLimitFinalLast = speed_limit_mph * CV.MPH_TO_MS
    self.v_cruise_helper.update_speed_limit_assist(False, lp_sp)

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
