from types import SimpleNamespace as NS

import pytest

from opendbc.sunnypilot.car.tesla.coop_steering import CoopSteeringCarController
from opendbc.sunnypilot.car.tesla.values import TeslaFlagsSP


@pytest.mark.parametrize('sign', [-1., 1.])
@pytest.mark.parametrize('torque', [-.5, -.07, 0., .07, .5])
@pytest.mark.parametrize('planner_step', [-1., 0., 1.])
def test_released_override_recovers_while_planner_turns(sign, torque, planner_step):
  controller = CoopSteeringCarController()
  controller.angle_override = -sign * 13.
  controller.apply_angle_last = sign * 50.
  controller.coop_apply_angle_sat_last = sign * 37.
  controller.resume_rate_limiter.reset(sign * 50.)
  controller.resume_rate_limiter_delta.reset(5.)
  vm = NS(get_steer_from_curvature=lambda curvature, speed, roll: curvature * 40.)
  cp = NS(flags=TeslaFlagsSP.COOP_STEERING.value)
  cs = NS(out=NS(vEgo=3.4, vEgoRaw=3.4, steeringTorque=torque, steeringAngleDeg=sign * 37.))
  previous = abs(controller.angle_override)
  for frame in range(1, 51):
    last_output = controller.coop_apply_angle_sat_last
    result = controller.update(sign * (50. + frame * planner_step), True, cp, cs, vm)
    assert abs(controller.angle_override) < previous
    assert controller.angle_override * sign <= 0.
    assert abs(result.steeringAngleDeg - last_output) <= 5.
    previous = abs(controller.angle_override)
    cs.out.steeringAngleDeg = result.steeringAngleDeg


@pytest.mark.parametrize('sign', [-1., 1.])
def test_active_driver_still_offsets_planner_and_inactive_resets(sign):
  controller = CoopSteeringCarController()
  vm = NS(get_steer_from_curvature=lambda curvature, speed, roll: curvature * 40.)
  cp = NS(flags=TeslaFlagsSP.COOP_STEERING.value)
  cs = NS(out=NS(vEgo=3.4, vEgoRaw=3.4, steeringTorque=sign, steeringAngleDeg=0.))
  for _ in range(20):
    result = controller.update(0., True, cp, cs, vm)
    cs.out.steeringAngleDeg = result.steeringAngleDeg
  assert controller.angle_override * sign > 0.
  assert result.steeringAngleDeg * sign > 0.
  controller.update(0., False, cp, cs, vm)
  assert controller.angle_override == 0.
