#!/usr/bin/env python3
"""
tests/test_battery_model.py
---------------------------
Pure-Python unit tests for evaluation_pkg.energy_model (Phase 10: battery model
and odometry distance tracking). No ROS 2 / Gazebo dependency.
"""

import math
import os
import sys

import pytest

PKG_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src', 'evaluation_pkg'))
if PKG_DIR not in sys.path:
    sys.path.insert(0, PKG_DIR)

from evaluation_pkg.energy_model import (  # noqa: E402
    BatteryModel,
    BatteryParams,
    DistanceTracker,
    consumed_joules,
)


def test_idle_step_consumes_idle_power_only():
    model = BatteryModel(BatteryParams(idle_power_w=3.0))
    energy = model.step(0.0, 0.0, dt=2.0)
    assert energy == pytest.approx(6.0)
    assert model.consumed_j == pytest.approx(6.0)


def test_driving_step_matches_hand_computation():
    params = BatteryParams(idle_power_w=3.0, k_linear=15.0, k_angular=4.0)
    model = BatteryModel(params)
    # P = 3 + 15*0.3 + 4*0.1 = 7.9 W, dt = 2 s -> 15.8 J
    energy = model.step(0.3, 0.1, dt=2.0)
    assert energy == pytest.approx(15.8)
    assert model.power_w == pytest.approx(7.9)


def test_negative_velocities_use_magnitude():
    a = BatteryModel()
    b = BatteryModel()
    assert a.step(0.3, -0.2, 1.0) == pytest.approx(b.step(-0.3, 0.2, 1.0))


def test_non_positive_dt_is_ignored():
    model = BatteryModel()
    assert model.step(0.3, 0.1, dt=0.0) == 0.0
    assert model.step(0.3, 0.1, dt=-1.0) == 0.0
    assert model.consumed_j == 0.0
    assert model.elapsed_s == 0.0


def test_soc_decreases_monotonically_and_matches_capacity():
    params = BatteryParams(capacity_wh=1.0)  # 3600 J
    model = BatteryModel(params)
    assert model.soc == pytest.approx(1.0)

    previous = model.soc
    for _ in range(10):
        model.step(0.3, 0.0, dt=10.0)
        assert model.soc <= previous
        previous = model.soc

    assert model.soc == pytest.approx(1.0 - model.consumed_j / 3600.0)


def test_soc_is_clamped_at_zero():
    model = BatteryModel(BatteryParams(capacity_wh=0.001))
    model.step(0.3, 0.0, dt=1000.0)
    assert model.soc == 0.0


def test_status_is_consistent():
    params = BatteryParams(capacity_wh=40.0, voltage_v=24.0)
    model = BatteryModel(params)
    model.step(0.3, 0.1, dt=10.0)
    model.add_distance(3.0)

    status = model.status()
    assert 0.0 <= status['percentage'] <= 1.0
    assert status['voltage'] == pytest.approx(24.0)
    assert status['current'] == pytest.approx(model.power_w / 24.0)
    assert status['distance_m'] == pytest.approx(3.0)
    assert status['elapsed_s'] == pytest.approx(10.0)
    consumed_from_charge = (
        (status['capacity_ah'] - status['charge_ah']) * status['voltage'] * 3600.0
    )
    assert consumed_from_charge == pytest.approx(status['consumed_j'])


def test_invalid_params_rejected():
    with pytest.raises(ValueError):
        BatteryModel(BatteryParams(capacity_wh=0.0))
    with pytest.raises(ValueError):
        BatteryModel(BatteryParams(voltage_v=-1.0))


def test_distance_tracker_accumulates_path_length():
    tracker = DistanceTracker()
    assert tracker.update(0.0, 0.0) == 0.0  # first sample only sets the origin
    assert tracker.update(3.0, 4.0) == pytest.approx(5.0)
    assert tracker.update(3.0, 4.0) == pytest.approx(5.0)  # no movement
    assert tracker.update(3.0, 5.0) == pytest.approx(6.0)
    assert tracker.total_m == pytest.approx(6.0)


def test_distance_tracker_reset():
    tracker = DistanceTracker()
    tracker.update(0.0, 0.0)
    tracker.update(1.0, 0.0)
    tracker.reset()
    assert tracker.total_m == 0.0
    assert tracker.update(10.0, 10.0) == 0.0
    assert math.isclose(tracker.update(10.0, 11.0), 1.0)


def test_consumed_joules_round_trips_the_battery_status():
    model = BatteryModel(BatteryParams(capacity_wh=40.0, voltage_v=24.0))
    model.step(0.3, 0.1, dt=25.0)
    status = model.status()
    assert consumed_joules(status['capacity_ah'], status['charge_ah'], status['voltage']) ==         pytest.approx(model.consumed_j)


def test_consumed_joules_never_negative():
    assert consumed_joules(1.0, 1.5, 24.0) == 0.0
