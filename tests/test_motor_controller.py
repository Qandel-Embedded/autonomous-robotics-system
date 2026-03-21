"""Unit tests for PID motor controller."""
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))
from motor_controller import PIDController, DifferentialDriveController


def test_pid_zero_error():
    pid = PIDController(kp=1.0, ki=0.0, kd=0.0)
    out = pid.compute(setpoint=5.0, measured=5.0)
    assert abs(out) < 0.01


def test_pid_output_clamped():
    pid = PIDController(kp=100.0, ki=0.0, kd=0.0, output_limits=(-100, 100))
    out = pid.compute(setpoint=1000.0, measured=0.0)
    assert out == 100.0


def test_differential_drive_straight():
    ctrl = DifferentialDriveController()
    left, right = ctrl.set_velocity(linear_mps=1.0, angular_rads=0.0)
    assert abs(left - right) < 0.001


def test_differential_drive_turn():
    ctrl = DifferentialDriveController()
    left, right = ctrl.set_velocity(linear_mps=0.0, angular_rads=1.0)
    assert left < 0 and right > 0
