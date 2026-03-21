"""PID Motor Controller for differential drive robot."""
import time


class PIDController:
    def __init__(self, kp, ki, kd, output_limits=(-100, 100)):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.integral = 0.0
        self.prev_error = 0.0
        self.limits = output_limits
        self.prev_time = time.time()

    def compute(self, setpoint, measured):
        now = time.time()
        dt = max(now - self.prev_time, 1e-6)
        error = setpoint - measured
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        output = self.kp*error + self.ki*self.integral + self.kd*derivative
        output = max(self.limits[0], min(self.limits[1], output))
        self.prev_error = error
        self.prev_time = now
        return output


class DifferentialDriveController:
    """Controls left/right motors for differential drive robot."""

    WHEEL_BASE = 0.30   # metres

    def __init__(self):
        self.left  = PIDController(kp=2.0, ki=0.5, kd=0.1)
        self.right = PIDController(kp=2.0, ki=0.5, kd=0.1)

    def set_velocity(self, linear_mps, angular_rads):
        v_left  = linear_mps - (angular_rads * self.WHEEL_BASE / 2)
        v_right = linear_mps + (angular_rads * self.WHEEL_BASE / 2)
        return v_left, v_right
