import numpy as np

class PID_class:
    def __init__(self, kp=0.5, ki=0.01, kd=0.1):
        # PID gains
        self.kp = kp
        self.ki = ki
        self.kd = kd

        # Internal state
        self.integral_sum = 0.0
        self.previous_error = 0.0
        self.last_input = 0.0

        # Limits
        self.output_limits = (-1.0, 1.0)
        self.integral_limits = (-10.0, 10.0)

        # Reset flags
        self.reset_integral = False

    def update_gains(self, kp=None, ki=None, kd=None):
        if kp is not None: self.kp = kp
        if ki is not None: self.ki = ki
        if kd is not None: self.kd = kd

    def set_output_limits(self, lower, upper):
        assert lower < upper, "Lower output limit must be less than upper"
        self.output_limits = (lower, upper)

    def set_integral_limits(self, lower, upper):
        assert lower < upper, "Lower integral limit must be less than upper"
        self.integral_limits = (lower, upper)

    def run_pid(self, target, current, dt):
        if dt <= 0.0:
            raise ValueError("dt must be > 0")

        error = target - current

        # --- Proportional
        P = self.kp * error

        # --- Integral with anti-windup
        if self.reset_integral:
            self.integral_sum = 0.0
            self.reset_integral = False
        self.integral_sum += error * dt
        self.integral_sum = np.clip(self.integral_sum, *self.integral_limits)
        I = self.ki * self.integral_sum

        # --- Derivative (on measurement)
        derivative_input = (current - self.last_input) / dt
        D = -self.kd * derivative_input  # Negative sign since d(measurement)/dt opposes error

        # --- Compute output
        output = P + I + D
        output = np.clip(output, *self.output_limits)

        # --- Save state
        self.previous_error = error
        self.last_input = current

        return output
