import numpy as np


class PID_class:
    def __init__(self):
        # PID gains (generic initial values for UAV attitude control)
        self.kp = 0.5
        self.ki = 0.01
        self.kd = 0.1

        # Process variables
        self.current = 0.0
        self.target = 0.0
        self.error = 0.0
        self.previous_error = 0.0

        # Integral accumulator and limits (anti-windup)
        self.integral_sum = 0.0
        self.higher_integral_limit = 10.0
        self.lower_integral_limit = -10.0
        self.reset_integral = False
        self.sample_time = 0.01  # 100 Hz control loop

        # PID term outputs
        self.P = 0.0
        self.I = 0.0
        self.D = 0.0

        # Output management
        self.output_limits = (
            -1.0,
            1.0,
        )  # Normalized control output for mixer or actuator
        self.last_output = 0.0
        self.output = 0.0
        self.last_input = 0.0

    def update_gains(self, kp=None, ki=None, kd=None):
        self.kp = kp if kp is not None else self.kp
        self.ki = ki if ki is not None else self.ki
        self.kd = kd if kd is not None else self.kd

    def run_pid(self, target, current, dt):
        self.target = target
        self.current = current
        self.sample_time = dt
        # Calculate error
        self.error = self.target - self.current

        # Calculate each term
        self.P = self.kp * self.error
        self.I = self.ki * self.integral(self.error)
        self.D = self.kd * self.derivative(self.error)

        # Compute total output and enforce limits
        self.output = self.P + self.I + self.D
        self.output = np.clip(self.output, *self.output_limits)

        # Update state for next iteration
        self.last_output = self.output
        self.last_input = self.current
        self.previous_error = self.error

        return self.output

    def integral(self, error):
        # Optionally reset the integral sum if needed
        if self.reset_integral:
            self.integral_sum = 0.0
            self.reset_integral = False

        # Accumulate error with sample time
        self.integral_sum += error * self.sample_time
        # Clamp the integral sum to prevent windup
        self.integral_sum = max(
            self.lower_integral_limit,
            min(self.integral_sum, self.higher_integral_limit),
        )
        return self.integral_sum

    def derivative(self, error):
        # Compute derivative using the change in error
        d = (error - self.previous_error) / self.sample_time
        return d
