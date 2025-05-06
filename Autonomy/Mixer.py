import numpy as np
from Global.Utils import linear_scale

"""
Mixer class for converting control surface and motor commands to PWM signals.
This class is designed to work with different flight modes, including QuadPlane.
Output will be in the range of 1100 to 2000 microseconds.
"""
class Mixer:
    def __init__(self):
        self.motor_pwm = [0] * 8
        self.ctrl_surface_pwm = [0] * 3
        self.PWM_output = (1100, 2000)
        self.output = [0] * 8

    def run(self, mode = "QuadPlane", mixerinput = [0] * 8):
        match(mode):
            case "QuadPlane":
                self.output = self.QuadPlaneMixer(mixerinput)
            case "shutdown":
                self.motor_pwm = [0] * 5
                self.ctrl_surface_pwm = [0] * 3
                self.output = [0] * 8
            case _:
                raise ValueError("Invalid mode")
        return self.output

    def QuadPlaneMixer(self, input):
        self.motor, self.ctrl_surface = input[0:5], input[5:]  # Adjusted to use only 4 motors.

        # Assign values to the 4 motors (indices 0 to 3)
        for i in range(len(self.motor)):
            self.motor_pwm[i] = linear_scale(self.motor[i], in_min=-10, in_max=10, out_min=self.PWM_output[0], out_max=self.PWM_output[1])

        # Assign values to the 3 control surfaces (indices 0 to 2)
        for i in range(len(self.ctrl_surface)):
            self.ctrl_surface_pwm[i] = linear_scale(self.ctrl_surface[i], in_min=-30, in_max=30, out_min=self.PWM_output[0], out_max=self.PWM_output[1])

        # Combine the motor and control surface PWM signals
        self.output = self.motor_pwm + self.ctrl_surface_pwm
        return self.output

