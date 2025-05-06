import numpy as np
from Global.Utils import linear_scale

"""
Mixer class for converting control surface and motor commands to PWM signals.
This class is designed to work with different flight modes, including QuadPlane.
Output will be in the range of 1100 to 2000 microseconds.
"""
class Mixer:
    def __init__(self):
        self.motor_pwm = [0] * 4
        self.ctrl_surface_pwm = [0] * 3
        self.PWM_output = (1100, 2000)
        self.output = [0] * 7

    def run(self, mode = "QuadPlane", mixerinput = [0] * 7):
        match(mode):
            case "QuadPlane":
                self.output = self.QuadPlaneMizer(mixerinput)
            case "shutdown":
                self.motor_pwm = [0] * 4
                self.ctrl_surface_pwm = [0] * 3
                self.output = [0] * 7
            case _:
                raise ValueError("Invalid mode")
        return self.output

    def QuadPlaneMizer(self, input):
        self.motor, self.ctrl_surface = input[0:4], input[4:7]

        self.motor_pwm[0] = linear_scale(self.motor[0], in_min=-10, in_max=10, out_min=self.PWM_output[0], out_max=self.PWM_output[1])
        self.motor_pwm[1] = linear_scale(self.motor[1], in_min=-10, in_max=10, out_min=self.PWM_output[0], out_max=self.PWM_output[1])
        self.motor_pwm[2] = linear_scale(self.motor[2], in_min=-10, in_max=10, out_min=self.PWM_output[0], out_max=self.PWM_output[1])
        self.motor_pwm[3] = linear_scale(self.motor[3], in_min=-10, in_max=10, out_min=self.PWM_output[0], out_max=self.PWM_output[1])
        self.motor_pwm[4] = linear_scale(self.motor[4], in_min=-100, in_max=100, out_min=self.PWM_output[0], out_max=self.PWM_output[1])

        self.ctrl_surface_pwm[0] = linear_scale(self.ctrl_surface[0], in_min=-30, in_max=30, out_min=self.PWM_output[0], out_max=self.PWM_output[1])
        self.ctrl_surface_pwm[1] = linear_scale(self.ctrl_surface[1], in_min=-30, in_max=30, out_min=self.PWM_output[0], out_max=self.PWM_output[1])
        self.ctrl_surface_pwm[2] = linear_scale(self.ctrl_surface[2], in_min=-30, in_max=30, out_min=self.PWM_output[0], out_max=self.PWM_output[1])
        self.output = self.motor_pwm + self.ctrl_surface_pwm
        return self.output


    # def QuadMixer(self, input):

    #     self.motor, self.ctrl_surface = input[0:4], input[4:7]

    #     self.motor_pwm[0] = linear_scale(self.motor[0], in_min=-10, in_max=10, out_min=self.PWM_output[0], out_max=self.PWM_output[1])
    #     self.motor_pwm[1] = linear_scale(self.motor[1], in_min=-10, in_max=10, out_min=self.PWM_output[0], out_max=self.PWM_output[1])
    #     self.motor_pwm[2] = linear_scale(self.motor[2], in_min=-10, in_max=10, out_min=self.PWM_output[0], out_max=self.PWM_output[1])
    #     self.motor_pwm[3] = linear_scale(self.motor[3], in_min=-10, in_max=10, out_min=self.PWM_output[0], out_max=self.PWM_output[1])

    #     self.output = self.motor_pwm + self.ctrl_surface_pwm
    #     return self.output

    # def PlaneMixer(self, input):
    #     self.motor, self.ctrl_surface = input[0:4], input[4:7]

    #     self.ctrl_surface_pwm[0] = linear_scale(self.ctrl_surface[0], in_min=-30, in_max=30, out_min=self.PWM_output[0], out_max=self.PWM_output[1])
    #     self.ctrl_surface_pwm[1] = linear_scale(self.ctrl_surface[1], in_min=-30, in_max=30, out_min=self.PWM_output[0], out_max=self.PWM_output[1])
    #     self.ctrl_surface_pwm[2] = linear_scale(self.ctrl_surface[2], in_min=-30, in_max=30, out_min=self.PWM_output[0], out_max=self.PWM_output[1])
    #     self.motor_pwm[4] = linear_scale(self.motor[0], in_min=-10, in_max=10, out_min=self.PWM_output[0], out_max=self.PWM_output[1])
    #     self.output = self.motor_pwm + self.ctrl_surface_pwm
    #     return self.output
