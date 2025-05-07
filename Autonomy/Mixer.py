import numpy as np
from Global.Utils import linear_scale

"""
Mixer class for converting control surface and motor commands to PWM signals.
This class is designed to work with different flight modes, including QuadPlane.
Output will be in the range of 1100 to 2000 microseconds.
"""
class Mixer:
    def __init__(self):
        self.motor_pwm = [0] * 5
        self.ctrl_surface_pwm = [0] * 3
        self.PWM_output = (1100, 2000)
        self.output = [0] * 8

    def run(self, mode = "Auto", mixerinput = [0] * 8):
        match(mode):
            case "Auto":
                self.output = self.QuadPlaneMixer(mixerinput)
            case "shutdown":
                self.motor_pwm = [0] * 5
                self.ctrl_surface_pwm = [0] * 3
                self.output = [0] *8

            case "manual":
                self.motor = mixerinput[0:5]
                self.ctrl_surface = mixerinput[5:]
                # Combine the motor and control surface PWM signals
                # self.motor_pwm[0] = linear_scale(self.motor[0], in_min=-1, in_max=1, out_min=self.PWM_output[0], out_max=self.PWM_output[1])
                # self.motor_pwm[1] = linear_scale(self.motor[1], in_min=-1, in_max=1, out_min=self.PWM_output[0], out_max=self.PWM_output[1])
                # self.motor_pwm[2] = linear_scale(self.motor[2], in_min=-1, in_max=1, out_min=self.PWM_output[0], out_max=self.PWM_output[1])
                # self.motor_pwm[3] = linear_scale(self.motor[3], in_min=-1, in_max=1, out_min=self.PWM_output[0], out_max=self.PWM_output[1])
                self.motor_pwm[0:4] = [0]*4
                self.motor_pwm[4] = linear_scale(self.motor[4], in_min=0, in_max=100, out_min=self.PWM_output[0], out_max=self.PWM_output[1])

                # Assign values to the 3 control surfaces (indices 0 to 2)
                self.ctrl_surface_pwm[0] = linear_scale(self.ctrl_surface[0], in_min=-10, in_max=10, out_min=self.PWM_output[0], out_max=self.PWM_output[1])
                self.ctrl_surface_pwm[1] = linear_scale(self.ctrl_surface[1], in_min=-10, in_max=10, out_min=self.PWM_output[0], out_max=self.PWM_output[1])
                self.ctrl_surface_pwm[2] = linear_scale(self.ctrl_surface[2], in_min=-10, in_max=10, out_min=self.PWM_output[0], out_max=self.PWM_output[1])
                self.output = self.motor_pwm + self.ctrl_surface_pwm
                return self.output
            case _:
                raise ValueError("Invalid mode")
        return self.output

    def QuadPlaneMixer(self, input):
        self.motor, self.ctrl_surface = input[0:5], input[5:]  # Adjusted to use only 4 motors.

        # Assign values to the 4 motors (indices 0 to 3)
        self.motor_pwm[0] = linear_scale(self.motor[0], in_min=-1, in_max=1, out_min=self.PWM_output[0], out_max=self.PWM_output[1])
        self.motor_pwm[1] = linear_scale(self.motor[1], in_min=-1, in_max=1, out_min=self.PWM_output[0], out_max=self.PWM_output[1])
        self.motor_pwm[2] = linear_scale(self.motor[2], in_min=-1, in_max=1, out_min=self.PWM_output[0], out_max=self.PWM_output[1])
        self.motor_pwm[3] = linear_scale(self.motor[3], in_min=-1, in_max=1, out_min=self.PWM_output[0], out_max=self.PWM_output[1])
        self.motor_pwm[4] = linear_scale(self.motor[4], in_min=0, in_max=100, out_min=self.PWM_output[0], out_max=self.PWM_output[1])
        # Assign values to the 3 control surfaces (indices 0 to 2)

        self.ctrl_surface_pwm[0] = linear_scale(self.ctrl_surface[0], in_min=np.deg2rad(-30.0), in_max=np.deg2rad(30.0), out_min=self.PWM_output[0], out_max=self.PWM_output[1])
        self.ctrl_surface_pwm[1] = linear_scale(self.ctrl_surface[1], in_min=np.deg2rad(-30.0), in_max=np.deg2rad(30.0), out_min=self.PWM_output[0], out_max=self.PWM_output[1])
        self.ctrl_surface_pwm[2] = linear_scale(self.ctrl_surface[2], in_min=np.deg2rad(-30.0), in_max=np.deg2rad(30.0), out_min=self.PWM_output[0], out_max=self.PWM_output[1])

        # Combine the motor and control surface PWM signals
        self.output = self.motor_pwm + self.ctrl_surface_pwm
        return self.output

