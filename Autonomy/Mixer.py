import numpy as np
from Global.utils import linear_scale
import Global.configs as configs
from Global.simdata import ActuatorOutputs, ControlOutputs

"""
Mixer class for converting control surface and motor commands to PWM signals.
This class is designed to work with different flight modes, including quadPlane.
Output will be in the range of 1000 to 2000 microseconds.
"""


class Mixer:
    def __init__(self):
        self.output: ActuatorOutputs = ActuatorOutputs()
        self.input: ControlOutputs = ControlOutputs()

    def run(self, mode: str, input: ControlOutputs)-> ActuatorOutputs:
        self.input = input
        match (mode.upper()):
            case "AUTO":
                self.output.fw.throttle = linear_scale(
                    self.input.fw.throttle,
                    in_min=configs.stick_input_min,
                    in_max=configs.stick_input_max,
                    out_min=configs.PWM_min,
                    out_max=configs.PWM_max,
                )

                # Assign values to the 3 control surfaces (indices 0 to 2)
                self.output.fw.aileron = linear_scale(
                    self.input.fw.aileron,
                    in_min=configs.stick_input_min,
                    in_max=configs.stick_input_max,
                    out_min=configs.PWM_min,
                    out_max=configs.PWM_max,
                )
                self.output.fw.elevator = linear_scale(
                    self.input.fw.elevator,
                    in_min=configs.stick_input_min,
                    in_max=configs.stick_input_max,
                    out_min=configs.PWM_min,
                    out_max=configs.PWM_max,
                )
                self.output.fw.rudder = linear_scale(
                    self.input.fw.rudder,
                    in_min=configs.stick_input_min,
                    in_max=configs.stick_input_max,
                    out_min=configs.PWM_min,
                    out_max=configs.PWM_max,
                )

                self.output.quad.motor1 = 0
                self.output.quad.motor2 = 0
                self.output.quad.motor3 = 0
                self.output.quad.motor4 = 0

            case "SHUTDOWN":
                self.output.quad.motor1 = 0
                self.output.quad.motor2 = 0
                self.output.quad.motor3 = 0
                self.output.quad.motor4 = 0
                self.output.fw.throttle = 0
                self.output.fw.aileron = 0
                self.output.fw.elevator = 0
                self.output.fw.rudder = 0

            # case "manual":
            #     self.motor = mixerinput[0:5]
            #     self.ctrl_surface = mixerinput[5:]
            #     self.motor_pwm[0:4] = [0] * 4
            #     self.motor_pwm[4] = linear_scale(
            #         self.motor[4],
            #         in_min=configs.stick_input_min,
            #         in_max=configs.stick_input_min,
            #         out_min=configs.PWM_min,
            #         out_max=configs.PWM_max,
            #     )

            #     # Assign values to the 3 control surfaces (indices 0 to 2)
            #     self.ctrl_surface_pwm[0] = linear_scale(
            #         self.ctrl_surface[0],
            #         in_min=configs.stick_input_min,
            #         in_max=configs.stick_input_min,
            #         out_min=configs.PWM_min,
            #         out_max=configs.PWM_max,
            #     )
            #     self.ctrl_surface_pwm[1] = linear_scale(
            #         self.ctrl_surface[1],
            #         in_min=configs.stick_input_min,
            #         in_max=configs.stick_input_min,
            #         out_min=configs.PWM_min,
            #         out_max=configs.PWM_max,
            #     )
            #     self.ctrl_surface_pwm[2] = linear_scale(
            #         self.ctrl_surface[2],
            #         in_min=configs.stick_input_min,
            #         in_max=configs.stick_input_min,
            #         out_min=configs.PWM_min,
            #         out_max=configs.PWM_max,
            #     )
            #     self.output = self.motor_pwm + self.ctrl_surface_pwm
            #     return self.output

        return self.output
