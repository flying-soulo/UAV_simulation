import numpy as np
from Global.utils import linear_scale
import Global.configs as configs
from Global.simdata import Controls_class, Actuator_class

"""
Mixer class for converting control surface and motor commands to PWM signals.
This class is designed to work with different flight modes, including QuadPlane.
Output will be in the range of 1100 to 2000 microseconds.
"""


class Mixer:
    def __init__(self):
        self.output: Actuator_class = Actuator_class()
        self.input: Controls_class = Controls_class()

    def run(self, mode: str, input: Controls_class):
        self.input = input
        match (mode):
            case "Auto":
                self.output.FW.throttle = linear_scale(
                    self.input.FW.throttle,
                    in_min=configs.radio_throttle_inputs[0],
                    in_max=configs.radio_throttle_inputs[1],
                    out_min=configs.PWM_output[0],
                    out_max=configs.PWM_output[1],
                )

                # Assign values to the 3 control surfaces (indices 0 to 2)
                self.output.FW.aileron = linear_scale(
                    self.input.FW.aileron,
                    in_min=configs.radio_roll_inputs[0],
                    in_max=configs.radio_roll_inputs[1],
                    out_min=configs.PWM_output[0],
                    out_max=configs.PWM_output[1],
                )
                self.output.FW.elevator = linear_scale(
                    self.input.FW.elevator,
                    in_min=configs.radio_pitch_inputs[0],
                    in_max=configs.radio_pitch_inputs[1],
                    out_min=configs.PWM_output[0],
                    out_max=configs.PWM_output[1],
                )
                self.output.FW.rudder = linear_scale(
                    self.input.FW.rudder,
                    in_min=configs.radio_yaw_inputs[0],
                    in_max=configs.radio_yaw_inputs[1],
                    out_min=configs.PWM_output[0],
                    out_max=configs.PWM_output[1],
                )

                self.output.Quad.Motor1 = 0
                self.output.Quad.Motor2 = 0
                self.output.Quad.Motor3 = 0
                self.output.Quad.Motor4 = 0

            case "shutdown":
                self.output.Quad.Motor1 = 0
                self.output.Quad.Motor2 = 0
                self.output.Quad.Motor3 = 0
                self.output.Quad.Motor4 = 0
                self.output.FW.throttle = 0
                self.output.FW.aileron = 0
                self.output.FW.elevator = 0
                self.output.FW.rudder = 0

            # case "manual":
            #     self.motor = mixerinput[0:5]
            #     self.ctrl_surface = mixerinput[5:]
            #     self.motor_pwm[0:4] = [0] * 4
            #     self.motor_pwm[4] = linear_scale(
            #         self.motor[4],
            #         in_min=configs.radio_throttle_inputs[0],
            #         in_max=configs.radio_throttle_inputs[1],
            #         out_min=configs.PWM_output[0],
            #         out_max=configs.PWM_output[1],
            #     )

            #     # Assign values to the 3 control surfaces (indices 0 to 2)
            #     self.ctrl_surface_pwm[0] = linear_scale(
            #         self.ctrl_surface[0],
            #         in_min=configs.radio_roll_inputs[0],
            #         in_max=configs.radio_roll_inputs[1],
            #         out_min=configs.PWM_output[0],
            #         out_max=configs.PWM_output[1],
            #     )
            #     self.ctrl_surface_pwm[1] = linear_scale(
            #         self.ctrl_surface[1],
            #         in_min=configs.radio_pitch_inputs[0],
            #         in_max=configs.radio_pitch_inputs[1],
            #         out_min=configs.PWM_output[0],
            #         out_max=configs.PWM_output[1],
            #     )
            #     self.ctrl_surface_pwm[2] = linear_scale(
            #         self.ctrl_surface[2],
            #         in_min=configs.radio_yaw_inputs[0],
            #         in_max=configs.radio_yaw_inputs[1],
            #         out_min=configs.PWM_output[0],
            #         out_max=configs.PWM_output[1],
            #     )
            #     self.output = self.motor_pwm + self.ctrl_surface_pwm
            #     return self.output

        return self.output
