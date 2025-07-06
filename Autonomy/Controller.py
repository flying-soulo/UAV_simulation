# Autonomy/Controller/controller_manager.py
from Autonomy.fw_controller import FixedWingController
from Autonomy.quad_controller import QuadController
import numpy as np
from Global.simdata import Controls_class, UAVState_class, Target_data_struct, controller_flags_class

class ControllerManager:
    def __init__(self, dt):
        self.fw_controller = FixedWingController(dt)
        self.quad_controller = QuadController(dt)
        self.output : Controls_class = Controls_class()

    def run(self, current_state: UAVState_class, target: Target_data_struct, flags: controller_flags_class):

        # run controller
        match(flags.mode.upper()):
            case "FW":
                self.output.FW = self.fw_controller.run(current_state, target.FW, flags)

                self.output.Quad.pitch = 0
                self.output.Quad.roll = 0
                self.output.Quad.throttle = 0
                self.output.Quad.yaw = 0

            case "QD":
                self.output.Quad = self.quad_controller.run(current_state, target.Quad)

                self.output.FW.aileron = 0
                self.output.FW.elevator = 0
                self.output.FW.rudder = 0
                self.output.FW.throttle = 0

            case "TRANSITION":
                self.output.Quad.pitch = 0
                self.output.Quad.roll = 0
                self.output.Quad.throttle = 0
                self.output.Quad.yaw = 0
                self.output.FW.aileron = 0
                self.output.FW.elevator = 0
                self.output.FW.rudder = 0
                self.output.FW.throttle = 0

            case "SHUTDOWN":
                self.output.Quad.pitch = 0
                self.output.Quad.roll = 0
                self.output.Quad.throttle = 0
                self.output.Quad.yaw = 0
                self.output.FW.aileron = 0
                self.output.FW.elevator = 0
                self.output.FW.rudder = 0
                self.output.FW.throttle = 0

        return self.output
