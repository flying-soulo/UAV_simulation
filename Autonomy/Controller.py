# Autonomy/Controller/controller_manager.py
from Autonomy.fw_controller import FixedWingController
from Autonomy.quad_controller import QuadController
import numpy as np
from Global.simdata import ControllerFlags, UAVState, TargetSetpoints, ControlOutputs

class ControllerManager:
    def __init__(self, dt):
        self.fw_controller = FixedWingController(dt)
        self.quad_controller = QuadController(dt)
        self.output : ControlOutputs = ControlOutputs()

    def run(self, current_state: UAVState, target: TargetSetpoints, flags: ControllerFlags):

        # run controller
        match(flags.current_mode.upper()):
            case "FW":
                self.output.fw = self.fw_controller.run(current_state, target.fw, flags)

                self.output.quad.pitch = 0
                self.output.quad.roll = 0
                self.output.quad.throttle = 0
                self.output.quad.yaw = 0

            case "QD":
                self.output.quad = self.quad_controller.run(current_state, target.quad)

                self.output.fw.aileron = 0
                self.output.fw.elevator = 0
                self.output.fw.rudder = 0
                self.output.fw.throttle = 0

            case "TRANSITION":
                self.output.quad.pitch = 0
                self.output.quad.roll = 0
                self.output.quad.throttle = 0
                self.output.quad.yaw = 0
                self.output.fw.aileron = 0
                self.output.fw.elevator = 0
                self.output.fw.rudder = 0
                self.output.fw.throttle = 0

            case "SHUTDOWN":
                self.output.quad.pitch = 0
                self.output.quad.roll = 0
                self.output.quad.throttle = 0
                self.output.quad.yaw = 0
                self.output.fw.aileron = 0
                self.output.fw.elevator = 0
                self.output.fw.rudder = 0
                self.output.fw.throttle = 0

        return self.output
