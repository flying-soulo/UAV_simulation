# Autonomy/Controller/controller_manager.py
from Autonomy.fw_controller import FixedWingController
from Autonomy.quad_controller import QuadController
import numpy as np
from Global.simdata import Controls_class, UAVState_class, Mission_track_data

class ControllerManager:
    def __init__(self, dt):
        self.fw_controller = FixedWingController(dt)
        self.quad_controller = QuadController(dt)
        self.output : Controls_class = Controls_class()

    def run(self, current_state: UAVState_class, target: Mission_track_data, mode: str):

        match(mode):
            case "FW":
                self.output.FW = self.fw_controller.run(current_state, target)

                self.output.Quad.pitch = 0
                self.output.Quad.roll = 0
                self.output.Quad.throttle = 0
                self.output.Quad.yaw = 0

            case "Quad":
                self.output.Quad = self.quad_controller.run(current_state, target)

                self.output.FW.aileron = 0
                self.output.FW.elevator = 0
                self.output.FW.rudder = 0
                self.output.FW.throttle = 0

        return self.output
