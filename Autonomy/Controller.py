# Autonomy/Controller/controller_manager.py
from Autonomy.fw_controller import FixedWingController
from Autonomy.quad_controller import QuadController
import numpy as np

class ControllerManager:
    def __init__(self, dt):
        self.fw = FixedWingController(dt)
        self.quad = QuadController(dt)
        self.motor = np.zeros(5)
        self.ctrl_surface = np.zeros(3)


    def run(self, current_state, nav_state):
        mode = nav_state.get("mode")
        if mode == "FW":
            output = self.fw.run(current_state, nav_state)
            aileron = output["aileron"]
            elevator = output["elevator"]
            rudder = output["rudder"]
            throttle = output["throttle"]
            self.motor = [0,0,0,0, throttle]
            self.ctrl_surface = [aileron, elevator, rudder]
            return self.motor, self.ctrl_surface

        elif mode == "QUAD":
            return self.quad.run(current_state, nav_state)

        else:
            raise ValueError(f"Unsupported mode: {mode}")
